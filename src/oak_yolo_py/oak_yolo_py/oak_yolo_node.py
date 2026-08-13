#!/usr/bin/env python3
"""OAK4-D-Pro board-side YOLO inference with RGB/depth ROS2 streaming.

The NN runs on the RVC4 device.  The host receives only the parsed detections,
RGB frames and aligned 16-bit depth frames, which keeps this node suitable for
AprilTag and xArm6 hand-eye calibration later.
"""

from __future__ import annotations

import collections
import json
import os
import tarfile
import time
from pathlib import Path

import cv2
import depthai as dai
import numpy as np
from depthai_nodes.node import ParsingNeuralNetwork

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


RGB_SOCKET = dai.CameraBoardSocket.CAM_A
LEFT_SOCKET = dai.CameraBoardSocket.CAM_B
RIGHT_SOCKET = dai.CameraBoardSocket.CAM_C
MODEL_SIZE = (640, 640)
FULL_SIZE = (1280, 800)
# RVC4 can run the yellow-peach model at roughly 60 FPS.  Keeping the full
# calibration RGB stream at 30 FPS prevents 1280x800 transport from slowing
# down the live inference preview and depth pipeline.
INFERENCE_FPS = 60
CALIBRATION_DATA_FPS = 30
CONFIDENCE_THRESHOLD = 0.70
POSE_SKELETON = (
    (15, 13), (13, 11), (16, 14), (14, 12), (11, 12), (5, 11), (6, 12),
    (5, 6), (5, 7), (6, 8), (7, 9), (8, 10), (1, 2), (0, 1), (0, 2),
    (1, 3), (2, 4), (3, 5), (4, 6),
)
POSE_KEYPOINT_NAMES = (
    "nose", "left_eye", "right_eye", "left_ear", "right_ear", "left_shoulder",
    "right_shoulder", "left_elbow", "right_elbow", "left_wrist", "right_wrist",
    "left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle",
)
RGB_WINDOW_NAME = "OAK4-D-Pro RGB + YOLO"
DEPTH_WINDOW_NAME = "OAK4-D-Pro peach centre depth (mm)"


def find_model_path() -> Path:
    """Find the RVC4 archive in source or installed package locations."""
    candidates = []
    if value := os.environ.get("OAK_MODEL_ARCHIVE"):
        candidates.append(Path(value))
    candidates.extend((
        Path("/home/qluo/best.rvc4.tar.xz"),
        Path("/home/qluo/yolov8l-pose.rvc4.tar.xz"),
        Path(__file__).parent / "best.rvc4.tar.xz",
        Path(__file__).parent / "yolov8l-pose.rvc4.tar.xz",
    ))
    try:
        share = Path(get_package_share_directory("oak_yolo_py"))
        candidates.append(share / "models" / "yolov8l-pose.rvc4.tar.xz")
    except Exception:
        pass
    for path in candidates:
        if path.is_file():
            return path
    raise FileNotFoundError("找不到 best.rvc4.tar.xz 或 yolov8l-pose.rvc4.tar.xz")


def read_model_spec(path: Path) -> dict:
    """Read NN Archive metadata so detection, segmentation and pose archives work."""
    with tarfile.open(path, "r:xz") as archive:
        config = json.load(archive.extractfile("config.json"))
    model = config["model"]
    shape = model["inputs"][0]["shape"]
    head = model.get("heads", [{}])[0]
    metadata = head.get("metadata", {})
    is_pose = bool(metadata.get("keypoints_outputs"))
    skeleton = tuple(tuple(edge) for edge in metadata.get("skeleton_edges", POSE_SKELETON)) if is_pose else ()
    return {
        "size": (int(shape[2]), int(shape[1])),
        "labels": tuple(metadata.get("classes", ("object",))),
        "is_pose": is_pose,
        "is_segmentation": bool(metadata.get("mask_outputs")),
        "skeleton": skeleton,
    }


class SpatialSmoother:
    def __init__(self, history_size: int = 15):
        self.history = collections.defaultdict(lambda: collections.deque(maxlen=history_size))
        self.last_seen: dict[str, float] = {}

    def update(self, key: str, xyz: tuple[int, int, int]) -> tuple[int, int, int]:
        now = time.monotonic()
        for old in [k for k, t in self.last_seen.items() if now - t > 5.0]:
            self.history.pop(old, None)
            self.last_seen.pop(old, None)
        x, y, z = xyz
        if z == 0 and not self.history[key]:
            return xyz
        if z > 0:
            self.history[key].append(xyz)
            self.last_seen[key] = now
        if not self.history[key]:
            return xyz
        values = np.asarray(self.history[key])
        return int(values[:, 0].mean()), int(values[:, 1].mean()), int(np.median(values[:, 2]))


def model_point_to_frame(
    x_norm: float, y_norm: float, width: int, height: int, model_size: tuple[int, int]
) -> tuple[float, float]:
    """Map a normalized model-input point to the centre-cropped camera frame."""
    source_ratio = width / height
    model_ratio = model_size[0] / model_size[1]
    if source_ratio > model_ratio:
        crop_width, crop_height = height * model_ratio, height
        offset_x, offset_y = (width - crop_width) / 2.0, 0.0
    else:
        crop_width, crop_height = width, width / model_ratio
        offset_x, offset_y = 0.0, (height - crop_height) / 2.0
    return offset_x + x_norm * crop_width, offset_y + y_norm * crop_height


def spatial_coords(
    depth: np.ndarray,
    bbox: tuple[float, float, float, float],
    intrinsics: list[float] | None = None,
    model_size: tuple[int, int] = MODEL_SIZE,
) -> tuple[int, int, int]:
    """Return XYZ in millimetres at a detection's visual centre.

    The model input is a center crop of the 1280x800 RGB stream.  Convert its
    normalized centre back into the aligned depth frame and use an 11x11 pixel
    median so an occasional invalid depth pixel does not affect the result.
    """
    if depth is None or depth.size == 0:
        return 0, 0, 0
    h, w = depth.shape[:2]
    xmin, ymin, xmax, ymax = bbox
    cx, cy = model_point_to_frame((xmin + xmax) / 2.0, (ymin + ymax) / 2.0, w, h, model_size)
    ix, iy = int(round(cx)), int(round(cy))
    radius = 5
    roi = depth[max(0, iy - radius):min(h, iy + radius + 1), max(0, ix - radius):min(w, ix + radius + 1)]
    valid = roi[(roi > 0) & (roi < 10000)]
    if valid.size == 0:
        return 0, 0, 0
    z = int(np.median(valid))
    if intrinsics is not None and len(intrinsics) >= 9:
        fx, fy = intrinsics[0], intrinsics[4]
        camera_cx, camera_cy = intrinsics[2], intrinsics[5]
    else:
        fx = fy = w * 0.75
        camera_cx, camera_cy = w / 2, h / 2
    return int((cx - camera_cx) * z / fx), int((cy - camera_cy) * z / fy), z


def image_msg(frame: np.ndarray, encoding: str, stamp, frame_id: str) -> Image:
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = frame.shape[:2]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = int(frame.strides[0])
    msg.data = frame.tobytes()
    return msg


def camera_info(calib, socket, width: int, height: int, stamp, frame_id: str) -> CameraInfo:
    info = CameraInfo()
    if stamp is not None:
        info.header.stamp = stamp
    info.header.frame_id = frame_id
    info.width, info.height = width, height
    try:
        k = np.asarray(calib.getCameraIntrinsics(socket, width, height), dtype=float)
        info.k = k.reshape(-1).tolist()
        info.p = [k[0, 0], 0.0, k[0, 2], 0.0, 0.0, k[1, 1], k[1, 2], 0.0, 0.0, 0.0, 1.0, 0.0]
        info.d = np.asarray(calib.getDistortionCoefficients(socket), dtype=float).reshape(-1).tolist()
        info.distortion_model = "plumb_bob"
    except Exception:
        info.distortion_model = "plumb_bob"
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    return info


class OakResultPublisher(Node):
    def __init__(self, calibration, show_window: bool, snapshot_dir: Path, model_spec: dict):
        super().__init__("oak_result_publisher")
        self.show_window = show_window
        self.snapshot_dir = snapshot_dir
        self.model_size = model_spec["size"]
        self.labels = model_spec["labels"]
        self.snapshot_dir.mkdir(parents=True, exist_ok=True)
        self.pub = self.create_publisher(String, "/oak_result", 10)
        self.pub_rgb = self.create_publisher(Image, "/oak/rgb/image_raw", qos_profile_sensor_data)
        self.pub_depth = self.create_publisher(Image, "/oak/depth/image_raw", qos_profile_sensor_data)
        self.pub_annotated = self.create_publisher(Image, "/oak/rgb/image_annotated", qos_profile_sensor_data)
        self.pub_pose = self.create_publisher(String, "/oak_pose", 10)
        self.pub_rgb_info = self.create_publisher(CameraInfo, "/oak/rgb/camera_info", qos_profile_sensor_data)
        self.pub_depth_info = self.create_publisher(CameraInfo, "/oak/depth/camera_info", qos_profile_sensor_data)
        self.rgb_info = camera_info(calibration, RGB_SOCKET, *FULL_SIZE, None, "oak_rgb_camera_optical_frame")
        # The stereo output is aligned to CAM_A, so it uses the RGB optical
        # frame and the same intrinsics for downstream AprilTag/PnP code.
        self.depth_info = camera_info(calibration, RGB_SOCKET, *FULL_SIZE, None, "oak_rgb_camera_optical_frame")
        self.calibration = calibration
        self.last_publish = 0.0
        self.smoother = SpatialSmoother()
        self.get_logger().info("发布: /oak_result, /oak/rgb/image_raw, /oak/depth/image_raw (16UC1), CameraInfo")

    def publish(self, rgb, depth, annotated, detections):
        now = time.monotonic()
        if now - self.last_publish < 1.0 / CALIBRATION_DATA_FPS:
            return
        self.last_publish = now
        stamp = self.get_clock().now().to_msg()
        self.pub_rgb.publish(image_msg(rgb, "bgr8", stamp, "oak_rgb_camera_optical_frame"))
        self.pub_depth.publish(image_msg(depth, "16UC1", stamp, "oak_rgb_camera_optical_frame"))
        self.pub_annotated.publish(image_msg(annotated, "bgr8", stamp, "oak_rgb_camera_optical_frame"))
        self.rgb_info.header.stamp = stamp
        self.depth_info.header.stamp = stamp
        self.pub_rgb_info.publish(self.rgb_info)
        self.pub_depth_info.publish(self.depth_info)
        for index, det in enumerate(detections):
            label = getattr(det, "labelName", "") or (self.labels[det.label] if det.label < len(self.labels) else str(det.label))
            xyz = self.smoother.update(
                f"{label}_{index}",
                spatial_coords(
                    depth,
                    (det.xmin, det.ymin, det.xmax, det.ymax),
                    self.depth_info.k,
                    self.model_size,
                ),
            )
            msg = String()
            msg.data = f"{label},{float(det.confidence):.4f},{xyz[0]},{xyz[1]},{xyz[2]}"
            self.pub.publish(msg)
            keypoints = []
            try:
                for kp in det.getKeypoints():
                    keypoints.append({
                        "x": float(kp.imageCoordinates.x),
                        "y": float(kp.imageCoordinates.y),
                        "confidence": float(kp.confidence),
                    })
            except (AttributeError, TypeError):
                pass
            pose = String()
            pose.data = json.dumps({
                "class": label,
                "confidence": float(det.confidence),
                "bbox": [float(det.xmin), float(det.ymin), float(det.xmax), float(det.ymax)],
                "xyz_mm": list(xyz),
                "keypoints": keypoints,
            }, separators=(",", ":"))
            self.pub_pose.publish(pose)

    def save_snapshot(self, rgb: np.ndarray, depth: np.ndarray, annotated: np.ndarray) -> Path:
        stamp = time.strftime("%Y%m%d_%H%M%S") + f"_{time.time_ns() % 1_000_000:06d}"
        cv2.imwrite(str(self.snapshot_dir / f"{stamp}_rgb.png"), rgb)
        cv2.imwrite(str(self.snapshot_dir / f"{stamp}_annotated.png"), annotated)
        cv2.imwrite(str(self.snapshot_dir / f"{stamp}_depth_mm.png"), depth)
        return self.snapshot_dir / f"{stamp}_rgb.png"


def save_calibration(calibration, destination: Path) -> None:
    """Persist factory intrinsics/extrinsics for AprilTag hand-eye scripts."""
    data = {"rgb_socket": "CAM_A", "left_socket": "CAM_B", "right_socket": "CAM_C"}
    for name, socket in (("rgb", RGB_SOCKET), ("left", LEFT_SOCKET), ("right", RIGHT_SOCKET)):
        try:
            data[name] = {
                "intrinsics_1280x800": calibration.getCameraIntrinsics(socket, *FULL_SIZE),
                "distortion": calibration.getDistortionCoefficients(socket),
            }
        except Exception as exc:
            data[name] = {"error": str(exc)}
    for name, source, target in (
        ("rgb_to_left", RGB_SOCKET, LEFT_SOCKET),
        ("rgb_to_right", RGB_SOCKET, RIGHT_SOCKET),
    ):
        try:
            data[name] = calibration.getCameraExtrinsics(source, target)
        except Exception as exc:
            data[name] = {"error": str(exc)}
    destination.write_text(json.dumps(data, indent=2), encoding="utf-8")


def main(args=None):
    rclpy.init(args=args)
    pipeline = None
    node = None
    try:
        model = find_model_path()
        model_spec = read_model_spec(model)
        print(f"加载 OAK4 板载模型: {model}")
        print(f"模型输入: {model_spec['size']} | 类别: {model_spec['labels']} | "
              f"pose={model_spec['is_pose']} seg={model_spec['is_segmentation']}")
        pipeline = dai.Pipeline()
        cam_rgb = pipeline.create(dai.node.Camera).build(RGB_SOCKET, sensorFps=INFERENCE_FPS)
        cam_left = pipeline.create(dai.node.Camera).build(LEFT_SOCKET, sensorFps=INFERENCE_FPS)
        cam_right = pipeline.create(dai.node.Camera).build(RIGHT_SOCKET, sensorFps=INFERENCE_FPS)
        # RVC4 Camera only accepts interleaved BGR888i; this matches all
        # current archives (best segmentation and yolov8l-pose).
        model_input = cam_rgb.requestOutput(
            model_spec["size"], type=dai.ImgFrame.Type.BGR888i, fps=INFERENCE_FPS
        )
        rgb_output = cam_rgb.requestOutput(
            FULL_SIZE, type=dai.ImgFrame.Type.BGR888i, fps=CALIBRATION_DATA_FPS
        )
        nn = pipeline.create(ParsingNeuralNetwork).build(model_input, dai.NNArchive(str(model)))
        stereo = pipeline.create(dai.node.StereoDepth)
        cam_left.requestFullResolutionOutput().link(stereo.left)
        cam_right.requestFullResolutionOutput().link(stereo.right)
        stereo.setDepthAlign(RGB_SOCKET)
        stereo.setRectification(True)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        q_preview = model_input.createOutputQueue(maxSize=4, blocking=False)
        q_rgb = rgb_output.createOutputQueue(maxSize=4, blocking=False)
        q_nn = nn.out.createOutputQueue(maxSize=4, blocking=False)
        q_depth = stereo.depth.createOutputQueue(maxSize=4, blocking=False)
        pipeline.start()
        calibration = pipeline.getDefaultDevice().readCalibration()
        node = OakResultPublisher(calibration, False, Path.home() / "oak_snapshots", model_spec)
        save_calibration(calibration, node.snapshot_dir / "oak_factory_calibration.json")
        node.declare_parameter("show_window", False)
        node.show_window = bool(node.get_parameter("show_window").value)
        node.display_fullscreen = node.show_window
        if node.show_window:
            # Fullscreen changes only the desktop presentation.  The camera
            # stream and the model input keep their original resolution.
            cv2.namedWindow(RGB_WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.setWindowProperty(RGB_WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            cv2.namedWindow(DEPTH_WINDOW_NAME, cv2.WINDOW_NORMAL)
        print("OAK4-D-Pro 板载推理已启动，主机正在接收 RGB/深度/检测结果")
        last_dets = []
        last_mask = None
        accepted_mask_indices = []
        nn_frame_times = collections.deque(maxlen=30)
        last_preview = last_rgb = last_depth = None
        empty_cycles = 0
        while pipeline.isRunning() and rclpy.ok():
            # Never block the host on one stream.  Networked OAK devices can
            # briefly deliver RGB, depth and NN packets on different cycles.
            in_preview = q_preview.tryGet()
            in_rgb = q_rgb.tryGet()
            in_depth = q_depth.tryGet()
            if in_preview is not None:
                last_preview = in_preview.getCvFrame()
            if in_rgb is not None:
                last_rgb = in_rgb.getCvFrame()
            if in_depth is not None:
                last_depth = in_depth.getFrame()
            if last_preview is None or last_rgb is None or last_depth is None:
                empty_cycles += 1
                if empty_cycles % 5000 == 0:
                    print(f"等待数据: preview={last_preview is not None}, rgb={last_rgb is not None}, depth={last_depth is not None}", flush=True)
                time.sleep(0.001)
                continue
            preview, rgb, depth = last_preview, last_rgb, last_depth
            result = q_nn.tryGet()
            if result is not None:
                nn_frame_times.append(time.monotonic())
                raw_dets = list(getattr(result, "detections", []))
                accepted_mask_indices = [
                    index for index, det in enumerate(raw_dets)
                    if float(det.confidence) >= CONFIDENCE_THRESHOLD
                ]
                last_dets = [raw_dets[index] for index in accepted_mask_indices]
                try:
                    last_mask = result.getCvSegmentationMask()
                except (AttributeError, RuntimeError):
                    last_mask = None
            annotated = preview.copy()
            if last_mask is not None and last_mask.size and accepted_mask_indices:
                mask = cv2.resize(last_mask, (annotated.shape[1], annotated.shape[0]), interpolation=cv2.INTER_NEAREST)
                foreground = np.isin(mask, accepted_mask_indices)
                overlay = np.zeros_like(annotated)
                overlay[:, :, 1] = 220
                annotated[foreground] = cv2.addWeighted(annotated, 0.55, overlay, 0.45, 0)[foreground]
            depth_centres = []
            for det in last_dets:
                centre_x = int((det.xmin + det.xmax) * model_spec["size"][0] / 2.0)
                centre_y = int((det.ymin + det.ymax) * model_spec["size"][1] / 2.0)
                _, _, centre_depth = spatial_coords(
                    depth,
                    (det.xmin, det.ymin, det.xmax, det.ymax),
                    node.depth_info.k,
                    model_spec["size"],
                )
                depth_x, depth_y = model_point_to_frame(
                    (det.xmin + det.xmax) / 2.0,
                    (det.ymin + det.ymax) / 2.0,
                    depth.shape[1],
                    depth.shape[0],
                    model_spec["size"],
                )
                depth_centres.append((int(depth_x), int(depth_y), centre_depth))

                # Segmentation mask provides the object outline.  Only mark
                # the visual centre and its local depth; do not draw a bbox.
                cv2.circle(annotated, (centre_x, centre_y), 5, (0, 0, 255), -1)
                if centre_depth > 0:
                    text = f"Conf: {det.confidence:.2f} | Z: {centre_depth} mm"
                else:
                    text = f"Conf: {det.confidence:.2f} | Z: N/A"
                cv2.putText(annotated, text, (centre_x + 8, centre_y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
                cv2.putText(annotated, text, (centre_x + 8, centre_y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                try:
                    keypoints = det.getKeypoints()
                except (AttributeError, TypeError):
                    keypoints = []
                for kp in keypoints:
                    if kp.confidence >= 0.4:
                        cv2.circle(
                            annotated,
                            (int(kp.imageCoordinates.x * model_spec["size"][0]), int(kp.imageCoordinates.y * model_spec["size"][1])),
                            4,
                            (0, 0, 255),
                            -1,
                        )
                for first, second in model_spec["skeleton"]:
                    if first < len(keypoints) and second < len(keypoints):
                        kp1, kp2 = keypoints[first], keypoints[second]
                        if kp1.confidence >= 0.4 and kp2.confidence >= 0.4:
                            cv2.line(
                                annotated,
                                (int(kp1.imageCoordinates.x * model_spec["size"][0]), int(kp1.imageCoordinates.y * model_spec["size"][1])),
                                (int(kp2.imageCoordinates.x * model_spec["size"][0]), int(kp2.imageCoordinates.y * model_spec["size"][1])),
                                (255, 255, 0),
                                2,
                            )
            depth_vis = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
            for depth_x, depth_y, centre_depth in depth_centres:
                cv2.circle(depth_vis, (depth_x, depth_y), 6, (255, 255, 255), 2)
                if centre_depth > 0:
                    cv2.putText(depth_vis, f"{centre_depth} mm", (depth_x + 8, depth_y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            if len(nn_frame_times) >= 2:
                elapsed = nn_frame_times[-1] - nn_frame_times[0]
                npu_fps = (len(nn_frame_times) - 1) / elapsed if elapsed > 0 else 0.0
            else:
                npu_fps = 0.0
            cv2.putText(
                annotated,
                f"RVC4 NPU: {npu_fps:.1f} FPS | Dets (>= {CONFIDENCE_THRESHOLD:.2f}): {len(last_dets)}",
                (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
            )
            node.publish(rgb, depth, annotated, last_dets)
            try:
                rclpy.spin_once(node, timeout_sec=0.0)
            except RCLError:
                # A supervisor (or Ctrl+C) can invalidate the ROS context
                # while the device stream is still unwinding.
                break
            if node.show_window:
                cv2.imshow(RGB_WINDOW_NAME, annotated)
                cv2.imshow(DEPTH_WINDOW_NAME, depth_vis)
                key = cv2.waitKey(1) & 0xFF
                if key == ord(" "):
                    saved = node.save_snapshot(rgb, depth, annotated)
                    print(f"已保存配对采样: {saved}", flush=True)
                if key in (ord("f"), ord("F")):
                    node.display_fullscreen = not node.display_fullscreen
                    fullscreen = cv2.WINDOW_FULLSCREEN if node.display_fullscreen else cv2.WINDOW_NORMAL
                    cv2.setWindowProperty(RGB_WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, fullscreen)
                if key in (ord("q"), 27):
                    break
    finally:
        if node is not None:
            node.destroy_node()
        if pipeline is not None:
            pipeline.stop()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
