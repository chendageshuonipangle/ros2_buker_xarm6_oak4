#!/usr/bin/env python3
"""Detect the target on the OAK NPU and publish camera-frame 3D points.

Publishes "label,confidence,x_mm,y_mm,z_mm" on /oak_result, which
target_to_base consumes to emit the ghost TF in the arm base frame.

CAM_B feeds both the detector and the stereo left input, so detections and
depth share one optical centre and need no extra cross-camera calibration.
Depth is reduced on the host (robust median inside a shrunken box) rather
than with SpatialLocationCalculator: that node ignores runtime ROI configs
on depthai 3.8 here, and blocks the camera when it waits for a message.

A detection is only published once it has held still for --stable-frames
consecutive frames within --stable-tolerance-mm. Neural detections jitter
by nature, and downstream this topic drives real arm motion, so a single
noisy frame must never be able to command a move.

Detection and depth run at --fps, while the preview is a separate low-rate
stream at --preview-fps. On USB2 the detection path saturates around 18 fps,
and measurements show the slow preview costs only about 0.4 fps of that, so
the GUI stays useful without starving the data path. Detection boxes are
cached and redrawn on the newest preview frame between preview updates.
"""

from __future__ import annotations

import argparse
from collections import deque

import cv2
import numpy as np
import depthai as dai
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from depthai_nodes.node import ParsingNeuralNetwork

CAM_SOCKET = dai.CameraBoardSocket.CAM_B
MONO_SOCKET = dai.CameraBoardSocket.CAM_C
MODEL_SIZE = (640, 640)
DEPTH_SIZE = (640, 400)
MIN_DEPTH_PIXELS = 10


def median_depth_mm(depth, box_px, shrink, depth_range):
    """Robust median depth (mm) inside the shrunken detection box."""
    if depth is None or depth.size == 0:
        return None
    height, width = depth.shape[:2]
    xmin, ymin, xmax, ymax = box_px
    cx = int((xmin + xmax) / 2.0)
    cy = int((ymin + ymax) / 2.0)
    half_w = max(1, int((xmax - xmin) * shrink))
    half_h = max(1, int((ymax - ymin) * shrink))
    patch = depth[
        max(0, cy - half_h): min(height, cy + half_h + 1),
        max(0, cx - half_w): min(width, cx + half_w + 1),
    ]
    values = patch[patch > 0].astype(np.float32)
    values = values[(values >= depth_range[0]) & (values <= depth_range[1])]
    if values.size < MIN_DEPTH_PIXELS:
        return None
    center = float(np.median(values))
    values = values[np.abs(values - center) < max(20.0, center * 0.15)]
    return float(np.median(values)) if values.size >= 5 else None


class OakPeachDetector(Node):
    WINDOW = "oak_peach_detector"

    def __init__(self, args):
        super().__init__("oak_peach_detector")
        self.args = args
        self.publisher = self.create_publisher(String, args.output_topic, 10)
        self.archive = dai.NNArchive(args.model)
        self.pipeline = None
        self.frames = 0
        self.published = 0
        self.rejected = 0
        self.overlay = []
        self.window_ready = False
        self._build_pipeline()
        # Host loop runs well above the device rate so no frame waits.
        self.timer = self.create_timer(1.0 / 60.0, self.poll)
        self.get_logger().info(
            f"publishing {args.output_topic}; label filter="
            f"{args.target_label or '<any>'}; depth window "
            f"{args.min_depth_mm}-{args.max_depth_mm} mm; "
            f"stability {args.stable_frames} frames within "
            f"{args.stable_tolerance_mm} mm"
        )

    def _build_pipeline(self):
        args = self.args
        self.pipeline = dai.Pipeline()
        cam_b = self.pipeline.create(dai.node.Camera).build(
            CAM_SOCKET, sensorFps=args.fps
        )
        cam_c = self.pipeline.create(dai.node.Camera).build(
            MONO_SOCKET, sensorFps=args.fps
        )
        # LETTERBOX keeps the full field of view; the default CROP mode
        # discards about 37 percent of the horizontal view.
        model_input = cam_b.requestOutput(
            size=MODEL_SIZE,
            type=dai.ImgFrame.Type.BGR888p,
            resizeMode=dai.ImgResizeMode.LETTERBOX,
            fps=args.fps,
        )
        nn = self.pipeline.create(ParsingNeuralNetwork).build(
            model_input, self.archive
        )
        nn.setNNArchive(self.archive, numShaves=args.num_shaves)
        try:
            parser = nn.getParser(0)
            parser.setConfidenceThreshold(args.confidence)
            # Without this the parser assumes 416x416 and scales NMS wrongly.
            parser.setInputImageSize(*MODEL_SIZE)
        except Exception as error:
            self.get_logger().warn(f"parser tuning failed: {error}")
        nn.input.setBlocking(False)
        nn.input.setMaxSize(1)

        stereo = self.pipeline.create(dai.node.StereoDepth)
        left = cam_b.requestOutput(
            DEPTH_SIZE, type=dai.ImgFrame.Type.NV12, fps=args.fps
        )
        right = cam_c.requestOutput(
            DEPTH_SIZE, type=dai.ImgFrame.Type.NV12, fps=args.fps
        )
        left.link(stereo.left)
        right.link(stereo.right)
        # DENSITY gives the best depth coverage at the same frame rate here.
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DENSITY)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        stereo.setDepthAlign(CAM_SOCKET)

        # Preview is its own low-rate stream so the GUI does not compete
        # with depth for USB2 bandwidth. Requesting the full-rate stream for
        # display is what previously collapsed the pipeline to 0.8 fps.
        if args.show:
            preview = cam_b.requestOutput(
                DEPTH_SIZE,
                type=dai.ImgFrame.Type.NV12,
                fps=max(1.0, args.preview_fps),
            )
            self.preview_queue = preview.createOutputQueue(
                maxSize=1, blocking=False
            )
        else:
            self.preview_queue = None
        self.nn_queue = nn.out.createOutputQueue(maxSize=1, blocking=False)
        self.depth_queue = stereo.depth.createOutputQueue(maxSize=1, blocking=False)

        self.pipeline.start()
        calib = self.pipeline.getDefaultDevice().readCalibration()
        intrinsics = np.array(
            calib.getCameraIntrinsics(CAM_SOCKET, DEPTH_SIZE[0], DEPTH_SIZE[1]),
            dtype=np.float64,
        )
        self.fx = intrinsics[0, 0]
        self.fy = intrinsics[1, 1]
        self.cx = intrinsics[0, 2]
        self.cy = intrinsics[1, 2]
        self.latest_depth = None
        self.preview_transform = None
        self.display = None
        self.display_count = 0
        # rolling window of recent camera-frame points for the stability gate
        self.history = deque(maxlen=self.args.stable_frames)
        self.stable = False
        self.overlay = []
        self.get_logger().info(
            f"pipeline started; fx={self.fx:.2f} fy={self.fy:.2f} "
            f"cx={self.cx:.2f} cy={self.cy:.2f}"
        )

    def poll(self):
        if self.pipeline is None or not self.pipeline.isRunning():
            return
        if self.preview_queue is not None:
            preview = self.preview_queue.tryGet()
            if preview is not None:
                frame = preview.getCvFrame()
                if frame.ndim == 2:
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                self.display = frame
                self.display_count += 1

        depth_message = self.depth_queue.tryGet()
        if depth_message is not None:
            self.latest_depth = depth_message.getFrame()
            # Depth carries its own transformation, so box remapping no
            # longer depends on a preview frame arriving.
            self.preview_transform = depth_message.getTransformation()

        frame = None
        if self.args.show and self.display is not None:
            frame = self.display
            if self.args.display_width > frame.shape[1]:
                scale = self.args.display_width / float(frame.shape[1])
                frame = cv2.resize(
                    frame,
                    (self.args.display_width, int(frame.shape[0] * scale)),
                    interpolation=cv2.INTER_LINEAR,
                )
            else:
                frame = frame.copy()
        hits = 0
        fresh = False
        detections = self.nn_queue.tryGet()
        if detections is not None:
            fresh = True
            self.overlay = []
        if detections is not None and not detections.detections:
            # target lost: forget the window so a stale point cannot persist
            self.history.clear()
            self.stable = False
        if detections is not None and self.preview_transform is not None:
            self.frames += 1
            nn_transform = detections.getTransformation()
            nn_w, nn_h = nn_transform.getSize()
            for det in detections.detections:
                if self.args.target_label and det.labelName != self.args.target_label:
                    continue
                box = dai.RotatedRect(
                    dai.Rect(
                        dai.Point2f(det.xmin * nn_w, det.ymin * nn_h),
                        dai.Point2f(det.xmax * nn_w, det.ymax * nn_h),
                    ),
                    0.0,
                )
                # Strict remap: NN is 640x640 letterboxed, depth is 640x400.
                mapped = nn_transform.remapRectTo(self.preview_transform, box)
                px0, py0, px1, py1 = mapped.getOuterRect()
                z_mm = median_depth_mm(
                    self.latest_depth,
                    (px0, py0, px1, py1),
                    self.args.roi_shrink,
                    (self.args.min_depth_mm, self.args.max_depth_mm),
                )
                if z_mm is None:
                    continue
                ucx = (px0 + px1) / 2.0
                vcy = (py0 + py1) / 2.0
                x_mm = (ucx - self.cx) * z_mm / self.fx
                y_mm = (vcy - self.cy) * z_mm / self.fy

                # Gate: only publish a point that has held still, so a
                # single jittery frame cannot command arm motion.
                self.history.append((x_mm, y_mm, z_mm))
                spread_mm = None
                self.stable = False
                if len(self.history) == self.history.maxlen:
                    points = np.array(self.history)
                    spread_mm = float(
                        np.linalg.norm(points.max(axis=0) - points.min(axis=0))
                    )
                    self.stable = spread_mm <= self.args.stable_tolerance_mm
                hits += 1
                if not self.stable:
                    self.rejected += 1
                    need = self.history.maxlen - len(self.history)
                    note = (
                        f"settling {len(self.history)}/{self.history.maxlen}"
                        if need > 0
                        else f"jitter {spread_mm:.0f}mm > "
                        f"{self.args.stable_tolerance_mm:.0f}mm"
                    )
                    self.overlay.append({
                        "box": (px0, py0, px1, py1),
                        "centre": (ucx, vcy),
                        "label": det.labelName,
                        "confidence": det.confidence,
                        "xyz": (x_mm, y_mm, z_mm),
                        "stable": False,
                        "note": note,
                    })
                    continue

                # Publish the window average: steadier than the newest frame.
                x_mm, y_mm, z_mm = np.array(self.history).mean(axis=0)
                message = String()
                message.data = (
                    f"{det.labelName},{det.confidence:.4f},"
                    f"{x_mm:.2f},{y_mm:.2f},{z_mm:.2f}"
                )
                self.publisher.publish(message)
                self.published += 1
                if self.published % 15 == 1:
                    self.get_logger().info(
                        f"{det.labelName} conf={det.confidence:.2f} "
                        f"camera XYZ=({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}) mm "
                        f"(stable, spread {spread_mm:.1f} mm)"
                    )
                self.overlay.append({
                    "box": (px0, py0, px1, py1),
                    "centre": (ucx, vcy),
                    "label": det.labelName,
                    "confidence": det.confidence,
                    "xyz": (x_mm, y_mm, z_mm),
                    "stable": True,
                    "note": f"spread {spread_mm:.0f}mm",
                })

        if frame is not None:
            hits = len(self.overlay)
            scale_x = frame.shape[1] / float(DEPTH_SIZE[0])
            scale_y = frame.shape[0] / float(DEPTH_SIZE[1])
            for item in self.overlay:
                bx0, by0, bx1, by1 = item["box"]
                bx0, bx1 = bx0 * scale_x, bx1 * scale_x
                by0, by1 = by0 * scale_y, by1 * scale_y
                ccx, ccy = item["centre"][0] * scale_x, item["centre"][1] * scale_y
                colour = (0, 255, 0) if item["stable"] else (0, 165, 255)
                cv2.rectangle(
                    frame, (int(bx0), int(by0)), (int(bx1), int(by1)), colour, 3
                )
                cv2.circle(frame, (int(ccx), int(ccy)), 5, (0, 0, 255), -1)
                cv2.putText(
                    frame,
                    f"{item['label']} {item['confidence']:.2f}",
                    (int(bx0), max(18, int(by0) - 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, colour, 2,
                )
                x_mm, y_mm, z_mm = item["xyz"]
                cv2.putText(
                    frame,
                    f"XYZ ({x_mm:.0f},{y_mm:.0f},{z_mm:.0f})mm",
                    (int(bx0), min(frame.shape[0] - 30, int(by1) + 26)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 255), 2,
                )
                cv2.putText(
                    frame, item["note"],
                    (int(bx0), min(frame.shape[0] - 8, int(by1) + 52)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2,
                )
            if hits and self.stable:
                banner, colour = "TARGET STABLE", (0, 255, 0)
            elif hits:
                banner, colour = "TARGET SETTLING", (0, 165, 255)
            else:
                banner, colour = "NO TARGET", (0, 165, 255)
            cv2.putText(
                frame, banner, (14, 42),
                cv2.FONT_HERSHEY_SIMPLEX, 1.1, colour, 3,
            )
            cv2.putText(
                frame,
                f"conf>={self.args.confidence:.2f} "
                f"depth {self.args.min_depth_mm:.0f}-{self.args.max_depth_mm:.0f}mm "
                f"stable {self.args.stable_frames}f/{self.args.stable_tolerance_mm:.0f}mm "
                f"nn~{self.args.fps:.0f}fps preview~{self.args.preview_fps:.0f}fps "
                f"published={self.published} held={self.rejected}",
                (14, frame.shape[0] - 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (210, 210, 210), 2,
            )
            if not self.window_ready:
                cv2.namedWindow(self.WINDOW, cv2.WINDOW_NORMAL)
                if self.args.fullscreen:
                    cv2.setWindowProperty(
                        self.WINDOW, cv2.WND_PROP_FULLSCREEN,
                        cv2.WINDOW_FULLSCREEN,
                    )
                self.window_ready = True
            cv2.imshow(self.WINDOW, frame)
            # f toggles fullscreen, q closes the preview but keeps publishing
            key = cv2.waitKey(1) & 0xFF
            if key == ord('f'):
                self.args.fullscreen = not self.args.fullscreen
                cv2.setWindowProperty(
                    self.WINDOW, cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_FULLSCREEN if self.args.fullscreen
                    else cv2.WINDOW_NORMAL,
                )
            elif key == ord('q'):
                self.args.show = False
                cv2.destroyWindow(self.WINDOW)
                self.window_ready = False
                self.get_logger().info("preview closed; detection continues")

    def destroy_node(self):
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass
        if self.args.show:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model",
        default="/home/qluo/ros2_ws/src/oak_yolo_py/oak_yolo_py/best_ckpt.rvc2.tar.xz",
    )
    parser.add_argument("--output-topic", default="/oak_result")
    parser.add_argument("--target-label", default="")
    parser.add_argument("--confidence", type=float, default=0.75)
    parser.add_argument(
        "--fps", type=float, default=18.0,
        help="detection and depth rate; USB2 saturates near 18",
    )
    parser.add_argument(
        "--preview-fps", type=float, default=5.0,
        help="GUI-only stream rate; kept low to protect bandwidth",
    )
    parser.add_argument(
        "--display-width", type=int, default=1280,
        help="upscale the preview to this width for readability",
    )
    parser.add_argument(
        "--fullscreen", nargs="?", const="true", default="true",
        help="open the preview fullscreen (true/false); press f to toggle",
    )
    parser.add_argument("--num-shaves", type=int, default=5)
    parser.add_argument("--roi-shrink", type=float, default=0.3)
    parser.add_argument("--min-depth-mm", type=float, default=150.0)
    parser.add_argument("--max-depth-mm", type=float, default=700.0)
    parser.add_argument(
        "--stable-frames", type=int, default=8,
        help="consecutive frames the point must hold still before publishing",
    )
    parser.add_argument(
        "--stable-tolerance-mm", type=float, default=15.0,
        help="maximum spread across that window",
    )
    parser.add_argument(
        "--show",
        nargs="?",
        const="true",
        default="true",
        help="open a preview window (true/false)",
    )
    # ros2 launch appends --ros-args; ignore what this tool does not define.
    args, _ = parser.parse_known_args(argv)
    args.show = str(args.show).strip().lower() in ("1", "true", "yes", "on")
    args.fullscreen = str(args.fullscreen).strip().lower() in (
        "1", "true", "yes", "on"
    )
    rclpy.init(args=None)
    node = OakPeachDetector(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
