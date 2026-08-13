#!/usr/bin/env python3
"""Collect eye-in-hand samples without commanding the xArm."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import depthai as dai
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from .geometry import (
    mean_transform,
    rotation_angle_deg,
    transform_from_ros,
    transform_from_rvec_tvec,
)


DEFAULT_TAG_SIZE_M = 0.0165
DEFAULT_BOARD_SIZE_M = 0.13365
GRID_SIZE = (6, 6)
# The grid is formed by the centres of the 36 printed black tag squares. Its
# pitch is 16.5 mm tag width plus the 6.93 mm clear gap.
DEFAULT_GRID_PITCH_M = (DEFAULT_BOARD_SIZE_M - DEFAULT_TAG_SIZE_M) / (GRID_SIZE[0] - 1)
# DFOPTIX's printed tag orientation is rotated 180 degrees relative to the
# corner convention emitted by OpenCV's GridBoard for its physical ID layout.
# Apply this to the object corners, while image corners remain detector order.
DFOPTIX_OBJECT_CORNER_ORDER = (2, 3, 0, 1)
# A 6x6 GridBoard needs 36 distinct marker IDs.  16h5 and 25h9 contain too
# few IDs, so including them would produce an invalid/repeated-ID board.
APRILTAG_DICTIONARIES = ("APRILTAG_36h11", "APRILTAG_36h10")
# The quadrilateral the detector returns is the outer edge of a 36h11 tag's
# black frame, which spans 8 modules: the 6x6 data core plus one border module
# per side. The detector needs roughly 5 px per module to decode reliably and
# 8 px per module for stable corner refinement, so that black square must cover
# about 40 px in the raw image, and ideally 64 px.
TAG_MODULES_PER_SIDE = 8.0
MIN_PIXELS_PER_MODULE = 5.0
GOOD_PIXELS_PER_MODULE = 8.0


def camera_socket(value: str) -> dai.CameraBoardSocket:
    try:
        return getattr(dai.CameraBoardSocket, value.upper())
    except AttributeError as exc:
        raise argparse.ArgumentTypeError(f"unknown OAK camera socket: {value}") from exc


def build_mono_stream(pipeline, socket, width: int, height: int, fps: float):
    """Build a GRAY8 automatic-exposure path sized for a USB2 link.

    The OV9782 pair is monochrome-friendly and AprilTag detection only ever
    uses luminance, so requesting GRAY8 instead of a BGR888 preview cuts the
    USB payload to one third.  Measured on this USB2 (HIGH speed) link at
    1280x800: BGR preview delivered 13.0 fps with 420 ms of frame latency,
    while GRAY8 delivers 29.7 fps with 40 ms.  The old latency was the reason
    a pose had to be held for a long time before TF and image agreed.
    """
    camera = pipeline.create(dai.node.Camera).build(socket, sensorFps=fps)
    output = camera.requestOutput((width, height), type=dai.ImgFrame.Type.GRAY8, fps=fps)
    # Keep only the newest image.  A calibration sample must never be taken
    # from a frame that was queued while the operator was moving the arm.
    return output.createOutputQueue(maxSize=1, blocking=False)


def dictionary_from_name(name: str):
    normalized = name.upper()
    if not normalized.startswith("DICT_"):
        normalized = "DICT_" + normalized
    try:
        return normalized, cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, normalized))
    except AttributeError as exc:
        choices = ", ".join(item for item in dir(cv2.aruco) if item.startswith("DICT_APRILTAG"))
        raise ValueError(f"unknown dictionary {name}; available AprilTag dictionaries: {choices}") from exc


def dictionary_candidates(name: str, tag_size_m: float, board_extent_m: float):
    names = APRILTAG_DICTIONARIES if name.upper() == "AUTO" else (name,)
    candidates = []
    for item in names:
        normalized, dictionary = dictionary_from_name(item)
        board, gap = make_board(dictionary, tag_size_m, board_extent_m)
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        parameters.adaptiveThreshWinSizeMin = 3
        # The printed 16.5 mm tags occupy only about 40 px at the working
        # distance. These settings were verified against a raw OAK-D-SR frame.
        parameters.adaptiveThreshWinSizeMax = 61
        parameters.adaptiveThreshWinSizeStep = 4
        parameters.adaptiveThreshConstant = 5
        parameters.minMarkerPerimeterRate = 0.01
        parameters.maxMarkerPerimeterRate = 8.0
        parameters.polygonalApproxAccuracyRate = 0.03
        candidates.append((normalized, cv2.aruco.ArucoDetector(dictionary, parameters), board, gap))
    return candidates


def make_board(dictionary, tag_size_m: float, board_extent_m: float):
    columns, rows = GRID_SIZE
    gap = (board_extent_m - columns * tag_size_m) / (columns - 1)
    if gap < 0:
        raise ValueError("board extent is smaller than six tag widths")
    # DFOPTIX Tag6-150-16.5mm prints IDs 0..5 on the physical bottom row and
    # increments upward. OpenCV's default GridBoard does the opposite, so pass
    # the real row-major, top-to-bottom ID layout explicitly.
    row_starts = range(columns * rows - columns, -1, -columns)
    ids = np.concatenate([
        np.arange(start, start + columns, dtype=np.int32) for start in row_starts
    ])
    return cv2.aruco.GridBoard(GRID_SIZE, tag_size_m, gap, dictionary, ids), gap


def reprojection_error(object_points, image_points, rvec, tvec, camera_matrix, distortion) -> float:
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, distortion)
    delta = projected.reshape(-1, 2) - np.asarray(image_points).reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum(delta * delta, axis=1))))


def match_board_image_points(board, corners, ids):
    """Return DFOPTIX board geometry matched to detector-order image corners."""
    object_points, image_points = board.matchImagePoints(corners, ids)
    object_points = np.asarray(object_points, dtype=np.float32).reshape(-1, 4, 3)
    object_points = object_points[:, DFOPTIX_OBJECT_CORNER_ORDER, :].reshape(-1, 1, 3)
    return object_points, np.asarray(image_points, dtype=np.float32).reshape(-1, 1, 2)


def detect_markers(gray, detector, detection_upscale: int = 1):
    """Detect small printed tags and return corners in original-image pixels."""
    if detection_upscale < 1:
        raise ValueError("detection_upscale must be at least one")
    if detection_upscale == 1:
        return detector.detectMarkers(gray)
    enlarged = cv2.resize(
        gray,
        None,
        fx=detection_upscale,
        fy=detection_upscale,
        interpolation=cv2.INTER_CUBIC,
    )
    corners, ids, rejected = detector.detectMarkers(enlarged)
    corners = [np.asarray(corner, dtype=np.float32) / detection_upscale for corner in corners]
    rejected = [np.asarray(corner, dtype=np.float32) / detection_upscale for corner in rejected]
    return corners, ids, rejected


def square_records(quads):
    """Return centre/shape measurements for AprilTag detector quadrilaterals."""
    records = []
    for quad in quads:
        points = np.asarray(quad, dtype=np.float32).reshape(4, 2)
        edge_lengths = np.linalg.norm(points - np.roll(points, -1, axis=0), axis=1)
        minimum = float(np.min(edge_lengths))
        if minimum <= 0.0:
            continue
        records.append({
            "center": points.mean(axis=0),
            "side": float(np.mean(edge_lengths)),
            "aspect": float(np.max(edge_lengths) / minimum),
        })
    return records


def detect_board(
    frame,
    candidates,
    camera_matrix,
    distortion,
    locked_dictionary=None,
    detection_upscale: int = 1,
    min_grid_points: int = 30,
):
    """Estimate board pose from the 6x6 AprilTag outer-square centre grid.

    The printed payloads are too small to decode every frame at the installed
    distance. A few decoded IDs establish grid orientation, then all detected
    tag-square centres (decoded and rejected) are matched to the 6x6 lattice.
    This produces chessboard-like, stable point correspondences without
    treating the encoded tag payload as a chessboard pattern.
    """
    gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    observations = []
    for dictionary_name, detector, board, _ in candidates:
        if locked_dictionary and dictionary_name != locked_dictionary:
            continue
        decoded_corners, ids, rejected = detect_markers(gray, detector, detection_upscale)
        if ids is None or len(ids) < 4:
            continue
        decoded_records = square_records(decoded_corners)
        if len(decoded_records) != len(ids):
            continue

        # Board IDs increase from the physical bottom row to the top row.
        ids_flat = ids.reshape(-1)
        lattice_seeds = np.asarray(
            [[marker_id % GRID_SIZE[0], GRID_SIZE[1] - 1 - marker_id // GRID_SIZE[0]]
             for marker_id in ids_flat],
            dtype=np.float32,
        )
        image_seeds = np.asarray([record["center"] for record in decoded_records], dtype=np.float32)
        homography, inliers = cv2.findHomography(lattice_seeds, image_seeds, cv2.RANSAC, 2.0)
        if homography is None or inliers is None or int(inliers.sum()) < 4:
            continue

        median_side = float(np.median([record["side"] for record in decoded_records]))
        all_records = square_records(decoded_corners + rejected)
        valid_records = [
            record for record in all_records
            if 0.5 * median_side < record["side"] < 1.6 * median_side and record["aspect"] < 1.6
        ]
        if len(valid_records) < min_grid_points:
            continue

        grid_indices = [(column, row) for row in range(GRID_SIZE[1]) for column in range(GRID_SIZE[0])]
        predicted_centres = cv2.perspectiveTransform(
            np.asarray(grid_indices, dtype=np.float32).reshape(-1, 1, 2), homography
        ).reshape(-1, 2)
        neighbour_distances = []
        for row in range(GRID_SIZE[1]):
            for column in range(GRID_SIZE[0]):
                index = row * GRID_SIZE[0] + column
                if column + 1 < GRID_SIZE[0]:
                    neighbour_distances.append(
                        np.linalg.norm(predicted_centres[index] - predicted_centres[index + 1])
                    )
                if row + 1 < GRID_SIZE[1]:
                    neighbour_distances.append(
                        np.linalg.norm(predicted_centres[index] - predicted_centres[index + GRID_SIZE[0]])
                    )
        match_radius = 0.42 * float(np.median(neighbour_distances))

        used_record_indices = set()
        matched_indices = []
        matched_centres = []
        for index, predicted in enumerate(predicted_centres):
            distances = [
                np.linalg.norm(record["center"] - predicted) if candidate not in used_record_indices else np.inf
                for candidate, record in enumerate(valid_records)
            ]
            record_index = int(np.argmin(distances))
            if distances[record_index] > match_radius:
                continue
            used_record_indices.add(record_index)
            matched_indices.append(grid_indices[index])
            matched_centres.append(valid_records[record_index]["center"])

        if len(matched_centres) < min_grid_points:
            continue
        pitch_m = (DEFAULT_BOARD_SIZE_M - DEFAULT_TAG_SIZE_M) / (GRID_SIZE[0] - 1)
        object_points = np.asarray(
            [[column * pitch_m, row * pitch_m, 0.0] for column, row in matched_indices], dtype=np.float32
        ).reshape(-1, 1, 3)
        image_points = np.asarray(matched_centres, dtype=np.float32).reshape(-1, 1, 2)
        found, rvec, tvec = cv2.solvePnP(
            object_points, image_points, camera_matrix, distortion, flags=cv2.SOLVEPNP_ITERATIVE
        )
        if not found or float(np.asarray(tvec, dtype=float).reshape(3)[2]) <= 0:
            continue
        observations.append({
            "dictionary_name": dictionary_name,
            "pixels_per_module": median_side / TAG_MODULES_PER_SIDE,
            "corners": decoded_corners,
            "ids": ids,
            "grid_centers": image_points.reshape(-1, 2),
            "grid_indices": matched_indices,
            "point_count": len(matched_centres),
            "decoded_tag_count": len(ids),
            "object_points": object_points,
            "image_points": image_points,
            "rvec": rvec,
            "tvec": tvec,
            "camera_to_board": transform_from_rvec_tvec(rvec, tvec),
            "reprojection_error_px": reprojection_error(
                object_points, image_points, rvec, tvec, camera_matrix, distortion
            ),
        })
    if not observations:
        return None
    return min(observations, key=lambda item: (-item["point_count"], item["reprojection_error_px"]))


def draw_frame_axes_if_visible(
    image, camera_matrix, distortion, rvec, tvec, length_m: float
) -> None:
    """Draw pose axes only when every endpoint lands inside the image.

    cv2.drawFrameAxes logs a warning per call when an axis tip projects out of
    frame, which floods the console once the board nears an edge. The drawing
    is cosmetic, so skip it instead of emitting the warning.
    """
    height, width = image.shape[:2]
    points = np.float32([[0, 0, 0], [length_m, 0, 0], [0, length_m, 0], [0, 0, length_m]])
    projected, _ = cv2.projectPoints(points, rvec, tvec, camera_matrix, distortion)
    projected = projected.reshape(-1, 2)
    if not np.all(np.isfinite(projected)):
        return
    inside = (
        (projected[:, 0] >= 0)
        & (projected[:, 0] < width)
        & (projected[:, 1] >= 0)
        & (projected[:, 1] < height)
    )
    if not np.all(inside):
        return
    cv2.drawFrameAxes(image, camera_matrix, distortion, rvec, tvec, length_m)


def ros_time_from_depthai_frame(message) -> Time:
    """Convert an OAK host-clock frame stamp into ROS system time.

    DepthAI's ``getTimestamp`` is relative to ``dai::Clock::now`` (the host
    monotonic clock), whereas xArm publishes /tf using Unix system time.
    Sample the host clock offset immediately when the frame arrives so TF is
    queried at the actual image exposure time, not at the later keypress.
    """
    system_ns = time.time_ns()
    monotonic_ns = time.monotonic_ns()
    frame_monotonic_ns = int(round(message.getTimestamp().total_seconds() * 1_000_000_000))
    return Time(nanoseconds=frame_monotonic_ns + system_ns - monotonic_ns)


def tcp_is_stationary(
    tf_buffer: Buffer,
    base_frame: str,
    tcp_frame: str,
    image_stamp: Time,
    window_s: float,
    translation_tolerance_m: float,
    rotation_tolerance_deg: float,
) -> bool:
    """Return whether the TCP barely moved over the window ending at the image.

    Comparing the pose at exposure time against the pose one window earlier
    detects residual motion directly, so the operator no longer has to guess
    how long to wait after stopping the arm.
    """
    earlier_stamp = Time(nanoseconds=image_stamp.nanoseconds - int(window_s * 1_000_000_000))
    if earlier_stamp.nanoseconds <= 0:
        return False
    current = lookup_base_to_tcp(tf_buffer, base_frame, tcp_frame, image_stamp)
    earlier = lookup_base_to_tcp(tf_buffer, base_frame, tcp_frame, earlier_stamp)
    translation = float(np.linalg.norm(current[:3, 3] - earlier[:3, 3]))
    rotation = rotation_angle_deg(current[:3, :3], earlier[:3, :3])
    return translation <= translation_tolerance_m and rotation <= rotation_tolerance_deg


def lookup_base_to_tcp(
    tf_buffer: Buffer, base_frame: str, tcp_frame: str, image_stamp: Time
) -> np.ndarray:
    transform = tf_buffer.lookup_transform(
        base_frame, tcp_frame, image_stamp, timeout=Duration(seconds=0.25)
    )
    return transform_from_ros(transform.transform)


class SampleCollector:
    def __init__(self, args, camera_matrix, distortion, queue, candidates, tf_buffer, tf_node):
        self.args = args
        self.camera_matrix = camera_matrix
        self.distortion = distortion
        self.queue = queue
        self.candidates = candidates
        self.tf_buffer = tf_buffer
        self.tf_node = tf_node
        self.base_to_tcp: list[np.ndarray] = []
        self.camera_to_board: list[np.ndarray] = []
        self.errors: list[float] = []
        self.marker_counts: list[int] = []
        self.image_timestamps_ns: list[int] = []
        self.capture_images: list[np.ndarray] = []
        self.dictionary_name = None if args.dictionary.upper() == "AUTO" else candidates[0][0]

    def capture(self) -> tuple[bool, str]:
        # Flush the displayed/pre-keypress image.  The next packet was exposed
        # after SPACE was pressed and is therefore the sample candidate.
        while self.queue.tryGet() is not None:
            pass
        observations = []
        rejected = {
            "no_frame": 0,
            "board": 0,
            "grid": 0,
            "reprojection": 0,
            "tf": 0,
            "moving": 0,
        }
        capture_dictionary = self.dictionary_name
        deadline = time.monotonic() + self.args.capture_timeout
        while len(observations) < self.args.frames_per_sample and time.monotonic() < deadline:
            message = self.queue.tryGet()
            if message is None:
                rejected["no_frame"] += 1
                time.sleep(0.002)
                continue
            image_stamp = ros_time_from_depthai_frame(message)
            frame = message.getCvFrame()
            observation = detect_board(
                frame,
                self.candidates,
                self.camera_matrix,
                self.distortion,
                capture_dictionary,
                self.args.detection_upscale,
                self.args.min_grid_points,
            )
            if observation is None:
                rejected["board"] += 1
                continue
            if capture_dictionary is None:
                capture_dictionary = observation["dictionary_name"]
            if observation["point_count"] < self.args.min_grid_points:
                rejected["grid"] += 1
                continue
            if observation["reprojection_error_px"] > self.args.max_reprojection_error_px:
                rejected["reprojection"] += 1
                continue
            try:
                if self.args.settle_window > 0.0 and not tcp_is_stationary(
                    self.tf_buffer,
                    self.args.base_frame,
                    self.args.tcp_frame,
                    image_stamp,
                    self.args.settle_window,
                    self.args.settle_translation_mm / 1000.0,
                    self.args.settle_rotation_deg,
                ):
                    rejected["moving"] += 1
                    continue
                observation["base_to_tcp"] = lookup_base_to_tcp(
                    self.tf_buffer, self.args.base_frame, self.args.tcp_frame, image_stamp
                )
            except Exception:
                rejected["tf"] += 1
                continue
            observation["image_timestamp_ns"] = image_stamp.nanoseconds
            observation["frame"] = frame
            observations.append(observation)
        if len(observations) != self.args.frames_per_sample:
            causes = ", ".join(
                f"{name}={count}" for name, count in rejected.items() if count
            )
            return False, (
                f"only {len(observations)}/{self.args.frames_per_sample} valid frames"
                f" ({causes or 'no accepted observation'})"
            )

        self.base_to_tcp.append(mean_transform([item["base_to_tcp"] for item in observations]))
        self.camera_to_board.append(mean_transform([item["camera_to_board"] for item in observations]))
        self.errors.append(float(np.mean([item["reprojection_error_px"] for item in observations])))
        self.marker_counts.append(int(round(np.mean([item["point_count"] for item in observations]))))
        self.image_timestamps_ns.append(
            int(round(np.mean([item["image_timestamp_ns"] for item in observations])))
        )
        self.capture_images.append(observations[len(observations) // 2]["frame"])
        if self.dictionary_name is None:
            self.dictionary_name = capture_dictionary
        return True, f"accepted {len(observations)} frames"

    def save(self, directory: Path, metadata: dict) -> Path:
        directory.mkdir(parents=True, exist_ok=True)
        images_dir = directory / "images"
        images_dir.mkdir(exist_ok=True)
        for index, image in enumerate(self.capture_images, start=1):
            cv2.imwrite(str(images_dir / f"sample_{index:02d}.png"), image)
        path = directory / "eye_in_hand_samples.npz"
        np.savez_compressed(
            path,
            base_to_tcp=np.asarray(self.base_to_tcp, dtype=float),
            camera_to_board=np.asarray(self.camera_to_board, dtype=float),
            reprojection_error_px=np.asarray(self.errors, dtype=float),
            marker_count=np.asarray(self.marker_counts, dtype=int),
            image_timestamp_ns=np.asarray(self.image_timestamps_ns, dtype=np.int64),
            metadata_json=np.asarray(json.dumps(metadata, ensure_ascii=True)),
        )
        return path


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-frame", default="link_base")
    parser.add_argument("--tcp-frame", default="link_tcp")
    parser.add_argument("--camera-frame", default="oak_rgb_camera_optical_frame")
    parser.add_argument("--camera-socket", default="CAM_B", type=camera_socket)
    parser.add_argument("--dictionary", default="APRILTAG_36h11", help="AprilTag family, or AUTO to detect 36h11/36h10")
    parser.add_argument("--tag-size-mm", type=float, default=DEFAULT_TAG_SIZE_M * 1000.0)
    parser.add_argument("--board-size-mm", type=float, default=DEFAULT_BOARD_SIZE_M * 1000.0)
    # CAM_B on this OAK-D-SR supports this BGR output configuration.
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=800)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument(
        "--detection-upscale",
        type=int,
        default=2,
        help="integer image enlargement used only for AprilTag detection",
    )
    parser.add_argument("--min-grid-points", type=int, default=30)
    parser.add_argument("--max-reprojection-error-px", type=float, default=0.6)
    parser.add_argument(
        "--frames-per-sample",
        type=int,
        default=3,
        help="fresh synchronized frames to average at one stopped robot pose (default: 3)",
    )
    parser.add_argument("--capture-timeout", type=float, default=3.0)
    parser.add_argument(
        "--settle-window",
        type=float,
        default=0.25,
        help="seconds of TF history that must show a stationary TCP; 0 disables the check",
    )
    parser.add_argument("--settle-translation-mm", type=float, default=0.5)
    parser.add_argument("--settle-rotation-deg", type=float, default=0.1)
    parser.add_argument("--output-dir", type=Path, default=None)
    return parser


def main(argv=None):
    args = make_parser().parse_args(argv)
    tag_size_m = args.tag_size_mm / 1000.0
    board_size_m = args.board_size_mm / 1000.0
    candidates = dictionary_candidates(args.dictionary, tag_size_m, board_size_m)
    tag_gap_m = candidates[0][3]
    output_dir = args.output_dir or (
        Path.home() / "oak_handeye" / time.strftime("session_%Y%m%d_%H%M%S")
    )

    rclpy.init(args=None)
    tf_node = rclpy.create_node("xarm_oak_handeye_tf_listener")
    tf_buffer = Buffer()
    # /tf must be received continuously.  A frame is normally 0.1-0.3 s old
    # on arrival, so pumping callbacks only after SPACE loses its historical
    # transform and rejects an otherwise valid calibration image.
    listener = TransformListener(tf_buffer, tf_node, spin_thread=True)
    pipeline = dai.Pipeline()
    try:
        queue = build_mono_stream(
            pipeline, args.camera_socket, args.width, args.height, args.fps
        )
        pipeline.start()
        calibration = pipeline.getDefaultDevice().readCalibration()
        camera_matrix = np.asarray(
            calibration.getCameraIntrinsics(args.camera_socket, args.width, args.height), dtype=float
        )
        distortion = np.asarray(
            calibration.getDistortionCoefficients(args.camera_socket), dtype=float
        ).reshape(-1, 1)
        collector = SampleCollector(
            args, camera_matrix, distortion, queue, candidates, tf_buffer, tf_node
        )
        print("xArm6 OAK-D-SR eye-in-hand collection started")
        print(
            "target: 6x6 AprilTag outer-square centre grid, "
            f"pitch={DEFAULT_GRID_PITCH_M * 1000:.3f} mm"
        )
        print(f"frames: {args.base_frame} -> {args.tcp_frame}; camera: {args.camera_frame}")
        print("camera: Camera CAM_B GRAY8 800P with automatic exposure")
        print(
            "Fix the board in the workspace. Stop the arm at a pose and press SPACE; "
            "residual motion is rejected automatically."
        )
        print("Collect at least 12 poses with strong orientation changes. Press S to save, Q or ESC to exit.")
        cv2.namedWindow("OAK-D-SR AprilTag hand-eye", cv2.WINDOW_NORMAL)

        last_display = None
        while True:
            message = queue.tryGet()
            if message is None:
                time.sleep(0.002)
                continue
            frame = message.getCvFrame()
            observation = detect_board(
                frame,
                candidates,
                camera_matrix,
                distortion,
                collector.dictionary_name,
                args.detection_upscale,
                args.min_grid_points,
            )
            display = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR) if frame.ndim == 2 else frame.copy()
            if observation is not None:
                for point in observation["grid_centers"]:
                    cv2.circle(display, tuple(np.round(point).astype(int)), 4, (0, 220, 0), 2)
                draw_frame_axes_if_visible(
                    display, camera_matrix, distortion, observation["rvec"], observation["tvec"], 0.04
                )
                pixels_per_module = observation["pixels_per_module"]
                status = (
                    f"outer-square grid={observation['point_count']}/36 "
                    f"reproj={observation['reprojection_error_px']:.3f}px "
                    f"px/module={pixels_per_module:.1f} "
                    f"samples={len(collector.base_to_tcp)}"
                )
                color = (0, 220, 0) if pixels_per_module >= GOOD_PIXELS_PER_MODULE else (0, 190, 220)
                if pixels_per_module < MIN_PIXELS_PER_MODULE:
                    cv2.putText(
                        display,
                        "board too small in frame: move camera closer",
                        (12, 86),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 190, 220),
                        2,
                    )
            else:
                status = f"board not accepted samples={len(collector.base_to_tcp)}"
                color = (0, 0, 230)
            cv2.putText(display, status, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
            cv2.putText(display, "SPACE capture | S save | Q quit", (12, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.imshow("OAK-D-SR AprilTag hand-eye", display)
            last_display = display
            key = cv2.waitKey(1) & 0xFF
            if key == ord(" "):
                accepted, message_text = collector.capture()
                print(f"sample {len(collector.base_to_tcp)}: {message_text}")
            elif key in (ord("s"), ord("S")):
                if len(collector.base_to_tcp) < 8:
                    print("need at least 8 accepted poses before saving")
                    continue
                metadata = {
                    "schema": "xarm_oak_handeye_eye_in_hand_v1",
                    "mode": "eye_in_hand",
                    "base_frame": args.base_frame,
                    "tcp_frame": args.tcp_frame,
                    "camera_frame": args.camera_frame,
                    "camera_socket": str(args.camera_socket).split(".")[-1],
                    "image_size": [args.width, args.height],
                    "detection_upscale": args.detection_upscale,
                    "camera_matrix": camera_matrix.tolist(),
                    "distortion": distortion.reshape(-1).tolist(),
                    "dictionary": "APRILTAG_36h11_outer_square_grid",
                    "grid_size": list(GRID_SIZE),
                    "tag_size_m": tag_size_m,
                    "tag_gap_m": tag_gap_m,
                    "grid_pitch_m": DEFAULT_GRID_PITCH_M,
                    "board_extent_m": board_size_m,
                    "captured_at_unix_s": time.time(),
                }
                saved = collector.save(output_dir, metadata)
                print(f"saved {len(collector.base_to_tcp)} samples: {saved}")
            elif key in (ord("q"), ord("Q"), 27):
                break
    finally:
        try:
            pipeline.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()
        # Stop the dedicated TF executor before shutting down rclpy so a
        # normal Q/ESC exit does not emit an executor-shutdown traceback.
        if hasattr(listener, "executor"):
            listener.executor.shutdown()
            listener.dedicated_listener_thread.join(timeout=1.0)
        listener.unregister()
        tf_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
