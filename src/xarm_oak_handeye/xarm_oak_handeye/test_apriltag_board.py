#!/usr/bin/env python3
"""Open the OAK-D-SR and visually verify the fixed AprilTag calibration board."""

from __future__ import annotations

import argparse
import time

import cv2
import depthai as dai
import numpy as np

from .collect_eye_in_hand import (
    DEFAULT_BOARD_SIZE_M,
    DEFAULT_TAG_SIZE_M,
    build_mono_stream,
    camera_socket,
    detect_board,
    dictionary_candidates,
    draw_frame_axes_if_visible,
)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-socket", default="CAM_B", type=camera_socket)
    parser.add_argument("--dictionary", default="APRILTAG_36h11")
    parser.add_argument("--tag-size-mm", type=float, default=DEFAULT_TAG_SIZE_M * 1000.0)
    parser.add_argument("--board-size-mm", type=float, default=DEFAULT_BOARD_SIZE_M * 1000.0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=800)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--detection-upscale", type=int, default=3)
    parser.add_argument("--min-grid-points", type=int, default=30)
    args = parser.parse_args(argv)

    tag_size_m = args.tag_size_mm / 1000.0
    board_size_m = args.board_size_mm / 1000.0
    candidates = dictionary_candidates(args.dictionary, tag_size_m, board_size_m)
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
        print(
            "OAK-D-SR AprilTag outer-square grid test started. "
            "GRAY8 800P automatic exposure. Press Q or ESC to close."
        )
        cv2.namedWindow("OAK-D-SR AprilTag board test", cv2.WINDOW_NORMAL)
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
                detection_upscale=args.detection_upscale,
                min_grid_points=args.min_grid_points,
            )
            display = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR) if frame.ndim == 2 else frame.copy()
            if observation is None:
                status, color = "Outer-square grid not yet stable", (0, 0, 230)
            else:
                for point in observation["grid_centers"]:
                    cv2.circle(display, tuple(np.round(point).astype(int)), 4, (0, 220, 0), 2)
                draw_frame_axes_if_visible(
                    display, camera_matrix, distortion, observation["rvec"], observation["tvec"], 0.04
                )
                status = (
                    f"Outer-square grid={observation['point_count']}/36 "
                    f"reprojection={observation['reprojection_error_px']:.3f}px"
                )
                color = (0, 220, 0)
            cv2.putText(display, status, (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
            cv2.putText(
                display,
                f"ColorCamera 800P auto exposure | detector {args.detection_upscale}x | Q quit",
                (12, 62),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
            )
            cv2.imshow("OAK-D-SR AprilTag board test", display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
    finally:
        try:
            pipeline.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
