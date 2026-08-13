#!/usr/bin/env python3
"""Test the calibration board with DepthAI's on-device AprilTag detector."""

from __future__ import annotations

import argparse
import time

import cv2
import depthai as dai

from .collect_eye_in_hand import camera_socket


def draw_detection(frame, detection):
    corners = (
        detection.topLeft,
        detection.topRight,
        detection.bottomRight,
        detection.bottomLeft,
    )
    points = [(round(point.x), round(point.y)) for point in corners]
    for index, point in enumerate(points):
        cv2.line(frame, point, points[(index + 1) % 4], (0, 220, 0), 2, cv2.LINE_AA)
    centre = (
        round((detection.topLeft.x + detection.bottomRight.x) / 2),
        round((detection.topLeft.y + detection.bottomRight.y) / 2),
    )
    cv2.putText(
        frame, str(detection.id), centre, cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 0), 2
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-socket", default="CAM_B", type=camera_socket)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=800)
    parser.add_argument("--fps", type=float, default=30.0)
    # 1000 us at ISO 100 renders this workspace almost black, which makes the
    # tool report zero tags for reasons unrelated to the detector settings.
    # Default to automatic exposure and only pin it when both flags are given.
    parser.add_argument(
        "--exposure-us",
        type=int,
        default=None,
        help="manual exposure in microseconds; omit for automatic exposure",
    )
    parser.add_argument(
        "--iso",
        type=int,
        default=None,
        help="manual ISO sensitivity; omit for automatic exposure",
    )
    args = parser.parse_args(argv)

    pipeline = dai.Pipeline()
    try:
        camera = pipeline.create(dai.node.Camera)
        manual_exposure = args.exposure_us is not None and args.iso is not None
        if manual_exposure:
            camera.initialControl.setManualExposure(args.exposure_us, args.iso)
        camera.build(args.camera_socket, sensorFps=args.fps)
        output = camera.requestOutput(
            (args.width, args.height), type=dai.ImgFrame.Type.GRAY8, fps=args.fps
        )

        april_tag = pipeline.create(dai.node.AprilTag)
        april_tag.initialConfig.setFamily(dai.AprilTagConfig.Family.TAG_36H11)
        april_tag.initialConfig.quadDecimate = 1
        april_tag.initialConfig.quadSigma = 0.0
        april_tag.initialConfig.refineEdges = True
        april_tag.initialConfig.decodeSharpening = 0.25
        april_tag.initialConfig.maxHammingDistance = 1
        output.link(april_tag.inputImage)
        april_tag.inputImage.setBlocking(False)

        image_queue = april_tag.passthroughInputImage.createOutputQueue(maxSize=2, blocking=False)
        tag_queue = april_tag.out.createOutputQueue(maxSize=2, blocking=False)
        pipeline.start()
        exposure_note = (
            f"manual exposure={args.exposure_us}us ISO={args.iso}"
            if manual_exposure
            else "automatic exposure"
        )
        print(
            f"DepthAI on-device AprilTag test started: TAG_36H11, {exposure_note}. "
            "Press Q or ESC to close."
        )
        cv2.namedWindow("OAK-D-SR DepthAI AprilTag board test", cv2.WINDOW_NORMAL)
        latest_tags = []
        while True:
            tag_message = tag_queue.tryGet()
            if tag_message is not None:
                latest_tags = tag_message.aprilTags
            image_message = image_queue.tryGet()
            if image_message is None:
                time.sleep(0.002)
                continue
            frame = cv2.cvtColor(image_message.getFrame(), cv2.COLOR_GRAY2BGR)
            for tag in latest_tags:
                draw_detection(frame, tag)
            color = (0, 220, 0) if len(latest_tags) >= 12 else (0, 200, 255)
            cv2.putText(
                frame,
                f"DepthAI TAG_36H11: {len(latest_tags)} tags seen (need 12+ for capture)",
                (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2,
            )
            cv2.imshow("OAK-D-SR DepthAI AprilTag board test", frame)
            if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q"), 27):
                break
    finally:
        try:
            pipeline.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
