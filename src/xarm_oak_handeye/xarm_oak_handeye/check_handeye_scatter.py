#!/usr/bin/env python3
"""Quantify eye-in-hand accuracy from a stationary target seen at many poses.

Keep one target fixed in the workspace, move the arm to several distinct
poses, and record a sample at each. Every sample converts the camera-frame
detection into the base frame, so a perfect calibration would put all of
them on the same point. The spread of those points is the achievable
accuracy of the whole chain: forward kinematics, hand-eye transform and
stereo depth.
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
from pathlib import Path

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from .geometry import transform_from_ros


class HandEyeScatterCheck(Node):
    def __init__(self, args):
        super().__init__("check_handeye_scatter")
        self.args = args
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self, spin_thread=True)
        self.latest = None
        self.samples = []
        self.subscription = self.create_subscription(
            String, args.input_topic, self.callback, 10
        )

    def callback(self, message: String):
        try:
            label, confidence, x_mm, y_mm, z_mm = message.data.split(",")
        except ValueError:
            return
        if self.args.target_label and label != self.args.target_label:
            return
        self.latest = (
            label,
            float(confidence),
            np.array([float(x_mm), float(y_mm), float(z_mm)]) / 1000.0,
        )

    def record(self):
        """Average several consecutive detections, then map to the base frame."""
        if self.latest is None:
            return False, "no detection received yet"
        points_camera = []
        points_base = []
        tcp_positions = []
        deadline = self.get_clock().now() + Duration(seconds=self.args.settle)
        while self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest is None:
                continue
            label, confidence, point_camera = self.latest
            try:
                base_to_camera_msg = self.buffer.lookup_transform(
                    self.args.base_frame,
                    self.args.camera_frame,
                    Time(),
                    timeout=Duration(seconds=0.1),
                )
                base_to_tcp_msg = self.buffer.lookup_transform(
                    self.args.base_frame,
                    self.args.tcp_frame,
                    Time(),
                    timeout=Duration(seconds=0.1),
                )
            except Exception as error:
                return False, f"TF lookup failed: {error}"
            base_to_camera = transform_from_ros(base_to_camera_msg.transform)
            base_to_tcp = transform_from_ros(base_to_tcp_msg.transform)
            points_camera.append(point_camera)
            points_base.append(
                base_to_camera[:3, :3] @ point_camera + base_to_camera[:3, 3]
            )
            tcp_positions.append(base_to_tcp[:3, 3])

        if len(points_base) < 3:
            return False, f"only {len(points_base)} detections during settle window"

        points_base = np.array(points_base)
        jitter_mm = float(np.linalg.norm(points_base.std(axis=0)) * 1000.0)
        if jitter_mm > self.args.max_jitter_mm:
            return False, (
                f"target or arm still moving (jitter {jitter_mm:.1f} mm > "
                f"{self.args.max_jitter_mm:.1f} mm)"
            )

        self.samples.append({
            "point_base_m": points_base.mean(axis=0).tolist(),
            "point_camera_m": np.array(points_camera).mean(axis=0).tolist(),
            "tcp_base_m": np.array(tcp_positions).mean(axis=0).tolist(),
            "jitter_mm": jitter_mm,
            "detections": len(points_base),
        })
        return True, f"jitter {jitter_mm:.1f} mm over {len(points_base)} detections"

    def report(self):
        if len(self.samples) < 2:
            print("need at least 2 samples to report a spread")
            return
        points = np.array([s["point_base_m"] for s in self.samples])
        centroid = points.mean(axis=0)
        deviations = np.linalg.norm(points - centroid, axis=1) * 1000.0
        per_axis = points.std(axis=0) * 1000.0
        print()
        print(f"samples          : {len(points)}")
        print(f"centroid (m)     : [{centroid[0]:.4f}, {centroid[1]:.4f}, {centroid[2]:.4f}]")
        print(f"per-axis std (mm): x={per_axis[0]:.2f} y={per_axis[1]:.2f} z={per_axis[2]:.2f}")
        print(f"radial mean (mm) : {deviations.mean():.2f}")
        print(f"radial max  (mm) : {deviations.max():.2f}")
        print(f"radial rms  (mm) : {np.sqrt((deviations ** 2).mean()):.2f}")
        print()
        print("per-sample deviation from centroid:")
        for index, (sample, deviation) in enumerate(zip(self.samples, deviations), 1):
            tcp = sample["tcp_base_m"]
            camera = sample["point_camera_m"]
            print(
                f"  {index:2d}: {deviation:6.2f} mm  "
                f"cam_z={camera[2] * 1000:6.1f} mm  "
                f"tcp=[{tcp[0]:.3f}, {tcp[1]:.3f}, {tcp[2]:.3f}]"
            )
        if self.args.output:
            payload = {
                "base_frame": self.args.base_frame,
                "camera_frame": self.args.camera_frame,
                "centroid_base_m": centroid.tolist(),
                "radial_rms_mm": float(np.sqrt((deviations ** 2).mean())),
                "radial_max_mm": float(deviations.max()),
                "per_axis_std_mm": per_axis.tolist(),
                "samples": self.samples,
            }
            self.args.output.parent.mkdir(parents=True, exist_ok=True)
            self.args.output.write_text(json.dumps(payload, indent=2), encoding="ascii")
            print(f"\nsaved {self.args.output}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-frame", default="link_base")
    parser.add_argument("--tcp-frame", default="link_tcp")
    parser.add_argument("--camera-frame", default="oak_rgb_camera_optical_frame")
    parser.add_argument("--input-topic", default="/oak_result")
    parser.add_argument("--target-label", default="")
    parser.add_argument("--settle", type=float, default=1.0,
                        help="seconds of detections to average per sample")
    parser.add_argument("--max-jitter-mm", type=float, default=15.0,
                        help="reject a sample if the point moves more than this")
    parser.add_argument("--output", type=Path, default=None)
    args, _ = parser.parse_known_args(argv)

    rclpy.init(args=None)
    node = HandEyeScatterCheck(args)

    print("=" * 60)
    print("hand-eye scatter check")
    print("=" * 60)
    print("Keep the target fixed. Move the arm to a new pose, let it settle,")
    print("then press ENTER to record. Type r to report, q to finish.")
    print("=" * 60)

    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True
    )

    try:
        while True:
            command = input(f"[{len(node.samples)} samples] ENTER=record r=report q=quit: ")
            command = command.strip().lower()
            if command == "q":
                break
            if command == "r":
                node.report()
                continue
            ok, detail = node.record()
            if ok:
                point = node.samples[-1]["point_base_m"]
                print(
                    f"  recorded #{len(node.samples)}: base=["
                    f"{point[0]:.4f}, {point[1]:.4f}, {point[2]:.4f}] m ({detail})"
                )
            else:
                print(f"  rejected: {detail}")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.report()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
