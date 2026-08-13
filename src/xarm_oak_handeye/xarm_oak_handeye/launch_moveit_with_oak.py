#!/usr/bin/env python3
"""Start xArm6 MoveIt with the OAK-D-SR mount and calibrated camera envelope."""

from __future__ import annotations

import argparse
import json
import math
import subprocess
from pathlib import Path


def quaternion_to_rpy(quaternion):
    x, y, z, w = (float(value) for value in quaternion)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_sine = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, pitch_sine) if abs(pitch_sine) >= 1.0 else math.asin(pitch_sine)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result", required=True, type=Path, help="eye_in_hand_result.json")
    parser.add_argument("--robot-ip", default="192.168.1.242")
    parser.add_argument("--camera-safety-radius-m", type=float, default=0.065)
    parser.add_argument("--no-gui", action="store_true")
    args = parser.parse_args(argv)
    result = json.loads(args.result.read_text(encoding="ascii"))
    if result.get("mode") != "eye_in_hand":
        raise ValueError("the supplied result is not an eye-in-hand calibration")
    transform = result["tcp_to_camera"]
    xyz = transform["translation_m"]
    rpy = quaternion_to_rpy(transform["rotation_xyzw"])
    command = [
        "ros2", "launch", "xarm_moveit_config", "xarm6_moveit_realmove.launch.py",
        f"robot_ip:={args.robot_ip}",
        "add_gripper:=true",
        "add_oak_d_sr:=true",
        "add_oak_d_sr_camera_collision:=true",
        "oak_d_sr_camera_xyz:=\"{} {} {}\"".format(*xyz),
        "oak_d_sr_camera_rpy:=\"{} {} {}\"".format(*rpy),
        f"oak_d_sr_camera_safety_radius:={args.camera_safety_radius_m}",
    ]
    if args.no_gui:
        command.append("no_gui_ctrl:=true")
    print("Starting MoveIt with the calibrated OAK-D-SR collision model:")
    print(" ".join(command))
    raise SystemExit(subprocess.call(command))


if __name__ == "__main__":
    main()
