#!/usr/bin/env python3
"""Publish the calibrated fixed transform from xArm TCP to OAK camera."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class HandEyeStaticTf(Node):
    def __init__(self, result: dict, parent_frame: str | None, child_frame: str | None):
        super().__init__("xarm_oak_handeye_static_tf")
        transform = result["tcp_to_camera"]
        parent = parent_frame or result["tcp_frame"]
        child = child_frame or result["camera_frame"]
        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = parent
        message.child_frame_id = child
        translation = transform["translation_m"]
        quaternion = transform["rotation_xyzw"]
        message.transform.translation.x = float(translation[0])
        message.transform.translation.y = float(translation[1])
        message.transform.translation.z = float(translation[2])
        message.transform.rotation.x = float(quaternion[0])
        message.transform.rotation.y = float(quaternion[1])
        message.transform.rotation.z = float(quaternion[2])
        message.transform.rotation.w = float(quaternion[3])
        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(message)
        self.get_logger().info(f"published fixed transform {parent} -> {child}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--parent-frame", default=None)
    parser.add_argument("--child-frame", default=None)
    # ros2 launch appends --ros-args; ignore what this tool does not define.
    args, _ = parser.parse_known_args(argv)
    result = json.loads(args.result.read_text(encoding="ascii"))
    rclpy.init(args=None)
    node = HandEyeStaticTf(result, args.parent_frame, args.child_frame)
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
