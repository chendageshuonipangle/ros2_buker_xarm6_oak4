#!/usr/bin/env python3
"""Transform OAK detection points to xArm base using the calibrated TF tree."""

from __future__ import annotations

import argparse

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from .geometry import transform_from_ros


class OakTargetToBase(Node):
    def __init__(self, args):
        super().__init__("oak_target_to_base")
        self.args = args
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self, spin_thread=True)
        self.broadcaster = TransformBroadcaster(self)
        self.publisher = self.create_publisher(PoseStamped, args.output_topic, 10)
        self.subscription = self.create_subscription(String, args.input_topic, self.callback, 10)
        self.get_logger().info(
            f"listening {args.input_topic}; transforming {args.camera_frame} -> {args.base_frame}"
        )

    def callback(self, message: String):
        try:
            label, confidence, x_mm, y_mm, z_mm = message.data.split(",")
            if self.args.target_label and label != self.args.target_label:
                return
            point_camera = [float(x_mm) / 1000.0, float(y_mm) / 1000.0, float(z_mm) / 1000.0]
            if point_camera[2] <= 0:
                return
            transform_msg = self.buffer.lookup_transform(
                self.args.base_frame,
                self.args.camera_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
            base_to_camera = transform_from_ros(transform_msg.transform)
            point_base = base_to_camera[:3, :3] @ point_camera + base_to_camera[:3, 3]
            output = PoseStamped()
            output.header.stamp = self.get_clock().now().to_msg()
            output.header.frame_id = self.args.base_frame
            output.pose.position.x = float(point_base[0])
            output.pose.position.y = float(point_base[1])
            output.pose.position.z = float(point_base[2])
            output.pose.orientation.w = 1.0
            self.publisher.publish(output)
            tf_message = TransformStamped()
            tf_message.header = output.header
            tf_message.child_frame_id = f"target_{label}"
            tf_message.transform.translation.x = float(point_base[0])
            tf_message.transform.translation.y = float(point_base[1])
            tf_message.transform.translation.z = float(point_base[2])
            tf_message.transform.rotation.w = 1.0
            self.broadcaster.sendTransform(tf_message)
        except Exception as exc:
            self.get_logger().warn(f"cannot transform OAK target: {exc}", throttle_duration_sec=2.0)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-frame", default="link_base")
    parser.add_argument("--camera-frame", default="oak_rgb_camera_optical_frame")
    parser.add_argument("--input-topic", default="/oak_result")
    parser.add_argument("--output-topic", default="/target_pose_in_base")
    parser.add_argument("--target-label", default="")
    # ros2 launch appends --ros-args; ignore what this tool does not define.
    args, _ = parser.parse_known_args(argv)
    rclpy.init(args=None)
    node = OakTargetToBase(args)
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
