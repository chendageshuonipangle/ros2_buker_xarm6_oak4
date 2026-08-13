#!/usr/bin/env python3
"""
path_recorder_node.py

Subscribes to /odom (nav_msgs/Odometry) and accumulates robot pose history.
Each pose is transformed from odom frame to map frame via TF2, so the
published /driven_path aligns with /waypoint_markers in RViz2.

Publishes nav_msgs/Path to /driven_path with RELIABLE + TRANSIENT_LOCAL QoS.

Parameters:
  min_dist_m   (float,  default 0.05):   min travel distance before new pose appended
  odom_topic   (string, default "odom"): odometry source topic
  target_frame (string, default "map"):  TF target frame for output path
  source_frame (string, default "odom"): TF source frame (must match odom header)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty

from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs  # noqa: F401  registers do_transform_pose


class PathRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__('path_recorder')

        self.declare_parameter('min_dist_m', 0.05)
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('source_frame', 'odom')

        self._min_dist: float = self.get_parameter('min_dist_m').get_parameter_value().double_value
        odom_topic: str = self.get_parameter('odom_topic').get_parameter_value().string_value
        self._target_frame: str = self.get_parameter('target_frame').get_parameter_value().string_value
        self._source_frame: str = self.get_parameter('source_frame').get_parameter_value().string_value

        # TF2 buffer and listener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._path = Path()
        self._path.header.frame_id = self._target_frame
        self._last_x: float | None = None
        self._last_y: float | None = None

        # Subscriber QoS: best-effort volatile is fine for odometry
        odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publisher QoS: RELIABLE + TRANSIENT_LOCAL so late-joining
        # RViz2 subscribers receive the full accumulated path immediately.
        path_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._pub = self.create_publisher(Path, 'driven_path', path_qos)
        self._sub = self.create_subscription(Odometry, odom_topic, self._odom_cb, odom_qos)
        self._srv = self.create_service(Empty, 'path_recorder/clear', self._clear_cb)

        self.get_logger().info(
            f'path_recorder started: odom={odom_topic}, '
            f'{self._source_frame}->{self._target_frame}, min_dist={self._min_dist}m'
        )

    def _clear_cb(self, _request, response):
        self._path.poses.clear()
        self._last_x = None
        self._last_y = None
        self._path.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._path)
        self.get_logger().info('path_recorder: driven_path cleared')
        return response

    def _odom_cb(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self._last_x is not None:
            dist = math.hypot(x - self._last_x, y - self._last_y)
            if dist < self._min_dist:
                return

        # Build a PoseStamped in source_frame (odom)
        pose_in_odom = PoseStamped()
        pose_in_odom.header.stamp = msg.header.stamp
        pose_in_odom.header.frame_id = self._source_frame
        pose_in_odom.pose = msg.pose.pose

        # Transform to target_frame (map)
        try:
            pose_in_map = self._tf_buffer.transform(
                pose_in_odom,
                self._target_frame,
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed, skipping pose: {e}', throttle_duration_sec=5.0)
            return

        self._last_x = x
        self._last_y = y

        self._path.header.stamp = pose_in_map.header.stamp
        self._path.poses.append(pose_in_map)
        self._pub.publish(self._path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathRecorderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
