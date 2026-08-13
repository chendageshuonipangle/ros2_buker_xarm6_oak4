#!/usr/bin/env python3
"""
waypoint_marker_node.py

Reads waypoints from ~/ros2_ws/maps/dadian.txt (ros2 topic echo /clicked_point
format) and publishes them as visualization_msgs/MarkerArray to /waypoint_markers
with RELIABLE + TRANSIENT_LOCAL QoS.

Markers:
  - Sphere per waypoint (color: green for middle, red for last, blue for first)
  - Text label above each sphere showing the index (WP01, WP02, ...)
  - LINE_STRIP connecting all waypoints in order

The array is published once on startup and then latched (re-published every 5 s
to survive RViz2 restarts without restarting the node).
"""

import re
import math
import os
from pathlib import Path as FilePath

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


DADIAN_TXT = os.path.expanduser('~/ros2_ws/maps/dadian.txt')

_RE_X = re.compile(r'^\s*x:\s*([-\d.e+]+)', re.MULTILINE)
_RE_Y = re.compile(r'^\s*y:\s*([-\d.e+]+)', re.MULTILINE)


def parse_waypoints(path: str) -> list[tuple[float, float]]:
    text = FilePath(path).read_text()
    # Split on the separator lines (---) to get individual messages
    blocks = re.split(r'^---\s*$', text, flags=re.MULTILINE)
    pts: list[tuple[float, float]] = []
    for block in blocks:
        xs = _RE_X.findall(block)
        ys = _RE_Y.findall(block)
        if xs and ys:
            pts.append((float(xs[0]), float(ys[0])))
    return pts


class WaypointMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__('waypoint_marker')

        self.declare_parameter('dadian_txt', DADIAN_TXT)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_interval_s', 5.0)

        txt_path: str = self.get_parameter('dadian_txt').get_parameter_value().string_value
        self._frame_id: str = self.get_parameter('frame_id').get_parameter_value().string_value
        interval: float = self.get_parameter('publish_interval_s').get_parameter_value().double_value

        try:
            self._waypoints = parse_waypoints(txt_path)
        except Exception as e:
            self.get_logger().error(f'Failed to parse {txt_path}: {e}')
            self._waypoints = []

        self.get_logger().info(
            f'waypoint_marker: loaded {len(self._waypoints)} waypoints from {txt_path}'
        )

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(MarkerArray, 'waypoint_markers', qos)
        self._txt_path = txt_path

        self._pub.publish(self._build_marker_array())
        self._timer = self.create_timer(interval, self._publish)

    # ------------------------------------------------------------------
    def _publish(self) -> None:
        try:
            new_pts = parse_waypoints(self._txt_path)
        except Exception as e:
            self.get_logger().warn(f'waypoint_marker: re-read failed: {e}')
            return
        if new_pts != self._waypoints:
            self._waypoints = new_pts
            self.get_logger().info(
                f'waypoint_marker: reloaded {len(self._waypoints)} waypoints'
            )
        self._pub.publish(self._build_marker_array())

    # ------------------------------------------------------------------
    def _build_marker_array(self) -> MarkerArray:
        array = MarkerArray()
        n = len(self._waypoints)
        if n == 0:
            return array

        stamp = self.get_clock().now().to_msg()

        # --- Sphere + text per waypoint ---
        for i, (x, y) in enumerate(self._waypoints):
            idx = i + 1  # 1-based

            # Color: blue for first, red for last, green for middle
            if i == 0:
                r, g, b = 0.0, 0.4, 1.0
            elif i == n - 1:
                r, g, b = 1.0, 0.2, 0.2
            else:
                r, g, b = 0.1, 0.85, 0.1

            # Sphere marker
            sphere = Marker()
            sphere.header.frame_id = self._frame_id
            sphere.header.stamp = stamp
            sphere.ns = 'wp_sphere'
            sphere.id = idx
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = 0.0
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.20
            sphere.scale.y = 0.20
            sphere.scale.z = 0.20
            sphere.color.r = r
            sphere.color.g = g
            sphere.color.b = b
            sphere.color.a = 1.0
            sphere.lifetime.sec = 0
            array.markers.append(sphere)

            # Text label
            text = Marker()
            text.header.frame_id = self._frame_id
            text.header.stamp = stamp
            text.ns = 'wp_text'
            text.id = idx
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = 0.35
            text.pose.orientation.w = 1.0
            text.scale.z = 0.22
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = f'WP{idx:02d}'
            text.lifetime.sec = 0
            array.markers.append(text)

        return array


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointMarkerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
