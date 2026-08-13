#!/usr/bin/env python3
"""Conservative frontier exploration node for SLAM mapping.

The node does not command velocity directly. It selects safe frontier goals from
the SLAM map and sends them to Nav2's NavigateToPose action server.
"""

from collections import deque
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener


class FrontierExplorer(Node):
    """Finds map frontiers and asks Nav2 to visit them one at a time."""

    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('global_costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('action_name', '/navigate_to_pose')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('explore_period', 4.0)
        self.declare_parameter('min_frontier_size', 8)
        self.declare_parameter('obstacle_clearance', 0.75)
        self.declare_parameter('unknown_clearance', 0.25)
        self.declare_parameter('frontier_standoff', 0.8)
        self.declare_parameter('costmap_clearance', 0.75)
        self.declare_parameter('min_goal_distance', 0.8)
        self.declare_parameter('max_goal_distance', 3.0)
        self.declare_parameter('goal_timeout', 75.0)
        self.declare_parameter('blacklist_radius', 0.6)
        self.declare_parameter('max_consecutive_failures', 8)
        self.declare_parameter('costmap_occupied_threshold', 40)
        self.declare_parameter('dry_run', False)

        self.map_topic = self.get_parameter('map_topic').value
        self.global_costmap_topic = self.get_parameter('global_costmap_topic').value
        self.action_name = self.get_parameter('action_name').value
        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.explore_period = float(self.get_parameter('explore_period').value)
        self.min_frontier_size = int(self.get_parameter('min_frontier_size').value)
        self.obstacle_clearance = float(self.get_parameter('obstacle_clearance').value)
        self.unknown_clearance = float(self.get_parameter('unknown_clearance').value)
        self.frontier_standoff = float(self.get_parameter('frontier_standoff').value)
        self.costmap_clearance = float(self.get_parameter('costmap_clearance').value)
        self.min_goal_distance = float(self.get_parameter('min_goal_distance').value)
        self.max_goal_distance = float(self.get_parameter('max_goal_distance').value)
        self.goal_timeout = float(self.get_parameter('goal_timeout').value)
        self.blacklist_radius = float(self.get_parameter('blacklist_radius').value)
        self.max_consecutive_failures = int(self.get_parameter('max_consecutive_failures').value)
        self.costmap_occupied_threshold = int(self.get_parameter('costmap_occupied_threshold').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)

        self.map_msg = None
        self.map_data = None
        self.costmap_msg = None
        self.costmap_data = None
        self.goal_handle = None
        self.goal_sent_time = None
        self.active_goal = None
        self.blacklist = []
        self.consecutive_failures = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        costmap_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, map_qos)
        self.create_subscription(OccupancyGrid, self.global_costmap_topic, self._costmap_cb, costmap_qos)
        self.goal_pub = self.create_publisher(PoseStamped, '/explore_goal', 10)
        self.nav_client = ActionClient(self, NavigateToPose, self.action_name)
        self.timer = self.create_timer(self.explore_period, self._timer_cb)

        self.get_logger().info('Conservative frontier explorer started')
        self.get_logger().info(f'map={self.map_topic}, costmap={self.global_costmap_topic}, action={self.action_name}')
        self.get_logger().info(
            f'dry_run={self.dry_run}, obstacle_clearance={self.obstacle_clearance:.2f}m, '
            f'costmap_clearance={self.costmap_clearance:.2f}m, standoff={self.frontier_standoff:.2f}m'
        )

    def _map_cb(self, msg):
        self.map_msg = msg
        self.map_data = list(msg.data)

    def _costmap_cb(self, msg):
        self.costmap_msg = msg
        self.costmap_data = list(msg.data)

    def _timer_cb(self):
        if self.map_msg is None or self.map_data is None:
            self.get_logger().warn('Waiting for /map...', throttle_duration_sec=10.0)
            return

        if self.goal_handle is not None:
            if self.goal_sent_time and time.monotonic() - self.goal_sent_time > self.goal_timeout:
                self.get_logger().warn('Goal timed out, canceling and blacklisting it')
                self._blacklist_active_goal()
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None
                self.goal_sent_time = None
            return

        if self.consecutive_failures >= self.max_consecutive_failures:
            self.get_logger().error('Too many consecutive exploration failures; stopping for safety')
            self.timer.cancel()
            return

        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            self.get_logger().warn('Waiting for robot pose map->base_link...', throttle_duration_sec=5.0)
            return

        goal = self._select_frontier_goal(robot_pose)
        if goal is None:
            self.get_logger().info('No safe frontier goal found. Exploration may be complete or blocked.')
            return

        self._send_goal(goal, robot_pose)

    def _get_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
            return (transform.transform.translation.x, transform.transform.translation.y)
        except Exception as exc:
            self.get_logger().debug(f'TF lookup failed: {exc}')
            return None

    def _select_frontier_goal(self, robot_pose):
        width = self.map_msg.info.width
        height = self.map_msg.info.height
        resolution = self.map_msg.info.resolution
        obstacle_cells = max(1, int(math.ceil(self.obstacle_clearance / resolution)))
        unknown_cells = max(0, int(math.ceil(self.unknown_clearance / resolution)))

        candidates = []
        visited = set()

        for row in range(1, height - 1):
            for col in range(1, width - 1):
                idx = row * width + col
                if idx in visited or not self._is_frontier_cell(row, col):
                    continue

                cluster = self._grow_frontier(row, col, visited)
                if len(cluster) < self.min_frontier_size:
                    continue

                goal_world = self._cluster_goal(cluster, robot_pose, obstacle_cells, unknown_cells)
                if goal_world is None:
                    continue

                if not self._goal_is_safe(goal_world, robot_pose):
                    continue

                distance = math.hypot(goal_world[0] - robot_pose[0], goal_world[1] - robot_pose[1])
                score = distance - 0.03 * len(cluster)
                candidates.append((score, distance, len(cluster), goal_world))

        if not candidates:
            return None

        candidates.sort(key=lambda item: item[0])
        score, distance, size, goal = candidates[0]
        self.get_logger().info(
            f'Selected frontier goal ({goal[0]:.2f}, {goal[1]:.2f}), '
            f'distance={distance:.2f}m, size={size}, score={score:.2f}'
        )
        return goal

    def _is_frontier_cell(self, row, col):
        width = self.map_msg.info.width
        idx = row * width + col
        if self.map_data[idx] != 0:
            return False
        for nr in range(row - 1, row + 2):
            for nc in range(col - 1, col + 2):
                if nr == row and nc == col:
                    continue
                if self.map_data[nr * width + nc] == -1:
                    return True
        return False

    def _grow_frontier(self, start_row, start_col, visited):
        width = self.map_msg.info.width
        height = self.map_msg.info.height
        cluster = []
        queue = deque([(start_row, start_col)])
        visited.add(start_row * width + start_col)

        while queue:
            row, col = queue.popleft()
            cluster.append((row, col))
            for nr, nc in ((row - 1, col), (row + 1, col), (row, col - 1), (row, col + 1)):
                if nr <= 0 or nr >= height - 1 or nc <= 0 or nc >= width - 1:
                    continue
                idx = nr * width + nc
                if idx in visited:
                    continue
                if not self._is_frontier_cell(nr, nc):
                    continue
                visited.add(idx)
                queue.append((nr, nc))
        return cluster

    def _cluster_goal(self, cluster, robot_pose, obstacle_cells, unknown_cells):
        centroid_row = sum(cell[0] for cell in cluster) / len(cluster)
        centroid_col = sum(cell[1] for cell in cluster) / len(cluster)

        frontier_x, frontier_y = self._grid_to_world(centroid_row, centroid_col)
        to_robot_x = robot_pose[0] - frontier_x
        to_robot_y = robot_pose[1] - frontier_y
        norm = math.hypot(to_robot_x, to_robot_y)
        if norm < 1.0e-6:
            return None

        unit_x = to_robot_x / norm
        unit_y = to_robot_y / norm
        step = max(self.map_msg.info.resolution, 0.05)
        max_backoff = self.frontier_standoff + 1.0
        backoff = self.frontier_standoff

        while backoff <= max_backoff:
            goal_x = frontier_x + unit_x * backoff
            goal_y = frontier_y + unit_y * backoff
            grid = self._world_to_grid(goal_x, goal_y)
            if grid is None:
                backoff += step
                continue
            row, col = grid
            if self.map_data[row * self.map_msg.info.width + col] != 0:
                backoff += step
                continue
            if self._cell_has_near_value(row, col, 100, obstacle_cells, occupied=True):
                backoff += step
                continue
            if unknown_cells and self._cell_has_near_value(row, col, -1, unknown_cells, occupied=False):
                backoff += step
                continue
            return (goal_x, goal_y)
        return None

    def _world_to_grid(self, x, y):
        info = self.map_msg.info
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
        if row < 0 or row >= info.height or col < 0 or col >= info.width:
            return None
        return (row, col)

    def _cell_has_near_value(self, row, col, value, radius, occupied):
        width = self.map_msg.info.width
        height = self.map_msg.info.height
        for nr in range(max(0, row - radius), min(height, row + radius + 1)):
            for nc in range(max(0, col - radius), min(width, col + radius + 1)):
                if (nr - row) ** 2 + (nc - col) ** 2 > radius ** 2:
                    continue
                cell = self.map_data[nr * width + nc]
                if occupied and cell > 50:
                    return True
                if not occupied and cell == value:
                    return True
        return False

    def _goal_is_safe(self, goal, robot_pose):
        distance = math.hypot(goal[0] - robot_pose[0], goal[1] - robot_pose[1])
        if distance < self.min_goal_distance or distance > self.max_goal_distance:
            return False
        for bad_goal in self.blacklist:
            if math.hypot(goal[0] - bad_goal[0], goal[1] - bad_goal[1]) < self.blacklist_radius:
                return False
        if self.costmap_msg is not None and self.costmap_data is not None:
            if not self._costmap_area_is_safe(goal[0], goal[1]):
                return False
        return True

    def _costmap_area_is_safe(self, x, y):
        info = self.costmap_msg.info
        center_col = int((x - info.origin.position.x) / info.resolution)
        center_row = int((y - info.origin.position.y) / info.resolution)
        if center_row < 0 or center_row >= info.height or center_col < 0 or center_col >= info.width:
            return False

        radius = max(1, int(math.ceil(self.costmap_clearance / info.resolution)))
        for row in range(max(0, center_row - radius), min(info.height, center_row + radius + 1)):
            for col in range(max(0, center_col - radius), min(info.width, center_col + radius + 1)):
                if (row - center_row) ** 2 + (col - center_col) ** 2 > radius ** 2:
                    continue
                cost = self.costmap_data[row * info.width + col]
                if cost < 0 or cost >= self.costmap_occupied_threshold:
                    return False
        return True

    def _costmap_value_at(self, x, y):
        info = self.costmap_msg.info
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
        if row < 0 or row >= info.height or col < 0 or col >= info.width:
            return None
        return self.costmap_data[row * info.width + col]

    def _grid_to_world(self, row, col):
        info = self.map_msg.info
        return (
            info.origin.position.x + (col + 0.5) * info.resolution,
            info.origin.position.y + (row + 0.5) * info.resolution,
        )

    def _send_goal(self, goal, robot_pose):
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = goal[0]
        pose.pose.position.y = goal[1]
        yaw = math.atan2(goal[1] - robot_pose[1], goal[0] - robot_pose[0])
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(pose)

        if self.dry_run:
            self.get_logger().info('dry_run=true; not sending NavigateToPose goal')
            self.blacklist.append(goal)
            return

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('NavigateToPose action server is not available')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.active_goal = goal
        self.goal_sent_time = time.monotonic()
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_cb)
        self.get_logger().info(f'Sent exploration goal ({goal[0]:.2f}, {goal[1]:.2f})')

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Exploration goal rejected by Nav2')
            self._blacklist_active_goal()
            self.consecutive_failures += 1
            self.goal_handle = None
            self.goal_sent_time = None
            return
        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        result = future.result().result
        status = future.result().status
        if result.error_code == 0:
            self.get_logger().info('Exploration goal reached')
            self.consecutive_failures = 0
        else:
            self.get_logger().warn(
                f'Exploration goal failed: status={status}, error_code={result.error_code}, msg={result.error_msg}'
            )
            self._blacklist_active_goal()
            self.consecutive_failures += 1
        self.goal_handle = None
        self.goal_sent_time = None
        self.active_goal = None

    def _blacklist_active_goal(self):
        if self.active_goal is not None:
            self.blacklist.append(self.active_goal)
            self.get_logger().warn(f'Blacklisted goal ({self.active_goal[0]:.2f}, {self.active_goal[1]:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
