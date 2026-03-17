#!/usr/bin/env python3
"""
JPS 路径规划代理节点
监听 RViz 的 Nav2 Goal，使用自定义 JPS 算法规划路径，然后调用 Nav2 FollowPath 执行

启动方式:
    python3 jps_planner_node.py

然后在 RViz 中使用 "2D Goal Pose" 或 "Nav2 Goal" 按钮设置目标点
"""

import sys
import os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from tf2_ros import Buffer, TransformListener

from algorithms.jps_improved import jps_improved
from algorithms.jps import jps
from algorithms.astar import a_star
from algorithms.astar_costmap import astar_costmap


class JPSPlannerNode(Node):
    """JPS 路径规划代理节点"""

    def __init__(self):
        super().__init__('jps_planner_node')
        
        # 声明参数
        self.declare_parameter('algorithm', 'jps_improved')
        self.declare_parameter('replan_interval', 2.0)  # 重规划间隔（秒）
        self.algorithm = self.get_parameter('algorithm').value
        self.replan_interval = self.get_parameter('replan_interval').value
        
        self.map_data = None
        self.map_info = None
        self.is_navigating = False
        self.current_goal = None
        self.current_goal_orientation = None
        self.last_replan_time = None
        self.goal_handle = None
        
        # 回调组（允许并发）
        self.callback_group = ReentrantCallbackGroup()
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 订阅 global_costmap（包含膨胀的代价地图）
        costmap_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.map_callback, costmap_qos)
        
        # 订阅 RViz 的目标点 (2D Goal Pose)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10,
            callback_group=self.callback_group)
        
        # 路径发布器
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.custom_path_pub = self.create_publisher(Path, '/custom_path', 10)
        
        # Nav2 FollowPath Action Client
        self.follow_path_client = ActionClient(
            self, FollowPath, 'follow_path',
            callback_group=self.callback_group)
        
        self.get_logger().info(f'========================================')
        self.get_logger().info(f'  JPS 路径规划节点已启动')
        self.get_logger().info(f'  算法: {self.algorithm}')
        self.get_logger().info(f'  在 RViz 中使用 "2D Goal Pose" 设置目标')
        self.get_logger().info(f'========================================')

    def map_callback(self, msg):
        """接收代价地图"""
        self.map_info = msg.info
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))
        
        # 保存原始代价地图（用于 astar_costmap）
        self.costmap_data = data.astype(np.int16)
        
        # 二值化地图（用于 JPS）
        self.map_data = np.zeros_like(data, dtype=np.int8)
        self.map_data[data >= 253] = 1
        self.map_data[data == 255] = 1
        
        self.get_logger().info(
            f'代价地图更新: {width}x{height}, 分辨率: {msg.info.resolution:.3f}m',
            throttle_duration_sec=10.0)
        
        # 如果正在导航，检查是否需要重规划
        if self.is_navigating and self.current_goal is not None:
            self.check_and_replan()

    def goal_callback(self, msg):
        """收到目标点"""
        if self.is_navigating:
            self.get_logger().warn('正在导航中，忽略新目标')
            return
        
        if self.map_data is None:
            self.get_logger().error('还没有收到地图!')
            return
        
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        
        self.get_logger().info(f'收到目标点: ({goal_x:.2f}, {goal_y:.2f})')
        
        # 异步执行导航
        self.is_navigating = True
        try:
            self.navigate_to(goal_x, goal_y, msg.pose.orientation)
        finally:
            self.is_navigating = False

    def get_robot_pose(self):
        """获取机器人位姿"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            return (x, y)
        except Exception as e:
            self.get_logger().warn(f'无法获取位姿: {e}')
            return None

    def world_to_grid(self, world_x, world_y):
        """世界坐标转栅格坐标"""
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        resolution = self.map_info.resolution
        grid_x = int((world_x - origin_x) / resolution)
        grid_y = int((world_y - origin_y) / resolution)
        return (grid_y, grid_x)

    def grid_to_world(self, grid_row, grid_col):
        """栅格坐标转世界坐标"""
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        resolution = self.map_info.resolution
        world_x = grid_col * resolution + origin_x + resolution / 2
        world_y = grid_row * resolution + origin_y + resolution / 2
        return (world_x, world_y)

    def is_valid_point(self, grid_point):
        """检查点是否有效"""
        row, col = grid_point
        if row < 0 or row >= self.map_data.shape[0]:
            return False
        if col < 0 or col >= self.map_data.shape[1]:
            return False
        return self.map_data[row, col] == 0

    def plan_path(self, start_x, start_y, goal_x, goal_y):
        """规划路径"""
        start_grid = self.world_to_grid(start_x, start_y)
        goal_grid = self.world_to_grid(goal_x, goal_y)
        
        self.get_logger().info(f'起点: ({start_x:.2f}, {start_y:.2f}) -> 栅格 {start_grid}')
        self.get_logger().info(f'终点: ({goal_x:.2f}, {goal_y:.2f}) -> 栅格 {goal_grid}')
        
        if not self.is_valid_point(start_grid):
            self.get_logger().error('起点无效!')
            return None
        if not self.is_valid_point(goal_grid):
            self.get_logger().error('终点无效!')
            return None
        
        # 调用算法
        import time
        start_time = time.time()
        
        if self.algorithm == 'astar':
            path, cost, expanded = a_star(self.map_data, start_grid, goal_grid)
            jump_count = 0
        elif self.algorithm == 'jps':
            path, cost, expanded, jump_count = jps(self.map_data, start_grid, goal_grid)
        elif self.algorithm == 'jps_improved':
            path, cost, expanded, jump_count = jps_improved(self.map_data, start_grid, goal_grid)
        elif self.algorithm == 'astar_costmap':
            # 使用考虑代价的 A*，路径会远离高代价区域
            path, cost, expanded = astar_costmap(
                self.costmap_data, start_grid, goal_grid,
                obstacle_threshold=253,
                cost_weight=0.05  # 代价权重，越大越远离障碍物
            )
            jump_count = 0
        else:
            self.get_logger().error(f'未知算法: {self.algorithm}')
            return None
        
        elapsed = (time.time() - start_time) * 1000
        
        if path is None:
            self.get_logger().error('路径规划失败!')
            return None
        
        self.get_logger().info(f'路径规划成功! 算法: {self.algorithm}')
        self.get_logger().info(f'  路径点: {len(path)}, 代价: {cost:.2f}, 扩展: {expanded}, 耗时: {elapsed:.2f}ms')
        
        # 转换为世界坐标
        world_path = []
        for grid_point in path:
            world_point = self.grid_to_world(grid_point[0], grid_point[1])
            world_path.append(world_point)
        
        return world_path

    def create_path_msg(self, world_path, goal_orientation):
        """创建 Path 消息，增加路径点密度"""
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        # 插值增加路径点密度
        dense_path = self.interpolate_path(world_path, max_dist=0.1)  # 每 10cm 一个点
        
        for i, (x, y) in enumerate(dense_path):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            
            # 计算朝向（指向下一个点）
            if i < len(dense_path) - 1:
                next_x, next_y = dense_path[i + 1]
                yaw = np.arctan2(next_y - y, next_x - x)
            elif i > 0:
                # 最后一个点使用前一段的方向
                prev_x, prev_y = dense_path[i - 1]
                yaw = np.arctan2(y - prev_y, x - prev_x)
            else:
                yaw = 0.0
            
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = np.sin(yaw / 2)
            pose.pose.orientation.w = np.cos(yaw / 2)
            
            path_msg.poses.append(pose)
        
        # 最后一个点使用目标朝向
        if len(path_msg.poses) > 0:
            path_msg.poses[-1].pose.orientation = goal_orientation
        
        return path_msg

    def interpolate_path(self, path, max_dist=0.1):
        """插值路径，增加点密度"""
        if len(path) < 2:
            return path
        
        dense_path = [path[0]]
        
        for i in range(1, len(path)):
            x1, y1 = path[i - 1]
            x2, y2 = path[i]
            
            dist = np.hypot(x2 - x1, y2 - y1)
            
            if dist > max_dist:
                # 需要插值
                n_points = int(np.ceil(dist / max_dist))
                for j in range(1, n_points + 1):
                    t = j / n_points
                    x = x1 + t * (x2 - x1)
                    y = y1 + t * (y2 - y1)
                    dense_path.append((x, y))
            else:
                dense_path.append((x2, y2))
        
        return dense_path

    def navigate_to(self, goal_x, goal_y, goal_orientation):
        """导航到目标点"""
        import time
        
        # 保存当前目标
        self.current_goal = (goal_x, goal_y)
        self.current_goal_orientation = goal_orientation
        self.last_replan_time = time.time()
        
        # 获取当前位置
        robot_pos = self.get_robot_pose()
        if robot_pos is None:
            self.get_logger().error('无法获取机器人位置!')
            return
        
        start_x, start_y = robot_pos
        
        # 规划路径
        world_path = self.plan_path(start_x, start_y, goal_x, goal_y)
        if world_path is None:
            self.is_navigating = False
            return
        
        # 创建路径消息
        path_msg = self.create_path_msg(world_path, goal_orientation)
        
        # 发布路径用于可视化
        self.path_pub.publish(path_msg)
        self.custom_path_pub.publish(path_msg)
        
        # 等待 FollowPath 服务
        if not self.follow_path_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('FollowPath 服务不可用!')
            self.is_navigating = False
            return
        
        # 发送路径
        goal = FollowPath.Goal()
        goal.path = path_msg
        
        self.get_logger().info('发送路径到 Nav2 FollowPath...')
        
        future = self.follow_path_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        
        rclpy.spin_until_future_complete(self, future)
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().error('路径被拒绝!')
            self.is_navigating = False
            return
        
        self.get_logger().info('路径已接受，执行中...')
        
        result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        if result.status == 4:
            self.get_logger().info('✓ 导航完成!')
        else:
            self.get_logger().warn(f'导航结束，状态: {result.status}')
        
        self.is_navigating = False
        self.current_goal = None

    def feedback_callback(self, feedback_msg):
        """反馈回调"""
        distance = feedback_msg.feedback.distance_to_goal
        self.get_logger().info(f'距离目标: {distance:.2f}m', throttle_duration_sec=2.0)

    def check_and_replan(self):
        """检查是否需要重规划"""
        import time
        current_time = time.time()
        
        # 检查重规划间隔
        if self.last_replan_time is not None:
            if current_time - self.last_replan_time < self.replan_interval:
                return
        
        # 获取当前位置
        robot_pos = self.get_robot_pose()
        if robot_pos is None:
            return
        
        goal_x, goal_y = self.current_goal
        start_x, start_y = robot_pos
        
        # 检查当前路径是否还有效（简单检查：能否规划出路径）
        start_grid = self.world_to_grid(start_x, start_y)
        goal_grid = self.world_to_grid(goal_x, goal_y)
        
        if not self.is_valid_point(start_grid) or not self.is_valid_point(goal_grid):
            return
        
        self.get_logger().info('地图更新，重新规划路径...')
        self.last_replan_time = current_time
        
        # 取消当前路径执行
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        
        # 重新规划并执行
        world_path = self.plan_path(start_x, start_y, goal_x, goal_y)
        if world_path is not None:
            self.execute_path_async(world_path, self.current_goal_orientation)

    def execute_path_async(self, world_path, goal_orientation):
        """异步执行路径"""
        # 创建路径消息
        path_msg = self.create_path_msg(world_path, goal_orientation)
        
        # 发布路径用于可视化
        self.path_pub.publish(path_msg)
        self.custom_path_pub.publish(path_msg)
        
        # 等待 FollowPath 服务
        if not self.follow_path_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('FollowPath 服务不可用!')
            return
        
        # 发送路径
        goal = FollowPath.Goal()
        goal.path = path_msg
        
        self.get_logger().info('发送新路径到 Nav2 FollowPath...')
        
        future = self.follow_path_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """目标响应回调"""
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('路径被拒绝!')
            return
        
        self.get_logger().info('路径已接受，执行中...')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """结果回调"""
        result = future.result()
        if result.status == 4:
            self.get_logger().info('✓ 导航完成!')
            self.is_navigating = False
            self.current_goal = None
        else:
            self.get_logger().warn(f'导航结束，状态: {result.status}')


def main():
    rclpy.init()
    node = JPSPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
