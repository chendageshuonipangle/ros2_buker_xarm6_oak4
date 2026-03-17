#!/usr/bin/env python3
"""
Bunker Mini 自定义路径规划器
使用自己的 A*/JPS 算法规划路径，然后调用 Nav2 的 FollowPath 执行

用法:
    # 终端1: 启动机器人和导航
    ./start_bunker_navigation.sh bringup
    ./start_bunker_navigation.sh slam_nav  # 或 nav

    # 终端2: 运行此脚本
    python3 bunker_path_planner.py --algorithm jps --goal 2.0 1.0
"""

import sys
import os
import time
import argparse
import numpy as np

# 添加算法路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowPath
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

# 导入自定义算法
from algorithms.astar import a_star
from algorithms.jps import jps
from algorithms.jps_improved import jps_improved


class BunkerPathPlanner(Node):
    """使用自定义算法的路径规划器"""

    def __init__(self, algorithm='jps'):
        super().__init__('bunker_path_planner')
        
        self.algorithm = algorithm
        self.map_data = None
        self.map_info = None
        self.robot_pose = None
        
        # TF2 监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 订阅地图
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )
        
        # 路径发布器（用于可视化）
        self.path_pub = self.create_publisher(Path, '/custom_path', 10)
        
        # Nav2 FollowPath Action Client
        self.follow_path_client = ActionClient(self, FollowPath, 'follow_path')
        
        self.get_logger().info(f'路径规划器已启动，使用算法: {algorithm}')

    def map_callback(self, msg):
        """接收地图数据"""
        self.map_info = msg.info
        # 转换为 numpy 数组 (0=free, 1=obstacle, -1=unknown)
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))
        
        # 转换: OccupancyGrid 中 0=free, 100=occupied, -1=unknown
        # 我们的算法: 0=free, 1=obstacle
        self.map_data = np.zeros_like(data, dtype=np.int8)
        self.map_data[data > 50] = 1  # 占用 -> 障碍
        # 未知区域 (-1) 保持为 0，允许通过（SLAM 模式下很多区域是未知的）
        
        # 统计
        free_count = np.sum(self.map_data == 0)
        obstacle_count = np.sum(self.map_data == 1)
        unknown_count = np.sum(data < 0)
        
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        
        self.get_logger().info(f'收到地图: {width}x{height}, 分辨率: {msg.info.resolution}m')
        self.get_logger().info(f'地图原点: ({origin_x:.2f}, {origin_y:.2f})')
        self.get_logger().info(f'地图范围: X[{origin_x:.2f}, {origin_x + width*msg.info.resolution:.2f}], Y[{origin_y:.2f}, {origin_y + height*msg.info.resolution:.2f}]')
        self.get_logger().info(f'栅格统计: 可通行={free_count}, 障碍={obstacle_count}, 未知={unknown_count}')

    def get_robot_pose(self):
        """获取机器人当前位姿"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            return (x, y)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'无法获取机器人位姿: {e}')
            return None

    def world_to_grid(self, world_x, world_y):
        """世界坐标转栅格坐标"""
        if self.map_info is None:
            return None
        
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        resolution = self.map_info.resolution
        
        grid_x = int((world_x - origin_x) / resolution)
        grid_y = int((world_y - origin_y) / resolution)
        
        return (grid_y, grid_x)  # 注意: numpy 是 (row, col) = (y, x)

    def grid_to_world(self, grid_row, grid_col):
        """栅格坐标转世界坐标"""
        if self.map_info is None:
            return None
        
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        resolution = self.map_info.resolution
        
        world_x = grid_col * resolution + origin_x + resolution / 2
        world_y = grid_row * resolution + origin_y + resolution / 2
        
        return (world_x, world_y)

    def plan_path(self, goal_x, goal_y):
        """使用自定义算法规划路径"""
        if self.map_data is None:
            self.get_logger().error('还没有收到地图数据!')
            return None
        
        # 获取机器人当前位置
        robot_pos = self.get_robot_pose()
        if robot_pos is None:
            self.get_logger().error('无法获取机器人位置!')
            return None
        
        start_x, start_y = robot_pos
        self.get_logger().info(f'起点: ({start_x:.2f}, {start_y:.2f})')
        self.get_logger().info(f'终点: ({goal_x:.2f}, {goal_y:.2f})')
        
        # 转换为栅格坐标
        start_grid = self.world_to_grid(start_x, start_y)
        goal_grid = self.world_to_grid(goal_x, goal_y)
        
        if start_grid is None or goal_grid is None:
            self.get_logger().error('坐标转换失败!')
            return None
        
        self.get_logger().info(f'栅格起点: {start_grid}, 栅格终点: {goal_grid}')
        
        # 检查起点和终点是否有效
        if not self._is_valid_point(start_grid):
            self.get_logger().error(f'起点无效或在障碍物中!')
            return None
        if not self._is_valid_point(goal_grid):
            self.get_logger().error(f'终点无效或在障碍物中!')
            return None
        
        # 调用路径规划算法
        self.get_logger().info(f'开始规划路径，算法: {self.algorithm}')
        start_time = time.time()
        
        if self.algorithm == 'astar':
            path, cost, expanded = a_star(self.map_data, start_grid, goal_grid)
            jump_count = 0
        elif self.algorithm == 'jps':
            path, cost, expanded, jump_count = jps(self.map_data, start_grid, goal_grid)
        elif self.algorithm == 'jps_improved':
            path, cost, expanded, jump_count = jps_improved(self.map_data, start_grid, goal_grid)
        else:
            self.get_logger().error(f'未知算法: {self.algorithm}')
            return None
        
        elapsed = time.time() - start_time
        
        if path is None:
            self.get_logger().error('路径规划失败，无法找到路径!')
            return None
        
        self.get_logger().info(f'路径规划成功!')
        self.get_logger().info(f'  - 路径点数: {len(path)}')
        self.get_logger().info(f'  - 路径代价: {cost:.2f}')
        self.get_logger().info(f'  - 扩展节点: {expanded}')
        self.get_logger().info(f'  - 跳点数: {jump_count}')
        self.get_logger().info(f'  - 耗时: {elapsed*1000:.2f}ms')
        
        # 转换为世界坐标路径
        world_path = []
        for grid_point in path:
            world_point = self.grid_to_world(grid_point[0], grid_point[1])
            if world_point:
                world_path.append(world_point)
        
        return world_path

    def _is_valid_point(self, grid_point):
        """检查栅格点是否有效"""
        row, col = grid_point
        if row < 0 or row >= self.map_data.shape[0]:
            self.get_logger().warn(f'点 ({row}, {col}) 超出地图范围 (0-{self.map_data.shape[0]-1}, 0-{self.map_data.shape[1]-1})')
            return False
        if col < 0 or col >= self.map_data.shape[1]:
            self.get_logger().warn(f'点 ({row}, {col}) 超出地图范围 (0-{self.map_data.shape[0]-1}, 0-{self.map_data.shape[1]-1})')
            return False
        if self.map_data[row, col] == 1:
            self.get_logger().warn(f'点 ({row}, {col}) 在障碍物中')
            return False
        return True

    def create_path_msg(self, world_path):
        """创建 nav_msgs/Path 消息"""
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for i, (x, y) in enumerate(world_path):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            
            # 计算朝向（指向下一个点）
            if i < len(world_path) - 1:
                next_x, next_y = world_path[i + 1]
                yaw = np.arctan2(next_y - y, next_x - x)
            else:
                yaw = 0.0
            
            # 四元数
            pose.pose.orientation.z = np.sin(yaw / 2)
            pose.pose.orientation.w = np.cos(yaw / 2)
            
            path_msg.poses.append(pose)
        
        return path_msg

    def execute_path(self, world_path):
        """调用 Nav2 FollowPath 执行路径"""
        if not self.follow_path_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('FollowPath action server 不可用!')
            return False
        
        # 创建路径消息
        path_msg = self.create_path_msg(world_path)
        
        # 发布路径用于可视化
        self.path_pub.publish(path_msg)
        self.get_logger().info('已发布路径到 /custom_path')
        
        # 创建 FollowPath goal
        goal = FollowPath.Goal()
        goal.path = path_msg
        goal.controller_id = ''  # 使用默认控制器
        
        self.get_logger().info('发送路径到 Nav2 FollowPath...')
        
        # 发送目标
        send_goal_future = self.follow_path_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('路径被拒绝!')
            return False
        
        self.get_logger().info('路径已接受，开始执行...')
        
        # 等待结果
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('路径执行完成!')
            return True
        else:
            self.get_logger().warn(f'路径执行结束，状态: {result.status}')
            return False

    def feedback_callback(self, feedback_msg):
        """路径执行反馈"""
        feedback = feedback_msg.feedback
        distance = feedback.distance_to_goal
        self.get_logger().info(f'距离目标: {distance:.2f}m', throttle_duration_sec=2.0)

    def navigate_to(self, goal_x, goal_y):
        """完整的导航流程"""
        # 等待地图
        self.get_logger().info('等待地图数据...')
        while self.map_data is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)
        
        # 等待 TF 变换准备好（需要给 TF buffer 时间填充）
        self.get_logger().info('等待 TF 变换...')
        import time
        time.sleep(2.0)  # 先等待 2 秒让 TF buffer 填充
        
        max_wait = 15  # 最多等待 15 秒
        waited = 0
        while waited < max_wait and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=1.0)  # 每次等待 1 秒
            robot_pos = self.get_robot_pose()
            if robot_pos is not None:
                break
            waited += 1
            self.get_logger().info(f'等待 TF... ({waited}s)')
        
        if self.get_robot_pose() is None:
            self.get_logger().error('TF 变换超时，请确保 SLAM 或 AMCL 正在运行!')
            return False
        
        # 规划路径
        world_path = self.plan_path(goal_x, goal_y)
        if world_path is None:
            return False
        
        # 执行路径
        return self.execute_path(world_path)


def main():
    parser = argparse.ArgumentParser(description='Bunker Mini 自定义路径规划器')
    parser.add_argument('--algorithm', '-a', type=str, default='jps_improved',
                        choices=['astar', 'jps', 'jps_improved'],
                        help='路径规划算法 (default: jps_improved)')
    parser.add_argument('--goal', '-g', type=float, nargs=2, required=True,
                        metavar=('X', 'Y'),
                        help='目标点坐标 (米)')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    planner = BunkerPathPlanner(algorithm=args.algorithm)
    
    try:
        success = planner.navigate_to(args.goal[0], args.goal[1])
        if success:
            print('\n✓ 导航成功!')
        else:
            print('\n✗ 导航失败!')
    except KeyboardInterrupt:
        print('\n用户中断')
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
