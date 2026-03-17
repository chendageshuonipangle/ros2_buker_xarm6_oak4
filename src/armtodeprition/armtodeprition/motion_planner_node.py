#!/usr/bin/env python3
"""
xArm6 MoveIt2 运动规划节点

功能：
1. 订阅 /target_pose_in_base 话题（来自 oaktf_trantoarm 的转换结果）
2. 调用 MoveIt2 进行路径规划
3. 执行规划的轨迹
4. 提供夹爪控制服务

输入话题: /target_pose_in_base (geometry_msgs/PoseStamped)
服务: /execute_grasp (std_srvs/Trigger) - 执行抓取序列
服务: /gripper_open (std_srvs/Trigger)
服务: /gripper_close (std_srvs/Trigger)
"""

import threading
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Pose
from std_srvs.srv import Trigger
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    JointConstraint,
    BoundingVolume,
    PlanningScene,
)

from control_msgs.action import GripperCommand


class ArmMotionPlannerNode(Node):
    def __init__(self):
        super().__init__('arm_motion_planner_node')

        self.callback_group = ReentrantCallbackGroup()

        # ==========================================
        # 参数声明
        # ==========================================
        self.declare_parameter('planning_group', 'xarm6')
        self.declare_parameter('base_frame', 'link_base')
        self.declare_parameter('end_effector_link', 'link_eef')
        self.declare_parameter('approach_height', 0.05)  # 接近高度 (m)
        self.declare_parameter('grasp_height_offset', 0.02)  # 抓取高度偏移 (m)
        self.declare_parameter('auto_execute', True)  # 自动执行模式
        self.declare_parameter('auto_execute_interval', 3.0)  # 自动执行间隔 (秒)

        self.planning_group = self.get_parameter('planning_group').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.end_effector_link = self.get_parameter('end_effector_link').get_parameter_value().string_value
        self.approach_height = self.get_parameter('approach_height').get_parameter_value().double_value
        self.grasp_height_offset = self.get_parameter('grasp_height_offset').get_parameter_value().double_value
        self.auto_execute = self.get_parameter('auto_execute').get_parameter_value().bool_value
        self.auto_execute_interval = self.get_parameter('auto_execute_interval').get_parameter_value().double_value

        # 当前目标位姿
        self.current_target_pose = None
        self.target_lock = threading.Lock()
        self.is_executing = False  # 执行中标志
        self.is_cooling_down = False  # 冷却中标志
        self.cooldown_duration = 5.0  # 冷却时间(秒)

        # ==========================================
        # 订阅目标位姿
        # ==========================================
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/target_pose_in_base',
            self.target_pose_callback,
            10,
            callback_group=self.callback_group
        )

        # ==========================================
        # 自动执行定时器
        # ==========================================
        if self.auto_execute:
            self.auto_timer = self.create_timer(
                self.auto_execute_interval,
                self._auto_execute_timer_callback,
                callback_group=self.callback_group
            )
            self.get_logger().info(f'自动执行模式已启用，检测间隔: {self.auto_execute_interval}s')

        # ==========================================
        # MoveIt2 Action Clients
        # ==========================================
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action',
            callback_group=self.callback_group
        )

        # ==========================================
        # 夹爪 Action 客户端
        # ==========================================
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            '/xarm_gripper/gripper_action',
            callback_group=self.callback_group
        )

        # ==========================================
        # 规划场景发布器 (用于添加障碍物)
        # ==========================================
        self.planning_scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )

        # ==========================================
        # 服务: 执行抓取、夹爪控制
        # ==========================================
        self.grasp_srv = self.create_service(
            Trigger,
            '/execute_grasp',
            self.execute_grasp_callback,
            callback_group=self.callback_group
        )

        self.gripper_open_srv = self.create_service(
            Trigger,
            '/gripper_open',
            self.gripper_open_callback,
            callback_group=self.callback_group
        )

        self.gripper_close_srv = self.create_service(
            Trigger,
            '/gripper_close',
            self.gripper_close_callback,
            callback_group=self.callback_group
        )

        # ==========================================
        # 初始化障碍物
        # ==========================================
        self.add_workspace_obstacles()

        self.get_logger().info('Arm Motion Planner Node 已启动')
        self.get_logger().info(f'  Planning Group: {self.planning_group}')
        self.get_logger().info(f'  订阅: /target_pose_in_base')
        self.get_logger().info(f'  服务: /execute_grasp, /gripper_open, /gripper_close')

    def add_workspace_obstacles(self):
        """添加工作空间内的固定障碍物"""
        # 障碍物: 底座左侧 7.5cm, 高 16cm, 12.2cm x 10.2cm
        scene_msg = PlanningScene()
        scene_msg.is_diff = True

        # 障碍物 1: 右下角障碍物 (小方块)
        obstacle = CollisionObject()
        obstacle.header.frame_id = self.base_frame
        obstacle.id = 'left_bottom_obstacle'
        obstacle.operation = CollisionObject.ADD

        # 尺寸: 12.2cm x 10.2cm x 16cm
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.122, 0.102, 0.16]  # x, y, z in meters

        # 位置: 右下角 (左右对称)
        box_pose = Pose()
        box_pose.position.x = 0.1   # 前方 10cm
        box_pose.position.y = 0.2   # 右侧 20cm
        box_pose.position.z = 0.08  # 高度中心
        box_pose.orientation.w = 1.0

        obstacle.primitives.append(box)
        obstacle.primitive_poses.append(box_pose)

        scene_msg.world.collision_objects.append(obstacle)

        # 障碍物 2: 后方障碍物 (假设为墙壁)
        rear_obstacle = CollisionObject()
        rear_obstacle.header.frame_id = self.base_frame
        rear_obstacle.id = 'rear_wall'
        rear_obstacle.operation = CollisionObject.ADD

        wall = SolidPrimitive()
        wall.type = SolidPrimitive.BOX
        wall.dimensions = [0.02, 1.0, 0.5]  # 薄墙

        wall_pose = Pose()
        wall_pose.position.x = -0.15  # 后方 15cm
        wall_pose.position.y = 0.0
        wall_pose.position.z = 0.25
        wall_pose.orientation.w = 1.0

        rear_obstacle.primitives.append(wall)
        rear_obstacle.primitive_poses.append(wall_pose)

        scene_msg.world.collision_objects.append(rear_obstacle)

        # 障碍物 3: 左侧障碍物 (大方块)
        left_bottom = CollisionObject()
        left_bottom.header.frame_id = self.base_frame
        left_bottom.id = 'right_obstacle'
        left_bottom.operation = CollisionObject.ADD

        lb_box = SolidPrimitive()
        lb_box.type = SolidPrimitive.BOX
        lb_box.dimensions = [0.15, 0.15, 0.20]  # 15cm x 15cm x 20cm

        lb_pose = Pose()
        lb_pose.position.x = 0.0
        lb_pose.position.y = -0.15  # 左侧 (左右对称)
        lb_pose.position.z = 0.1   # 高度中心
        lb_pose.orientation.w = 1.0

        left_bottom.primitives.append(lb_box)
        left_bottom.primitive_poses.append(lb_pose)

        scene_msg.world.collision_objects.append(left_bottom)

        self.planning_scene_pub.publish(scene_msg)
        self.get_logger().info('障碍物已添加到规划场景 (右下角、后方、左侧)')

    def target_pose_callback(self, msg: PoseStamped):
        """接收目标位姿，更新当前目标"""
        # 执行中或冷却中不接受新目标
        if self.is_executing or self.is_cooling_down:
            return
        
        with self.target_lock:
            self.current_target_pose = msg
        
        self.get_logger().debug(
            f'收到目标: ({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})'
            )

    def _auto_execute_timer_callback(self):
        """定时检查是否应该自动执行抓取"""
        if not self.auto_execute:
            return
        
        # 调试日志
        self.get_logger().debug(f'定时器检查: is_executing={self.is_executing}, has_target={self.current_target_pose is not None}')
            
        if self.is_executing:
            return
            
        with self.target_lock:
            if self.current_target_pose is None:
                return
            target_pose = self.current_target_pose
            # 清除当前目标，等待新目标
            self.current_target_pose = None
        
        self.get_logger().info(
            f'检测到目标: ({target_pose.pose.position.x:.3f}, {target_pose.pose.position.y:.3f}, {target_pose.pose.position.z:.3f})'
        )
        self.get_logger().info('自动触发抓取序列...')
        self.is_executing = True
        # 在新线程中执行抓取，传递目标位姿
        threading.Thread(target=self._auto_execute_grasp, args=(target_pose.pose,), daemon=True).start()

    def gripper_control(self, position: float) -> bool:
        """控制夹爪
        
        Args:
            position: 夹爪位置 (0.0=全开, 0.85=全闭)
        """
        # 等待 Action Server
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('夹爪 Action Server 不可用，跳过夹爪控制')
            return True  # 跳过夹爪，继续执行

        # 构建 GripperCommand Goal
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 0.0

        self.get_logger().info(f'发送夹爪命令: position={position}')
        
        # 发送目标
        send_goal_future = self.gripper_client.send_goal_async(goal)
        
        # 轮询等待目标被接受
        timeout = 10.0
        start_time = self.get_clock().now().nanoseconds / 1e9
        while not send_goal_future.done():
            if (self.get_clock().now().nanoseconds / 1e9 - start_time) > timeout:
                self.get_logger().warn('夹爪目标发送超时')
                return False
            time.sleep(0.1)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('夹爪目标被拒绝')
            return False

        # 等待结果
        result_future = goal_handle.get_result_async()
        timeout = 15.0
        start_time = self.get_clock().now().nanoseconds / 1e9
        while not result_future.done():
            if (self.get_clock().now().nanoseconds / 1e9 - start_time) > timeout:
                self.get_logger().warn('夹爪执行超时')
                return False
            time.sleep(0.1)

        result = result_future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(f'夹爪控制完成: position={result.result.position:.3f}')
            return True
        else:
            self.get_logger().warn(f'夹爪控制失败: status={result.status}')
            return False

    def gripper_open_callback(self, request, response):
        """打开夹爪服务"""
        success = self.gripper_control(0.0)
        response.success = success
        response.message = '夹爪已打开' if success else '夹爪打开失败'
        return response

    def gripper_close_callback(self, request, response):
        """关闭夹爪服务"""
        success = self.gripper_control(0.85)
        response.success = success
        response.message = '夹爪已关闭' if success else '夹爪关闭失败'
        return response

    def plan_and_execute(self, target_pose: Pose) -> bool:
        """规划并执行到目标位姿
        
        使用 MoveIt2 MoveGroup Action
        """
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup Action Server 不可用')
            return False

        self.get_logger().info(f'目标位置: x={target_pose.position.x:.3f}, y={target_pose.position.y:.3f}, z={target_pose.position.z:.3f}')

        # 构建 MoveGroup Goal
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = self.planning_group
        goal.request.num_planning_attempts = 20
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.1  # 10% 速度
        goal.request.max_acceleration_scaling_factor = 0.05  # 5% 加速度
        
        # 设置工作空间
        goal.request.workspace_parameters.header.frame_id = self.base_frame
        goal.request.workspace_parameters.min_corner.x = -1.0
        goal.request.workspace_parameters.min_corner.y = -1.0
        goal.request.workspace_parameters.min_corner.z = -0.5
        goal.request.workspace_parameters.max_corner.x = 1.0
        goal.request.workspace_parameters.max_corner.y = 1.0
        goal.request.workspace_parameters.max_corner.z = 1.5

        # 目标约束 - 使用位置和姿态约束
        goal_constraints = Constraints()

        # 位置约束 (放大约束球体)
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.base_frame
        position_constraint.link_name = self.end_effector_link

        bounding_volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.05]  # 半径 5cm (放大容差)
        bounding_volume.primitives.append(sphere)

        sphere_pose = Pose()
        sphere_pose.position = target_pose.position
        sphere_pose.orientation.w = 1.0
        bounding_volume.primitive_poses.append(sphere_pose)

        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0
        goal_constraints.position_constraints.append(position_constraint)

        # 姿态约束 - 末端朝下 (绕x轴旋转180度)
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.base_frame
        orientation_constraint.link_name = self.end_effector_link
        # 末端朝下的姿态: 绕X轴旋转180度
        orientation_constraint.orientation.x = 1.0
        orientation_constraint.orientation.y = 0.0
        orientation_constraint.orientation.z = 0.0
        orientation_constraint.orientation.w = 0.0
        orientation_constraint.absolute_x_axis_tolerance = 0.5  # ~30度
        orientation_constraint.absolute_y_axis_tolerance = 0.5
        orientation_constraint.absolute_z_axis_tolerance = 3.14  # Z轴自由
        orientation_constraint.weight = 1.0
        goal_constraints.orientation_constraints.append(orientation_constraint)

        goal.request.goal_constraints.append(goal_constraints)
        
        # 路径约束 - 保持末端朝下姿态，减少旋转
        path_orientation = OrientationConstraint()
        path_orientation.header.frame_id = self.base_frame
        path_orientation.link_name = self.end_effector_link
        path_orientation.orientation.x = 1.0
        path_orientation.orientation.y = 0.0
        path_orientation.orientation.z = 0.0
        path_orientation.orientation.w = 0.0
        path_orientation.absolute_x_axis_tolerance = 0.3  # ~17度
        path_orientation.absolute_y_axis_tolerance = 0.3
        path_orientation.absolute_z_axis_tolerance = 0.5  # 限制Z轴旋转
        path_orientation.weight = 1.0
        goal.request.path_constraints.orientation_constraints.append(path_orientation)
        
        # 规划选项
        goal.planning_options.plan_only = False  # 规划并执行
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5

        # 发送 Goal (使用轮询方式等待，避免阻塞定时器)
        send_goal_future = self.move_group_client.send_goal_async(goal)
        
        # 轮询等待 goal 被接受
        timeout = 10.0
        start_time = self.get_clock().now().nanoseconds / 1e9
        while not send_goal_future.done():
            if (self.get_clock().now().nanoseconds / 1e9 - start_time) > timeout:
                self.get_logger().error('发送目标超时')
                return False
            time.sleep(0.1)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('MoveGroup Goal 被拒绝')
            return False

        # 等待结果 (轮询方式)
        result_future = goal_handle.get_result_async()
        timeout = 120.0
        start_time = self.get_clock().now().nanoseconds / 1e9
        while not result_future.done():
            if (self.get_clock().now().nanoseconds / 1e9 - start_time) > timeout:
                self.get_logger().error('运动执行超时')
                return False
            time.sleep(0.1)

        if result_future.result() is None:
            self.get_logger().error('运动执行超时，未收到结果')
            return False

        result = result_future.result()
        self.get_logger().info(f'Result status: {result.status}')
        self.get_logger().info(f'Result type: {type(result.result)}')
        
        # 检查 status: 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('运动执行成功 (status=SUCCEEDED)')
            return True
        elif result.status == 6:  # ABORTED
            error_code = result.result.error_code.val if hasattr(result.result, 'error_code') else -999
            self.get_logger().error(f'运动被中止: error_code={error_code}')
            return False
        
        error_code = result.result.error_code.val if hasattr(result.result, 'error_code') else -999
        
        # MoveIt2 error codes: 1=SUCCESS, -1 to -10 各种失败
        if error_code == 1:
            self.get_logger().info('运动执行成功')
            return True
        else:
            error_names = {
                1: 'SUCCESS', -1: 'FAILURE', -2: 'PLANNING_FAILED',
                -3: 'INVALID_MOTION_PLAN', -4: 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE',
                -5: 'CONTROL_FAILED', -6: 'UNABLE_TO_AQUIRE_SENSOR_DATA',
                -7: 'TIMED_OUT', -10: 'START_STATE_IN_COLLISION',
                -12: 'GOAL_IN_COLLISION', -31: 'NO_IK_SOLUTION'
            }
            error_name = error_names.get(error_code, f'UNKNOWN({error_code})')
            self.get_logger().error(f'运动执行失败: {error_name}')
            return False

    def move_to_joint_target(self, joint_positions: list) -> bool:
        """使用关节目标进行运动规划
        
        Args:
            joint_positions: 6个关节的目标角度(弧度)
        """
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup Action Server 不可用')
            return False

        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.get_logger().info(f'关节目标: {[f"{np.degrees(j):.1f}°" for j in joint_positions]}')

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = self.planning_group
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.05

        # 关节约束
        goal_constraints = Constraints()
        for i, (name, pos) in enumerate(zip(joint_names, joint_positions)):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal_constraints.joint_constraints.append(jc)

        goal.request.goal_constraints.append(goal_constraints)
        
        # 路径约束 - 保持末端姿态稳定，减少译异旋转
        path_orientation = OrientationConstraint()
        path_orientation.header.frame_id = self.base_frame
        path_orientation.link_name = self.end_effector_link
        path_orientation.orientation.x = 1.0
        path_orientation.orientation.y = 0.0
        path_orientation.orientation.z = 0.0
        path_orientation.orientation.w = 0.0
        path_orientation.absolute_x_axis_tolerance = 0.5
        path_orientation.absolute_y_axis_tolerance = 0.5
        path_orientation.absolute_z_axis_tolerance = 0.5
        path_orientation.weight = 1.0
        goal.request.path_constraints.orientation_constraints.append(path_orientation)
        
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True

        # 发送并等待 (轮询方式)
        send_goal_future = self.move_group_client.send_goal_async(goal)
        
        timeout = 10.0
        start_time = self.get_clock().now().nanoseconds / 1e9
        while not send_goal_future.done():
            if (self.get_clock().now().nanoseconds / 1e9 - start_time) > timeout:
                self.get_logger().error('发送关节目标超时')
                return False
            time.sleep(0.1)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('关节目标被拒绝')
            return False

        result_future = goal_handle.get_result_async()
        timeout = 120.0
        start_time = self.get_clock().now().nanoseconds / 1e9
        while not result_future.done():
            if (self.get_clock().now().nanoseconds / 1e9 - start_time) > timeout:
                self.get_logger().error('关节运动超时')
                return False
            time.sleep(0.1)

        if result_future.result() is None:
            self.get_logger().error('关节运动超时')
            return False

        result = result_future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('关节运动成功')
            return True
        else:
            self.get_logger().error(f'关节运动失败: status={result.status}')
            return False

    def check_workspace_limits(self, pose: Pose) -> tuple:
        """检查位置是否在工作范围内，超出则限制
        
        xArm6 工作范围约 0.7m
        Returns: (is_valid, clamped_pose, warning_msg)
        """
        x, y, z = pose.position.x, pose.position.y, pose.position.z
        
        # 计算水平距离
        horizontal_dist = np.sqrt(x**2 + y**2)
        max_reach = 0.65  # 保守值，留余量
        min_reach = 0.15  # 最小可达距离
        min_z = 0.05      # 最小高度（避免碰撞桌面）
        max_z = 0.8       # 最大高度
        
        warnings = []
        clamped = Pose()
        clamped.orientation = pose.orientation
        
        # 检查并限制水平距离
        if horizontal_dist > max_reach:
            scale = max_reach / horizontal_dist
            clamped.position.x = x * scale
            clamped.position.y = y * scale
            warnings.append(f'水平距离 {horizontal_dist:.3f}m 超出范围，限制到 {max_reach}m')
        elif horizontal_dist < min_reach:
            scale = min_reach / horizontal_dist if horizontal_dist > 0.01 else 1.0
            clamped.position.x = x * scale
            clamped.position.y = y * scale
            warnings.append(f'水平距离 {horizontal_dist:.3f}m 太近，调整到 {min_reach}m')
        else:
            clamped.position.x = x
            clamped.position.y = y
        
        # 检查并限制Z高度
        if z < min_z:
            clamped.position.z = min_z
            warnings.append(f'高度 {z:.3f}m 太低，限制到 {min_z}m')
        elif z > max_z:
            clamped.position.z = max_z
            warnings.append(f'高度 {z:.3f}m 太高，限制到 {max_z}m')
        else:
            clamped.position.z = z
        
        is_valid = len(warnings) == 0
        return is_valid, clamped, '; '.join(warnings) if warnings else ''

    def _auto_execute_grasp(self, target_pose: Pose):
        """自动执行抓取序列（在单独线程中运行）
        
        Args:
            target_pose: 目标位姿
        """
        try:
            target = target_pose

            # 检查工作范围
            is_valid, clamped_target, warning = self.check_workspace_limits(target)
            if not is_valid:
                self.get_logger().warn(f'目标超出工作范围: {warning}')
                self.get_logger().info(f'限制后: x={clamped_target.position.x:.3f}, y={clamped_target.position.y:.3f}, z={clamped_target.position.z:.3f}')
                target = clamped_target

            self.get_logger().info('=== 自动抓取开始 ===')

            # 1. 打开夹爪
            self.get_logger().info('Step 1: 打开夹爪')
            self.gripper_control(0.0)

            # 2. 移动到接近点
            approach_pose = Pose()
            approach_pose.position.x = target.position.x
            approach_pose.position.y = target.position.y
            approach_pose.position.z = target.position.z + self.approach_height
            approach_pose.orientation.x = 1.0
            approach_pose.orientation.w = 0.0

            self.get_logger().info(f'Step 2: 移动到接近点 Z={approach_pose.position.z:.3f}m')
            if not self.plan_and_execute(approach_pose):
                self.get_logger().error('移动到接近点失败')
                return

            # 3. 下降到抓取点
            grasp_pose = Pose()
            grasp_pose.position.x = target.position.x
            grasp_pose.position.y = target.position.y
            grasp_pose.position.z = target.position.z + self.grasp_height_offset
            grasp_pose.orientation.x = 1.0
            grasp_pose.orientation.w = 0.0

            self.get_logger().info(f'Step 3: 下降到抓取点 Z={grasp_pose.position.z:.3f}m')
            if not self.plan_and_execute(grasp_pose):
                self.get_logger().error('下降到抓取点失败')
                return

            # 4. 关闭夹爪
            self.get_logger().info('Step 4: 关闭夹爪')
            self.gripper_control(0.85)

            # 5. 抬起
            lift_pose = Pose()
            lift_pose.position.x = target.position.x
            lift_pose.position.y = target.position.y
            lift_pose.position.z = target.position.z + self.approach_height + 0.05
            lift_pose.orientation.x = 1.0
            lift_pose.orientation.w = 0.0

            self.get_logger().info(f'Step 5: 抬起 Z={lift_pose.position.z:.3f}m')
            if not self.plan_and_execute(lift_pose):
                self.get_logger().error('抬起失败')
                return

            # 6. 回到初始位置
            self.get_logger().info('Step 6: 回到初始位置')
            home_joints = [
                np.radians(-88), np.radians(-23), np.radians(-69),
                np.radians(0), np.radians(88), np.radians(-86),
            ]
            self.move_to_joint_target(home_joints)

            # 7. 打开夹爪释放物体
            self.get_logger().info('Step 7: 打开夹爪')
            self.gripper_control(0.0)

            self.get_logger().info('=== 自动抓取完成 ===')

        except Exception as e:
            self.get_logger().error(f'自动执行出错: {e}')
        finally:
            self.is_executing = False
            # 进入冷却期，期间不接受新目标
            self.is_cooling_down = True
            self.get_logger().info(f'进入冷却期 {self.cooldown_duration}s，移除目标后再放置新目标')
            time.sleep(self.cooldown_duration)
            self.is_cooling_down = False
            self.get_logger().info('冷却期结束，可接受新目标')

    def execute_grasp_callback(self, request, response):
        """执行完整抓取序列"""
        with self.target_lock:
            if self.current_target_pose is None:
                response.success = False
                response.message = '没有可用的目标位姿'
                return response
            target = self.current_target_pose.pose

        # 检查工作范围
        is_valid, clamped_target, warning = self.check_workspace_limits(target)
        if not is_valid:
            self.get_logger().warn(f'目标超出工作范围: {warning}')
            self.get_logger().info(f'原始: x={target.position.x:.3f}, y={target.position.y:.3f}, z={target.position.z:.3f}')
            self.get_logger().info(f'限制后: x={clamped_target.position.x:.3f}, y={clamped_target.position.y:.3f}, z={clamped_target.position.z:.3f}')
            target = clamped_target

        self.get_logger().info('开始抓取序列...')

        # 1. 打开夹爪 (失败不阻止后续运动)
        self.get_logger().info('Step 1: 打开夹爪')
        gripper_ok = self.gripper_control(0.0)
        if not gripper_ok:
            self.get_logger().warn('夹爪打开失败，继续执行运动...')

        # 2. 移动到接近点 (目标上方)
        approach_pose = Pose()
        approach_pose.position.x = target.position.x
        approach_pose.position.y = target.position.y
        approach_pose.position.z = target.position.z + self.approach_height
        approach_pose.orientation = target.orientation

        self.get_logger().info(f'Step 2: 移动到接近点 Z={approach_pose.position.z:.3f}m')
        if not self.plan_and_execute(approach_pose):
            response.success = False
            response.message = '移动到接近点失败'
            return response

        # 3. 下降到抓取点
        grasp_pose = Pose()
        grasp_pose.position.x = target.position.x
        grasp_pose.position.y = target.position.y
        grasp_pose.position.z = target.position.z + self.grasp_height_offset
        grasp_pose.orientation = target.orientation

        self.get_logger().info(f'Step 3: 下降到抓取点 Z={grasp_pose.position.z:.3f}m')
        if not self.plan_and_execute(grasp_pose):
            response.success = False
            response.message = '下降到抓取点失败'
            return response

        # 4. 关闭夹爪 (失败不阻止后续运动)
        self.get_logger().info('Step 4: 关闭夹爪')
        gripper_ok = self.gripper_control(0.85)
        if not gripper_ok:
            self.get_logger().warn('夹爪关闭失败，继续执行运动...')

        # 5. 抬起
        lift_pose = Pose()
        lift_pose.position.x = target.position.x
        lift_pose.position.y = target.position.y
        lift_pose.position.z = target.position.z + self.approach_height + 0.05
        lift_pose.orientation = target.orientation

        self.get_logger().info(f'Step 5: 抬起 Z={lift_pose.position.z:.3f}m')
        if not self.plan_and_execute(lift_pose):
            response.success = False
            response.message = '抬起失败'
            return response

        # 6. 回到初始位置 (使用关节目标)
        self.get_logger().info('Step 6: 回到初始位置')
        # 初始关节角度 (度转弧度): -88, -23, -69, 0, 88, -86
        home_joints = [
            np.radians(-88),  # joint1
            np.radians(-23),  # joint2
            np.radians(-69),  # joint3
            np.radians(0),    # joint4
            np.radians(88),   # joint5
            np.radians(-86),  # joint6
        ]
        
        if not self.move_to_joint_target(home_joints):
            self.get_logger().warn('回到初始位置失败，但抓取已完成')

        self.get_logger().info('抓取序列完成!')
        response.success = True
        response.message = '抓取成功'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ArmMotionPlannerNode()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
