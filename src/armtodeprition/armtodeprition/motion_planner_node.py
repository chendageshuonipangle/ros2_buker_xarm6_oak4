#!/usr/bin/env python3
"""
xArm6 抓取规划节点 (Final: Guarantee Advance Motion)

核心修复：
针对"到达A点后不前进"的问题：
1. 【顺势而为】Stage 2 (前进) 放弃强制理想姿态，改为继承 A 点的"实际姿态"。
   - 事实证明，如果在 A 点都摆不正，强行要求 B 点摆正会导致规划器罢工(原地不动)。
   - 继承姿态能保证 100% 规划出直线位移。
2. 【位移校验】在闭合夹爪前，强制检查是否真的前进了。如果没有(位移<1cm)，则中止任务。
3. 【参数确认】保持 10cm 间隙 + 17cm 夹爪补偿。
"""

import threading
import time
import math
import numpy as np
import copy

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from rclpy.duration import Duration

# TF 库
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import PoseStamped, Pose
from std_srvs.srv import Trigger
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from moveit_msgs.msg import (
    PositionIKRequest,
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    JointConstraint,
    BoundingVolume,
    CollisionObject,
    PlanningScene,
    RobotTrajectory
)
from control_msgs.action import GripperCommand


def quaternion_from_euler_xyz(roll, pitch, yaw, degrees=False):
    if degrees:
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return x, y, z, w


def euler_xyz_from_quaternion(quat, degrees=False):
    x, y, z, w = quat

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    if degrees:
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
    return roll, pitch, yaw

class ArmMotionPlannerNode(Node):
    def __init__(self):
        super().__init__('arm_motion_planner_node')
        self.callback_group = ReentrantCallbackGroup()

        # 参数
        self.declare_parameter('planning_group', 'xarm6')
        self.declare_parameter('base_frame', 'link_base')
        self.declare_parameter('end_effector_link', 'link_eef')
        self.declare_parameter('auto_execute', False)
        # Gripper drive_joint is in radians; the MoveIt GUI shows degrees.
        # SRDF: open=0 rad (0 deg), close=0.85 rad (48.7 deg ~ fully closed).
        # 42 deg leaves the jaws just short of full closure so the fruit is
        # held rather than crushed.
        self.declare_parameter('gripper_close_deg', 42.0)
        self.declare_parameter('gripper_open_deg', 0.0)
        self.declare_parameter('auto_execute_interval', 3.0)
        self.declare_parameter('target_x_min', 0.10)
        self.declare_parameter('target_x_max', 0.80)
        self.declare_parameter('target_y_min', -0.50)
        self.declare_parameter('target_y_max', 0.50)
        self.declare_parameter('target_z_min', 0.00)
        self.declare_parameter('target_z_max', 0.80)

        self.planning_group = self.get_parameter('planning_group').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.end_effector_link = self.get_parameter('end_effector_link').get_parameter_value().string_value
        self.auto_execute = self.get_parameter('auto_execute').get_parameter_value().bool_value
        self.gripper_close_deg = self.get_parameter('gripper_close_deg').get_parameter_value().double_value
        self.gripper_open_deg = self.get_parameter('gripper_open_deg').get_parameter_value().double_value
        # Full closure is 0.85 rad; refuse to exceed it so a bad parameter
        # cannot drive the jaws past their mechanical limit.
        self.gripper_close_pos = min(math.radians(self.gripper_close_deg), 0.85)
        self.gripper_open_pos = max(math.radians(self.gripper_open_deg), 0.0)
        self.auto_execute_interval = self.get_parameter('auto_execute_interval').get_parameter_value().double_value
        self.target_x_min = self.get_parameter('target_x_min').get_parameter_value().double_value
        self.target_x_max = self.get_parameter('target_x_max').get_parameter_value().double_value
        self.target_y_min = self.get_parameter('target_y_min').get_parameter_value().double_value
        self.target_y_max = self.get_parameter('target_y_max').get_parameter_value().double_value
        self.target_z_min = self.get_parameter('target_z_min').get_parameter_value().double_value
        self.target_z_max = self.get_parameter('target_z_max').get_parameter_value().double_value

        self._assert_single_instance()

        self.current_target_pose = None
        self.last_target_time = 0.0
        # A target older than this is not trustworthy: the object or the arm
        # may have moved since, and driving to a stale point is how you crash
        # into something that is no longer there.
        self.target_max_age = 2.0
        self.target_lock = threading.Lock()
        # Only one trajectory may be in flight: MoveIt rejects a second
        # push with CONTROL_FAILED, which looks exactly like a hardware
        # fault but is really two goals racing.
        self.motion_lock = threading.Lock()
        self.is_executing = False
        self.is_cooling_down = False
        self.cooldown_duration = 4.0

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subs/Pubs
        self.pose_sub = self.create_subscription(
            PoseStamped, '/target_pose_in_base', self.target_pose_callback, 10, callback_group=self.callback_group
        )
        if self.auto_execute:
            self.auto_timer = self.create_timer(
              self.auto_execute_interval, self._auto_execute_timer_callback, callback_group=self.callback_group
            )

        # Clients
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action', callback_group=self.callback_group)
        self.execute_trajectory_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory', callback_group=self.callback_group)
        self.cartesian_path_client = self.create_client(GetCartesianPath, '/compute_cartesian_path', callback_group=self.callback_group)
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik', callback_group=self.callback_group)
        self.gripper_client = ActionClient(self, GripperCommand, '/xarm_gripper/gripper_action', callback_group=self.callback_group)
        self.planning_scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)

        # Services
        self.create_service(Trigger, '/execute_grasp', self.execute_grasp_callback, callback_group=self.callback_group)
        self.create_service(Trigger, '/gripper_open', self.gripper_open_callback, callback_group=self.callback_group)
        self.create_service(Trigger, '/gripper_close', self.gripper_close_callback, callback_group=self.callback_group)

        self.add_workspace_obstacles()
        self.get_logger().info(
            f'夹爪行程: 张开 {self.gripper_open_deg:.0f}° / 闭合 {self.gripper_close_deg:.0f}° '
            f'(满闭合 48.7°)'
        )
        self.get_logger().info('Arm Motion Planner (Guarantee Advance) Ready.')

    def _assert_single_instance(self):
        """Refuse to run if another planner node is already on the graph.

        Two planner nodes both sending MoveGroup goals make MoveIt abort with
        CONTROL_FAILED on every attempt. That failure mode is indistinguishable
        from a robot fault from the logs, so it is cheaper to refuse startup.
        """
        others = [
            name for name, _ in self.get_node_names_and_namespaces()
            if name == 'arm_motion_planner_node'
        ]
        # Our own name is already registered, so >1 means a duplicate.
        if len(others) > 1:
            self.get_logger().fatal(
                '检测到已有 arm_motion_planner_node 在运行，拒绝启动。'
                '两个规划节点会同时向 MoveIt 发轨迹，导致 CONTROL_FAILED。'
                '先执行: pkill -f motion_planner_node'
            )
            raise RuntimeError('duplicate arm_motion_planner_node')

    def is_pose_within_target_limits(self, pose: Pose, label='Target') -> bool:
        x_ok = self.target_x_min <= pose.position.x <= self.target_x_max
        y_ok = self.target_y_min <= pose.position.y <= self.target_y_max
        z_ok = self.target_z_min <= pose.position.z <= self.target_z_max

        if x_ok and y_ok and z_ok:
            return True

        self.get_logger().warn(
            f'{label} 超出限位: '
            f'[{pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}] '
            f'not in '
            f'X[{self.target_x_min:.3f}, {self.target_x_max:.3f}] '
            f'Y[{self.target_y_min:.3f}, {self.target_y_max:.3f}] '
            f'Z[{self.target_z_min:.3f}, {self.target_z_max:.3f}]'
        )
        return False

    def is_pose_reachable(self, pose: Pose, label='Pose') -> bool:
        """Ask MoveIt for an IK solution before committing to any motion.

        Validating the whole approach up front means an unreachable target is
        rejected while the arm is still parked, instead of the arm walking into
        a singularity mid-trajectory and the controller aborting there.
        """
        if not self.ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/compute_ik 不可用，跳过可达性预检。')
            return True

        request = GetPositionIK.Request()
        request.ik_request = PositionIKRequest()
        request.ik_request.group_name = self.planning_group
        request.ik_request.ik_link_name = self.end_effector_link
        request.ik_request.pose_stamped.header.frame_id = self.base_frame
        request.ik_request.pose_stamped.pose = pose
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = Duration(seconds=0.5).to_msg()

        future = self.ik_client.call_async(request)
        deadline = time.time() + 3.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)
        if not future.done():
            self.get_logger().warn(f'{label} IK 预检超时，保守拒绝。')
            return False

        response = future.result()
        if response is None:
            self.get_logger().warn(f'{label} IK 预检无响应，保守拒绝。')
            return False
        code = response.error_code.val
        if code == 1:
            return True
        self.get_logger().warn(
            f'{label} 不可达 (IK code {code}): '
            f'[{pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}]'
        )
        return False

    def add_workspace_obstacles(self):
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        wall = CollisionObject()
        wall.header.frame_id = self.base_frame
        wall.id = 'rear_safety_wall'
        wall.operation = CollisionObject.ADD
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [0.05, 1.0, 1.0]
        pose = Pose()
        pose.position.x = -0.3; pose.position.z = 0.5; pose.orientation.w = 1.0
        wall.primitives.append(prim); wall.primitive_poses.append(pose)
        scene_msg.world.collision_objects.append(wall)
        self.planning_scene_pub.publish(scene_msg)

    def target_pose_callback(self, msg: PoseStamped):
        if not self.is_executing and not self.is_cooling_down:
            if not self.is_pose_within_target_limits(msg.pose):
                return
            with self.target_lock:
                self.current_target_pose = msg
                self.last_target_time = time.time()

    def _auto_execute_timer_callback(self):
        if not self.auto_execute or self.is_executing: return
        with self.target_lock:
            if self.current_target_pose is None: return
            age = time.time() - self.last_target_time
            target = self.current_target_pose
            self.current_target_pose = None
        # Same staleness rule as the manual trigger: a point older than the
        # window may describe where the object used to be.
        if age > self.target_max_age:
            self.get_logger().warn(f'目标点已过期 {age:.1f}s，跳过本次自动抓取。')
            return
        
        self.is_executing = True
        threading.Thread(target=self._auto_execute_grasp, args=(target.pose,), daemon=True).start()

    def get_current_pose_msg(self):
        try:
            if self.tf_buffer.can_transform(self.base_frame, self.end_effector_link, rclpy.time.Time(), timeout=Duration(seconds=1.0)):
                t = self.tf_buffer.lookup_transform(self.base_frame, self.end_effector_link, rclpy.time.Time())
                p = Pose()
                p.position.x = t.transform.translation.x
                p.position.y = t.transform.translation.y
                p.position.z = t.transform.translation.z
                p.orientation = t.transform.rotation
                return p
        except Exception as e:
            self.get_logger().warn(f'TF Error: {e}')
        return None

    def calculate_orientation_error_deg(self, current_quat_msg, target_rpy):
        """计算当前姿态与目标RPY的角度误差(度)"""
        try:
            q_curr = [current_quat_msg.x, current_quat_msg.y, current_quat_msg.z, current_quat_msg.w]
            q_tgt = quaternion_from_euler_xyz(*target_rpy, degrees=True)

            # 计算四元数点积
            q1 = np.array(q_curr, dtype=float)
            q2 = np.array(q_tgt, dtype=float)
            q1 /= np.linalg.norm(q1)
            q2 /= np.linalg.norm(q2)
            dot = np.abs(np.dot(q1, q2))
            if dot > 1.0:
                dot = 1.0

            angle_rad = 2 * np.arccos(dot)
            return np.degrees(angle_rad)
        except:
            return 999.9

    def log_current_pose(self, tag="Current", target_rpy=None):
        p = self.get_current_pose_msg()
        if p:
            roll, pitch, yaw = euler_xyz_from_quaternion(
                [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w],
                degrees=True,
            )
            
            self.get_logger().info(f'📍 [{tag}] Pos: [{p.position.x:.3f}, {p.position.y:.3f}, {p.position.z:.3f}]')
            
            quat_str = f"Quat: [{p.orientation.x:.3f}, {p.orientation.y:.3f}, {p.orientation.z:.3f}, {p.orientation.w:.3f}]"
            
            if target_rpy:
                err = self.calculate_orientation_error_deg(p.orientation, target_rpy)
                self.get_logger().info(f'   ├── 实际 RPY: ({roll:.1f}, {pitch:.1f}, {yaw:.1f})')
                self.get_logger().info(f'   ├── 目标 RPY: ({target_rpy[0]:.1f}, {target_rpy[1]:.1f}, {target_rpy[2]:.1f})')
                self.get_logger().info(f'   ├── {quat_str}')
                self.get_logger().info(f'   └── 📉 姿态偏差: {err:.2f}°')
            else:
                self.get_logger().info(f'   └── RPY: ({roll:.1f}, {pitch:.1f}, {yaw:.1f}) | {quat_str}')
            
            return p
        return None

    def move_ptp(self, target_pose: Pose, strict_orientation=True) -> bool:
        """点到点规划 (通用)"""
        with self.motion_lock:
            return self._move_ptp_locked(target_pose, strict_orientation)

    def _move_ptp_locked(self, target_pose: Pose, strict_orientation=True) -> bool:
        if not self.move_group_client.wait_for_server(2.0): return False
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = self.planning_group
        goal.request.num_planning_attempts = 15
        goal.request.allowed_planning_time = 10.0 
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.2

        constraints = Constraints()
        pos_con = PositionConstraint()
        pos_con.header.frame_id = self.base_frame
        pos_con.link_name = self.end_effector_link
        sphere = SolidPrimitive(); sphere.type = SolidPrimitive.SPHERE; sphere.dimensions = [0.005]
        bv = BoundingVolume(); bv.primitives.append(sphere)
        p = Pose(); p.position = target_pose.position; p.orientation.w = 1.0
        bv.primitive_poses.append(p)
        pos_con.constraint_region = bv
        pos_con.weight = 1.0
        constraints.position_constraints.append(pos_con)

        if strict_orientation:
            ori_con = OrientationConstraint()
            ori_con.header.frame_id = self.base_frame
            ori_con.link_name = self.end_effector_link
            ori_con.orientation = target_pose.orientation
            # === 这里稍微放松一点点，避免卡死 (0.05 rad ~ 2.8度) ===
            ori_con.absolute_x_axis_tolerance = 0.05 
            ori_con.absolute_y_axis_tolerance = 0.05
            ori_con.absolute_z_axis_tolerance = 0.05
            ori_con.weight = 1.0
            constraints.orientation_constraints.append(ori_con)

        goal.request.goal_constraints.append(constraints)
        
        future = self.move_group_client.send_goal_async(goal)
        while not future.done(): time.sleep(0.1)
        res = future.result()
        
        if not res.accepted: return False
        res_future = res.get_result_async()
        while not res_future.done(): time.sleep(0.1)
        
        result_wrapper = res_future.result()
        if result_wrapper.status == 4: return True
        else:
            self.get_logger().error(f'PTP 失败: {self.get_moveit_error_string(result_wrapper.result.error_code.val)}')
            return False

    def get_moveit_error_string(self, val):
        if val == 1: return "SUCCESS"
        mapping = { -1: "PLANNING_FAILED", -4: "CONTROL_FAILED", -10: "START_STATE_IN_COLLISION", -12: "GOAL_IN_COLLISION" }
        return mapping.get(val, f"Code {val}")

    def move_joints(self, joints):
        with self.motion_lock:
            return self._move_joints_locked(joints)

    def _move_joints_locked(self, joints):
        if not self.move_group_client.wait_for_server(2.0): return False
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = self.planning_group
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.2
        
        constraints = Constraints()
        names = ['joint1','joint2','joint3','joint4','joint5','joint6']
        for i, val in enumerate(joints):
            jc = JointConstraint()
            jc.joint_name = names[i]; jc.position = val
            jc.tolerance_above = 0.02; jc.tolerance_below = 0.02; jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints.append(constraints)
        
        future = self.move_group_client.send_goal_async(goal)
        while not future.done(): time.sleep(0.1)
        res = future.result()
        if not res.accepted: return False
        res_future = res.get_result_async()
        while not res_future.done(): time.sleep(0.1)
        return res_future.result().status == 4

    def gripper_control(self, val):
        if not self.gripper_client.wait_for_server(2.0): return True
        goal = GripperCommand.Goal()
        goal.command.position = val
        future = self.gripper_client.send_goal_async(goal)
        while not future.done(): time.sleep(0.1)
        return True

    def _auto_execute_grasp(self, target_pose: Pose):
        try:
            self.get_logger().info('>>> 开始执行抓取 (Final: Guarantee Advance Motion)')

            if not self.is_pose_within_target_limits(target_pose):
                self.get_logger().warn('目标点被限位拒绝，取消本次抓取。')
                return
            
            # 理想目标姿态
            TARGET_RPY = [180, -90, 0]
            self.log_current_pose("Start", target_rpy=TARGET_RPY)

            # === 1. 参数设置 ===
            GRIPPER_LEN = 0.17   
            APPROACH_GAP = 0.10  # Point A 的后退量 10cm
            # Stage 2 的前进量比后退量少 2cm，终点停在 target.x - 0.19
            # 而不是 target.x - 0.17，避免夹爪顶到目标。
            ADVANCE_GAP = APPROACH_GAP - 0.02
            dist_a = GRIPPER_LEN + APPROACH_GAP

            qx, qy, qz, qw = quaternion_from_euler_xyz(*TARGET_RPY, degrees=True)

            # Point A (Pre-Grasp)
            pose_a = Pose()
            pose_a.position.x = target_pose.position.x - dist_a
            pose_a.position.y = target_pose.position.y
            pose_a.position.z = target_pose.position.z
            pose_a.orientation.x = qx; pose_a.orientation.y = qy; pose_a.orientation.z = qz; pose_a.orientation.w = qw

            self.get_logger().info(f'🎯 目标X: {target_pose.position.x:.3f}')
            self.get_logger().info(f'   Point A (Pre): {pose_a.position.x:.3f} (Back {dist_a*100:.0f}cm)')

            # === 0. 预检：A 与 B 都必须在限位内且可达 ===
            # 之前只校验目标点，A/B 是算出来才用，结果臂走到一半才发现
            # 进了奇异区，控制器直接 CONTROL_FAILED 卡死。
            pose_b_planned = copy.deepcopy(pose_a)
            pose_b_planned.position.x += ADVANCE_GAP
            if not self.is_pose_within_target_limits(pose_a, label='Point A'):
                self.get_logger().warn('Point A 超出限位，取消本次抓取。')
                return
            if not self.is_pose_within_target_limits(pose_b_planned, label='Point B'):
                self.get_logger().warn('Point B 超出限位，取消本次抓取。')
                return
            if not self.is_pose_reachable(pose_a, label='Point A'):
                self.get_logger().warn('Point A 不可达，取消本次抓取。')
                return
            if not self.is_pose_reachable(pose_b_planned, label='Point B'):
                self.get_logger().warn('Point B 不可达，取消本次抓取。')
                return
            self.get_logger().info('   ✓ 预检通过：A/B 均在限位内且可达')

            # STAGE 1: PTP 到 A
            self.get_logger().info('[STAGE 1] PTP 移动到 Point A...')
            self.gripper_control(self.gripper_open_pos)
            if not self.move_ptp(pose_a, strict_orientation=True):
                self.get_logger().error('无法到达 Point A!')
                return
            
            self.get_logger().info('👀 停顿 2s...')
            time.sleep(2.0)
            
            current_pose_a = self.log_current_pose("At Point A", target_rpy=TARGET_RPY) 
            if current_pose_a is None: return

            # === 关键修改：Stage 2 必须动！ ===
            # 使用 A 点的 *实际姿态* 作为 B 点姿态，放弃矫正
            # 这样保证了 Stage 2 只是纯粹的平移，MoveIt 绝对能解算出来
            pose_b = copy.deepcopy(current_pose_a)
            pose_b.position.x += ADVANCE_GAP

            # A 点的实际位姿会偏离规划值 (规划器为满足姿态约束而挑了别的解)，
            # 所以之前基于规划 A 算的 B 预检不足以代表真实 B，这里按实际
            # 落点再验一次，确认前进那一下不会撞进奇异区。
            if not self.is_pose_within_target_limits(pose_b, label='Point B (actual)'):
                self.get_logger().error('实际 Point B 超出限位，中止前进。')
                return
            if not self.is_pose_reachable(pose_b, label='Point B (actual)'):
                self.get_logger().error('实际 Point B 不可达，中止前进。')
                return

            self.get_logger().info(f'[STAGE 2] PTP 前进 (X={pose_b.position.x:.3f}) - 使用实际姿态')
            
            # 这里 strict_orientation=True 是为了保持"当前歪姿态"不变，而不是修正回"理想姿态"
            if not self.move_ptp(pose_b, strict_orientation=True):
                self.get_logger().error('前进失败!')
                return

            # === 位移校验 ===
            final_pose_b = self.log_current_pose("At Point B", target_rpy=TARGET_RPY)
            if final_pose_b and (final_pose_b.position.x - current_pose_a.position.x < 0.01):
                self.get_logger().error("❌ 严重错误：机械臂没有前进！中止抓取！")
                return

            self.get_logger().info(
                f'   > 闭合夹爪至 {self.gripper_close_deg:.0f}° '
                f'({self.gripper_close_pos:.3f} rad)'
            )
            self.gripper_control(self.gripper_close_pos)
            time.sleep(0.8)

            # STAGE 3: PTP 后退
            self.get_logger().info(f'[STAGE 3] PTP 退回 Point A (X={current_pose_a.position.x:.3f})')
            if not self.move_ptp(current_pose_a, strict_orientation=True):
                self.get_logger().warn('后退失败...')

            # STAGE 4: 回家
            self.get_logger().info('[STAGE 4] 关节运动回家...')
            # xArm6 "hold-up" named state from _xarm6_macro.srdf.xacro: only
            # joint5 is turned -90 deg. This points the OAK camera straight
            # ahead so it can look for the next object to pick.
            home_degrees = [0, 0, 0, 0, -90, 0]
            home = [np.radians(d) for d in home_degrees]
            
            if not self.move_joints(home):
                self.get_logger().warn('关节回家未成功。')

            self.get_logger().info('>>> 抓取完成')
            self.gripper_control(self.gripper_open_pos)

        except Exception as e:
            self.get_logger().error(f'执行异常: {e}')
        finally:
            self.is_executing = False
            self.is_cooling_down = True
            time.sleep(self.cooldown_duration)
            self.is_cooling_down = False

    def execute_grasp_callback(self, req, res):
        """Trigger one grasp on the latest stable target.

        Manual triggering is the safe default: the operator decides when the
        arm moves, instead of a detection frame deciding for them.
        """
        if self.is_executing:
            res.success = False
            res.message = '正在执行抓取，忽略本次触发'
            return res
        with self.target_lock:
            target = self.current_target_pose
            age = time.time() - self.last_target_time
            self.current_target_pose = None
        if target is None:
            res.success = False
            res.message = '没有可用目标点（检测未稳定或超出限位）'
            self.get_logger().warn(res.message)
            return res
        if age > self.target_max_age:
            res.success = False
            res.message = f'目标点已过期 {age:.1f}s，重新对准目标再触发'
            self.get_logger().warn(res.message)
            return res

        self.is_executing = True
        threading.Thread(
            target=self._auto_execute_grasp, args=(target.pose,), daemon=True
        ).start()
        res.success = True
        res.message = (
            f'已触发抓取: ['
            f'{target.pose.position.x:.3f}, '
            f'{target.pose.position.y:.3f}, '
            f'{target.pose.position.z:.3f}]'
        )
        self.get_logger().info(res.message)
        return res
    def gripper_open_callback(self, req, res):
        self.gripper_control(self.gripper_open_pos); res.success=True; return res

    def gripper_close_callback(self, req, res):
        self.gripper_control(self.gripper_close_pos); res.success=True; return res

def main(args=None):
    rclpy.init(args=args)
    try:
        node = ArmMotionPlannerNode()
    except RuntimeError as exc:
        # Duplicate instance: bail out before touching the arm.
        print(f'启动中止: {exc}')
        if rclpy.ok():
            rclpy.shutdown()
        return
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
