#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'planning_group',
            default_value='xarm6',
            description='MoveIt 规划组名称'
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='link_base',
            description='机械臂基座坐标系'
        ),
        DeclareLaunchArgument(
            'end_effector_link',
            default_value='link_eef',
            description='末端执行器 link'
        ),
        DeclareLaunchArgument(
            'auto_execute',
            default_value='true',
            description='true 时检测到稳定目标自动抓取；默认 false，需手动调用 /execute_grasp'
        ),
        DeclareLaunchArgument(
            'auto_execute_interval',
            default_value='3.0',
            description='自动抓取轮询间隔 (s)'
        ),
        DeclareLaunchArgument(
            'gripper_close_deg',
            default_value='42.0',
            description='夹爪闭合角度 (deg)，满闭合约 48.7'
        ),
        DeclareLaunchArgument(
            'gripper_open_deg',
            default_value='0.0',
            description='夹爪张开角度 (deg)'
        ),
        DeclareLaunchArgument(
            'twist_enable',
            default_value='true',
            description='夹住后转 joint6 拧下果子'
        ),
        DeclareLaunchArgument(
            'twist_deg',
            default_value='45.0',
            description='joint6 单侧拧转幅度 (deg)'
        ),
        DeclareLaunchArgument(
            'twist_cycles',
            default_value='2',
            description='拧转轮数，每轮为正转->反转->回中'
        ),
        DeclareLaunchArgument(
            'twist_settle_s',
            default_value='0.4',
            description='每步拧转后的停顿 (s)'
        ),
        DeclareLaunchArgument(
            'grip_settle_s',
            default_value='1.0',
            description='闭合夹爪后等待夹持稳定的时间 (s)，之后才拧转'
        ),
        DeclareLaunchArgument(
            'payload_kg',
            default_value='0.3',
            description='抓取后告知控制器的负载 (kg)，避免误报 C31'
        ),
        DeclareLaunchArgument(
            'auto_recover',
            default_value='true',
            description='控制器故障后自动清错误并重新激活'
        ),
        DeclareLaunchArgument(
            'twist_collision_sensitivity',
            default_value='0',
            description='拧转期间的碰撞灵敏度 0~5，0 为关闭，拧完立即还原'
        ),
        DeclareLaunchArgument(
            'normal_collision_sensitivity',
            default_value='3',
            description='拧转结束后还原的碰撞灵敏度 0~5'
        ),
        DeclareLaunchArgument(
            'loaded_velocity_scale',
            default_value='0.15',
            description='带果子退回/回家的速度比例，越低关节电流越平稳'
        ),
        DeclareLaunchArgument(
            'loaded_acceleration_scale',
            default_value='0.08',
            description='带果子退回/回家的加速度比例'
        ),
        DeclareLaunchArgument(
            'approach_height',
            default_value='0.05',
            description='接近高度 (m)'
        ),
        DeclareLaunchArgument(
            'target_x_min',
            default_value='0.10',
            description='目标点 X 最小限位 (m)'
        ),
        DeclareLaunchArgument(
            'target_x_max',
            default_value='0.95',
            description='果子 X 最大限位 (m)；臂停在 target.x-0.27，不会真伸到这里'
        ),
        DeclareLaunchArgument(
            'arm_x_max',
            default_value='0.80',
            description='TCP 自身 X 最大限位 (m)，约束 Point A/B'
        ),
        DeclareLaunchArgument(
            'limit_log_period_s',
            default_value='5.0',
            description='超限告警最小间隔 (s)，防止按检测帧率刷屏'
        ),
        DeclareLaunchArgument(
            'target_y_min',
            default_value='-0.50',
            description='目标点 Y 最小限位 (m)'
        ),
        DeclareLaunchArgument(
            'target_y_max',
            default_value='0.50',
            description='目标点 Y 最大限位 (m)'
        ),
        DeclareLaunchArgument(
            'target_z_min',
            default_value='0.00',
            description='目标点 Z 最小限位 (m)'
        ),
        DeclareLaunchArgument(
            'target_z_max',
            default_value='0.80',
            description='目标点 Z 最大限位 (m)'
        ),

        Node(
            package='armtodeprition',
            executable='motion_planner_node',
            name='arm_motion_planner_node',
            output='screen',
            parameters=[{
                'planning_group': LaunchConfiguration('planning_group'),
                'base_frame': LaunchConfiguration('base_frame'),
                'end_effector_link': LaunchConfiguration('end_effector_link'),
                'auto_execute': LaunchConfiguration('auto_execute'),
                'gripper_close_deg': LaunchConfiguration('gripper_close_deg'),
                'twist_enable': LaunchConfiguration('twist_enable'),
                'twist_deg': LaunchConfiguration('twist_deg'),
                'twist_cycles': LaunchConfiguration('twist_cycles'),
                'twist_settle_s': LaunchConfiguration('twist_settle_s'),
                'grip_settle_s': LaunchConfiguration('grip_settle_s'),
                'payload_kg': LaunchConfiguration('payload_kg'),
                'auto_recover': LaunchConfiguration('auto_recover'),
                'twist_collision_sensitivity': LaunchConfiguration('twist_collision_sensitivity'),
                'normal_collision_sensitivity': LaunchConfiguration('normal_collision_sensitivity'),
                'loaded_velocity_scale': LaunchConfiguration('loaded_velocity_scale'),
                'loaded_acceleration_scale': LaunchConfiguration('loaded_acceleration_scale'),
                'gripper_open_deg': LaunchConfiguration('gripper_open_deg'),
                'auto_execute_interval': LaunchConfiguration('auto_execute_interval'),
                'approach_height': LaunchConfiguration('approach_height'),
                'target_x_min': LaunchConfiguration('target_x_min'),
                'target_x_max': LaunchConfiguration('target_x_max'),
                'arm_x_max': LaunchConfiguration('arm_x_max'),
                'limit_log_period_s': LaunchConfiguration('limit_log_period_s'),
                'target_y_min': LaunchConfiguration('target_y_min'),
                'target_y_max': LaunchConfiguration('target_y_max'),
                'target_z_min': LaunchConfiguration('target_z_min'),
                'target_z_max': LaunchConfiguration('target_z_max'),
                'grasp_height_offset': 0.02,
            }]
        ),
    ])
