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
            default_value='0.80',
            description='目标点 X 最大限位 (m)'
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
                'gripper_open_deg': LaunchConfiguration('gripper_open_deg'),
                'auto_execute_interval': LaunchConfiguration('auto_execute_interval'),
                'approach_height': LaunchConfiguration('approach_height'),
                'target_x_min': LaunchConfiguration('target_x_min'),
                'target_x_max': LaunchConfiguration('target_x_max'),
                'target_y_min': LaunchConfiguration('target_y_min'),
                'target_y_max': LaunchConfiguration('target_y_max'),
                'target_z_min': LaunchConfiguration('target_z_min'),
                'target_z_max': LaunchConfiguration('target_z_max'),
                'grasp_height_offset': 0.02,
            }]
        ),
    ])
