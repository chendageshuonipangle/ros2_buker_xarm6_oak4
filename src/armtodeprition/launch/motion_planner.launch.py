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
            'approach_height',
            default_value='0.05',
            description='接近高度 (m)'
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
                'approach_height': LaunchConfiguration('approach_height'),
                'grasp_height_offset': 0.02,
            }]
        ),
    ])
