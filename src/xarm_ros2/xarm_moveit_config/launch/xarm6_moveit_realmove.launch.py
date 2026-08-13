#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default='false')
    add_gripper = LaunchConfiguration('add_gripper', default='false')
    add_arc_gripper = LaunchConfiguration('add_arc_gripper', default='false')
    arc_gripper_left_xyz = LaunchConfiguration('arc_gripper_left_xyz', default='"0.011717 -0.038337 0.058564"')
    arc_gripper_left_rpy = LaunchConfiguration('arc_gripper_left_rpy', default='"0 -1.5707963267948966 0"')
    arc_gripper_right_xyz = LaunchConfiguration('arc_gripper_right_xyz', default='"0.010000 0.096583 0.058564"')
    arc_gripper_right_rpy = LaunchConfiguration('arc_gripper_right_rpy', default='"0 -1.5707963267948966 0"')
    add_oak_d_sr = LaunchConfiguration('add_oak_d_sr', default='false')
    add_oak_d_sr_camera_collision = LaunchConfiguration('add_oak_d_sr_camera_collision', default='false')
    oak_d_sr_camera_xyz = LaunchConfiguration('oak_d_sr_camera_xyz', default='"0 0 0"')
    oak_d_sr_camera_rpy = LaunchConfiguration('oak_d_sr_camera_rpy', default='"0 0 0"')
    oak_d_sr_camera_safety_radius = LaunchConfiguration('oak_d_sr_camera_safety_radius', default='0.065')
    
    # robot moveit realmove launch
    # xarm_moveit_config/launch/_robot_moveit_realmove.launch.py
    robot_moveit_realmove_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_robot_moveit_realmove.launch.py'])),
        launch_arguments={
            'robot_ip': robot_ip,
            'dof': '6',
            'robot_type': 'xarm',
            'hw_ns': hw_ns,
            'no_gui_ctrl': no_gui_ctrl,
            'add_gripper': add_gripper,
            'add_arc_gripper': add_arc_gripper,
            'arc_gripper_left_xyz': arc_gripper_left_xyz,
            'arc_gripper_left_rpy': arc_gripper_left_rpy,
            'arc_gripper_right_xyz': arc_gripper_right_xyz,
            'arc_gripper_right_rpy': arc_gripper_right_rpy,
            'add_oak_d_sr': add_oak_d_sr,
            'add_oak_d_sr_camera_collision': add_oak_d_sr_camera_collision,
            'oak_d_sr_camera_xyz': oak_d_sr_camera_xyz,
            'oak_d_sr_camera_rpy': oak_d_sr_camera_rpy,
            'oak_d_sr_camera_safety_radius': oak_d_sr_camera_safety_radius,
        }.items(),
    )
    
    return LaunchDescription([
        robot_moveit_realmove_launch
    ])
