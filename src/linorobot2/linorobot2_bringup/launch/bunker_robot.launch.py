# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    sensors_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'launch', 'sensors.launch.py']
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_description'), 'launch', 'description.launch.py']
    )

    bunker_base_launch_path = PathJoinSubstitution(
        [FindPackageShare('bunker_base'), 'launch', 'bunker_base.launch.py']
    )

    rplidar_launch_path = PathJoinSubstitution(
        [FindPackageShare('sllidar_ros2'), 'launch', 'sllidar_a1_launch.py']
    )

    laser_filter_config = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'config', 'laser_filter.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='rviz',
            default_value='true',
            description='Launch RViz'
        ),

        DeclareLaunchArgument(
            name='port_name',
            default_value='can0',
            description='CAN bus name for Bunker robot'
        ),

        DeclareLaunchArgument(
            name='is_bunker_mini',
            default_value='false',
            description='Set to true if using Bunker Mini'
        ),

        DeclareLaunchArgument(
            name='odom_frame',
            default_value='odom',
            description='Odometry frame id'
        ),

        DeclareLaunchArgument(
            name='base_frame',
            default_value='base_link',
            description='Base link frame id'
        ),

        DeclareLaunchArgument(
            name='odom_topic_name',
            default_value='odom/unfiltered',
            description='Odometry topic name from bunker_base'
        ),

        # Launch Bunker base driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bunker_base_launch_path),
            launch_arguments={
                'port_name': LaunchConfiguration('port_name'),
                'is_bunker_mini': LaunchConfiguration('is_bunker_mini'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'odom_topic_name': LaunchConfiguration('odom_topic_name'),
                'use_sim_time': 'false',
                'simulated_robot': 'false'
            }.items()
        ),

        # Launch robot description with rviz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'rviz': LaunchConfiguration('rviz')
            }.items()
        ),

        # Launch RPLIDAR A1 (output to /scan_raw)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch_path)
        ),

        # Laser filter: 过滤后方 120° 遮挡区域
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter',
            parameters=[laser_filter_config],
            remappings=[
                ('scan', 'scan'),
                ('scan_filtered', 'scan_filtered')
            ]
        ),

        # Launch sensors
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensors_launch_path),
        )
    ])
