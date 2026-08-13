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
from launch.conditions import IfCondition
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

    yesense_net_launch_path = PathJoinSubstitution(
        [FindPackageShare('yesense_std_ros2'), 'launch', 'yesense_net_node.launch.py']
    )

    laser_filter_config = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'config', 'laser_filter.yaml']
    )

    laser_filter_slam_config = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'config', 'laser_filter_slam.yaml']
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

        DeclareLaunchArgument(
            name='use_yesense_imu',
            default_value='true',
            description='Launch YESENSE network IMU'
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

        # Launch YESENSE network IMU
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(yesense_net_launch_path),
            condition=IfCondition(LaunchConfiguration('use_yesense_imu')),
        ),

        # Laser filter: obstacle avoidance, 240-deg FOV -> /scan_filtered
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

        # Laser filter: AMCL localization, 270-deg FOV -> /scan_slam
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter_slam',
            parameters=[laser_filter_slam_config],
            remappings=[
                ('scan', 'scan'),
                ('scan_filtered', 'scan_slam')
            ]
        ),

        # Launch sensors
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensors_launch_path),
        ),

        # Record driven path and publish to /driven_path (TRANSIENT_LOCAL)
        Node(
            package='path_recorder',
            executable='path_recorder_node.py',
            name='path_recorder',
            parameters=[{
                'odom_topic': 'odom',
                'frame_id': 'odom',
                'min_dist_m': 0.05,
            }],
            output='screen',
        ),

        # Publish waypoints from dadian.txt as MarkerArray to /waypoint_markers (TRANSIENT_LOCAL)
        Node(
            package='path_recorder',
            executable='waypoint_marker_node.py',
            name='waypoint_marker',
            parameters=[{
                'frame_id': 'map',
                'publish_interval_s': 5.0,
            }],
            output='screen',
        ),
    ])
