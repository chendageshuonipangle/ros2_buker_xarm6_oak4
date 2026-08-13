#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_base = LaunchConfiguration('enable_base')
    enable_arm = LaunchConfiguration('enable_arm')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_transform = LaunchConfiguration('enable_transform')
    enable_grasp = LaunchConfiguration('enable_grasp')
    enable_path_planner = LaunchConfiguration('enable_path_planner')
    navigation_mode = LaunchConfiguration('navigation_mode')
    camera_mode = LaunchConfiguration('camera_mode')

    bringup_launch = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'launch', 'bringup.launch.py']
    )
    navigation_launch = PathJoinSubstitution(
        [FindPackageShare('linorobot2_navigation'), 'launch', 'navigation.launch.py']
    )
    slam_navigation_launch = PathJoinSubstitution(
        [FindPackageShare('linorobot2_navigation'), 'launch', 'slam_navigation.launch.py']
    )
    xarm_moveit_launch = PathJoinSubstitution(
        [FindPackageShare('xarm_moveit_config'), 'launch', '_robot_moveit_realmove.launch.py']
    )

    launch_actions = [
        DeclareLaunchArgument('enable_base', default_value='true', description='Whether to start the Bunker base bringup'),
        DeclareLaunchArgument('use_bunker', default_value='true', description='Forwarded to linorobot2 bringup.launch.py'),
        DeclareLaunchArgument('port_name', default_value='can0', description='CAN interface for the Bunker base'),
        DeclareLaunchArgument('is_bunker_mini', default_value='true', description='Whether the base is Bunker Mini'),
        DeclareLaunchArgument('enable_navigation', default_value='true', description='Deprecated switch retained for compatibility'),
        DeclareLaunchArgument('navigation_mode', default_value='slam_nav', description='Navigation mode: none, nav, slam_nav'),
        DeclareLaunchArgument('nav_rviz', default_value='false', description='Whether to start RViz for Nav2'),
        DeclareLaunchArgument('map', default_value='', description='Static map yaml path used when navigation_mode=nav'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0', description='Initial robot x pose for Nav2'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0', description='Initial robot y pose for Nav2'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0', description='Initial robot yaw for Nav2'),
        DeclareLaunchArgument('enable_path_planner', default_value='true', description='Whether to start the custom path planner proxy'),
        DeclareLaunchArgument('planner_algorithm', default_value='astar_costmap', description='Path planning algorithm: astar, jps, jps_improved, astar_costmap'),
        DeclareLaunchArgument('replan_interval', default_value='2.0', description='Planner replan interval in seconds'),
        DeclareLaunchArgument('enable_camera', default_value='true', description='Whether to start the OAK camera node'),
        DeclareLaunchArgument('camera_mode', default_value='yolo', description='Camera mode: yolo or chessboard'),
        DeclareLaunchArgument('show_oak_window', default_value='false', description='Whether to display the OAK OpenCV window'),
        DeclareLaunchArgument('publish_rate', default_value='15.0', description='OAK image publish rate in Hz'),
        DeclareLaunchArgument('target_label', default_value='orange', description='Target label used by the transform node'),
        DeclareLaunchArgument('enable_transform', default_value='true', description='Whether to start camera-to-arm transform node'),
        DeclareLaunchArgument('base_frame', default_value='link_base', description='Arm base frame used by transform and grasp nodes'),
        DeclareLaunchArgument('enable_arm', default_value='true', description='Whether to start xArm MoveIt real robot stack'),
        DeclareLaunchArgument('robot_ip', default_value='', description='xArm robot IP address'),
        DeclareLaunchArgument('arm_no_gui_ctrl', default_value='true', description='Disable xArm MoveIt GUI controllers'),
        DeclareLaunchArgument('enable_grasp', default_value='true', description='Whether to start the grasp execution node'),
        DeclareLaunchArgument('auto_execute', default_value='true', description='Whether the grasp node executes targets automatically'),
        DeclareLaunchArgument('auto_execute_interval', default_value='3.0', description='Automatic grasp polling interval in seconds'),
        DeclareLaunchArgument('end_effector_link', default_value='link_eef', description='End effector link for MoveIt grasp planning'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            condition=IfCondition(enable_base),
            launch_arguments={
                'use_bunker': LaunchConfiguration('use_bunker'),
                'port_name': LaunchConfiguration('port_name'),
                'is_bunker_mini': LaunchConfiguration('is_bunker_mini'),
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('enable_navigation'), "' == 'true' and '", navigation_mode, "' == 'nav'"
            ])),
            launch_arguments={
                'sim': 'false',
                'rviz': LaunchConfiguration('nav_rviz'),
                'map': LaunchConfiguration('map'),
                'initial_pose_x': LaunchConfiguration('initial_pose_x'),
                'initial_pose_y': LaunchConfiguration('initial_pose_y'),
                'initial_pose_yaw': LaunchConfiguration('initial_pose_yaw'),
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_navigation_launch),
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('enable_navigation'), "' == 'true' and '", navigation_mode, "' == 'slam_nav'"
            ])),
            launch_arguments={
                'sim': 'false',
                'rviz': LaunchConfiguration('nav_rviz'),
            }.items(),
        ),

        Node(
            package='path_planning',
            executable='jps_planner_node',
            name='jps_planner_node',
            output='screen',
            condition=IfCondition(enable_path_planner),
            parameters=[{
                'algorithm': LaunchConfiguration('planner_algorithm'),
                'replan_interval': LaunchConfiguration('replan_interval'),
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(xarm_moveit_launch),
            condition=IfCondition(enable_arm),
            launch_arguments={
                'robot_ip': LaunchConfiguration('robot_ip'),
                'dof': '6',
                'robot_type': 'xarm',
                'hw_ns': 'xarm',
                'no_gui_ctrl': LaunchConfiguration('arm_no_gui_ctrl'),
                'add_gripper': 'true',
            }.items(),
        ),

        Node(
            package='oak_yolo_py',
            executable='oak_yolo_node',
            name='oak_yolo_node',
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", enable_camera, "' == 'true' and '", camera_mode, "' == 'yolo'"
            ])),
            parameters=[{
                'show_window': LaunchConfiguration('show_oak_window'),
                'publish_rate': LaunchConfiguration('publish_rate'),
            }],
        ),

        Node(
            package='oak_yolo_py',
            executable='oak_chessboard_node',
            name='oak_chessboard_node',
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", enable_camera, "' == 'true' and '", camera_mode, "' == 'chessboard'"
            ])),
            parameters=[{
                'show_window': LaunchConfiguration('show_oak_window'),
            }],
        ),

        Node(
            package='oaktf_trantoarm',
            executable='transform_node',
            name='oak_tf_transform_node',
            output='screen',
            condition=IfCondition(enable_transform),
            parameters=[{
                'target_label': LaunchConfiguration('target_label'),
                'base_frame': LaunchConfiguration('base_frame'),
                'r_base_cam': [
                    0.0, 0.0, 1.0,
                    -1.0, 0.0, 0.0,
                    0.0, -1.0, 0.0,
                ],
                't_base_cam_mm': [-75.0, -110.0, 210.0],
            }],
        ),

        Node(
            package='armtodeprition',
            executable='motion_planner_node',
            name='arm_motion_planner_node',
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", enable_arm, "' == 'true' and '", enable_grasp, "' == 'true'"
            ])),
            parameters=[{
                'planning_group': 'xarm6',
                'base_frame': LaunchConfiguration('base_frame'),
                'end_effector_link': LaunchConfiguration('end_effector_link'),
                'auto_execute': LaunchConfiguration('auto_execute'),
                'auto_execute_interval': LaunchConfiguration('auto_execute_interval'),
            }],
        ),
    ]

    return LaunchDescription(launch_actions)
