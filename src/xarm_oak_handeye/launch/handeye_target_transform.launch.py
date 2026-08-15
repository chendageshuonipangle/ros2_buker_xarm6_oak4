import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_result = os.path.join(
        get_package_share_directory("xarm_oak_handeye"),
        "config",
        "eye_in_hand_result.json",
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "result",
            default_value=default_result,
            description="eye_in_hand_result.json path",
        ),
        DeclareLaunchArgument("base_frame", default_value="link_base"),
        DeclareLaunchArgument("tcp_frame", default_value="link_tcp"),
        DeclareLaunchArgument("camera_frame", default_value="oak_rgb_camera_optical_frame"),
        DeclareLaunchArgument("target_label", default_value=""),
        DeclareLaunchArgument(
            "run_detector",
            default_value="true",
            description="also start the OAK NPU detector on /oak_result",
        ),
        DeclareLaunchArgument("confidence", default_value="0.65"),
        DeclareLaunchArgument("fps", default_value="18.0"),
        DeclareLaunchArgument("preview_fps", default_value="5.0"),
        DeclareLaunchArgument("fullscreen", default_value="true"),
        DeclareLaunchArgument(
            "show",
            default_value="true",
            description="open the detector preview window",
        ),
        DeclareLaunchArgument("min_depth_mm", default_value="150.0"),
        DeclareLaunchArgument("max_depth_mm", default_value="700.0"),
        DeclareLaunchArgument(
            "track_tolerance_mm",
            default_value="60.0",
            description="视为同一目标的最大帧间跳变，超过则重新判稳",
        ),
        Node(
            package="xarm_oak_handeye",
            executable="publish_handeye_tf",
            name="xarm_oak_handeye_static_tf",
            arguments=[
                "--result", LaunchConfiguration("result"),
                "--parent-frame", LaunchConfiguration("tcp_frame"),
                "--child-frame", LaunchConfiguration("camera_frame"),
            ],
        ),
        Node(
            package="xarm_oak_handeye",
            executable="oak_peach_detector",
            name="oak_peach_detector",
            condition=IfCondition(LaunchConfiguration("run_detector")),
            output="screen",
            arguments=[
                "--target-label", LaunchConfiguration("target_label"),
                "--confidence", LaunchConfiguration("confidence"),
                "--min-depth-mm", LaunchConfiguration("min_depth_mm"),
                "--max-depth-mm", LaunchConfiguration("max_depth_mm"),
                "--show", LaunchConfiguration("show"),
                "--fps", LaunchConfiguration("fps"),
                "--preview-fps", LaunchConfiguration("preview_fps"),
                "--fullscreen", LaunchConfiguration("fullscreen"),
                "--track-tolerance-mm", LaunchConfiguration("track_tolerance_mm"),
            ],
        ),
        Node(
            package="xarm_oak_handeye",
            executable="target_to_base",
            name="oak_target_to_base",
            arguments=[
                "--base-frame", LaunchConfiguration("base_frame"),
                "--camera-frame", LaunchConfiguration("camera_frame"),
                "--target-label", LaunchConfiguration("target_label"),
            ],
        ),
    ])
