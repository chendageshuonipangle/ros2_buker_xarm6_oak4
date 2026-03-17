#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'target_label',
            default_value='orange',
            description='过滤的目标类别名称'
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='link_base',
            description='机械臂基座坐标系'
        ),

        Node(
            package='oaktf_trantoarm',
            executable='transform_node',
            name='oak_tf_transform_node',
            output='screen',
            parameters=[{
                'target_label': LaunchConfiguration('target_label'),
                'base_frame': LaunchConfiguration('base_frame'),
                # 手眼标定矩阵: 相机位置(后方7.5cm, 右方11cm, 高度21cm), 朝向前方
                'r_base_cam': [
                     0.0,  0.0,  1.0,
                    -1.0,  0.0,  0.0,
                     0.0, -1.0,  0.0,
                ],
                't_base_cam_mm': [
                    -75.0,
                    -110.0,
                    210.0,
                ],
            }]
        ),
    ])
