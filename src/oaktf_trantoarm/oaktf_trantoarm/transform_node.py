#!/usr/bin/env python3
"""
OAK 相机坐标 -> xArm 基座坐标 转换节点

功能：
1. 订阅 /oak_result 话题（来自 oak_yolo_py 的检测结果）
2. 使用手眼标定矩阵将相机坐标转换为机械臂基座坐标
3. 发布转换后的坐标到 /target_pose_in_base 话题
4. 同时发布 TF 用于 RViz 可视化

输入话题: /oak_result (std_msgs/String) 格式: "label,confidence,x,y,z"
输出话题: /target_pose_in_base (geometry_msgs/PoseStamped)
"""

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class OakTfTransformNode(Node):
    def __init__(self):
        super().__init__('oak_tf_transform_node')

        # ==========================================
        # 手眼标定结果: 相机坐标 -> 机械臂基座坐标 (单位: mm)
        # 相机位置: 后方7.5cm, 右方11cm, 高度21cm
        # 相机朝向: 正向前方
        # ==========================================
        # 旋转矩阵: 相机坐标系 -> 基座坐标系
        # 相机 Z(深度) -> 基座 X(前), 相机 X(右) -> 基座 -Y, 相机 Y(下) -> 基座 -Z
        self.declare_parameter('r_base_cam', [
             0.0,  0.0,  1.0,
            -1.0,  0.0,  0.0,
             0.0, -1.0,  0.0,
        ])
        # 平移向量: 相机在基座坐标系中的位置 (mm)
        # X=-75(后方), Y=-110(右方), Z=210(高度)
        self.declare_parameter('t_base_cam_mm', [
            -75.0,
            -110.0,
            210.0,
        ])
        self.declare_parameter('base_frame', 'link_base')
        self.declare_parameter('target_label', 'orange')  # 过滤目标类别

        # 加载参数
        r_flat = self.get_parameter('r_base_cam').get_parameter_value().double_array_value
        t_flat = self.get_parameter('t_base_cam_mm').get_parameter_value().double_array_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.target_label = self.get_parameter('target_label').get_parameter_value().string_value

        self.R_base_cam = np.array(r_flat).reshape(3, 3)
        self.T_base_cam_mm = np.array(t_flat)

        self.get_logger().info(f'手眼标定矩阵已加载，目标类别: {self.target_label}')
        self.get_logger().info(f'R_base_cam:\n{self.R_base_cam}')
        self.get_logger().info(f'T_base_cam_mm: {self.T_base_cam_mm}')

        # 订阅 OAK 检测结果
        self.oak_sub = self.create_subscription(
            String,
            '/oak_result',
            self.oak_result_callback,
            10
        )

        # 发布转换后的目标位姿
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/target_pose_in_base',
            10
        )

        # TF 广播器（用于 RViz 可视化）
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('OAK TF Transform Node 已启动')
        self.get_logger().info(f'  订阅: /oak_result')
        self.get_logger().info(f'  发布: /target_pose_in_base')

    def transform_cam_to_base(self, cam_xyz_mm: np.ndarray) -> np.ndarray:
        """将相机坐标系下的点转换到机械臂基座坐标系
        
        Args:
            cam_xyz_mm: 相机坐标系下的 [x, y, z]，单位 mm
            
        Returns:
            base_xyz_mm: 基座坐标系下的 [x, y, z]，单位 mm
        """
        base_xyz_mm = self.R_base_cam @ cam_xyz_mm + self.T_base_cam_mm
        return base_xyz_mm

    def oak_result_callback(self, msg: String):
        """处理 OAK 检测结果"""
        try:
            # 解析消息: "label,confidence,x,y,z"
            parts = msg.data.split(',')
            if len(parts) != 5:
                return

            label = parts[0]
            confidence = float(parts[1])
            x_cam = float(parts[2])
            y_cam = float(parts[3])
            z_cam = float(parts[4])

            # 过滤目标类别
            if self.target_label and label != self.target_label:
                return

            # 检查有效深度
            if z_cam <= 0:
                return

            # 相机坐标 (mm)
            cam_xyz_mm = np.array([x_cam, y_cam, z_cam])

            # 转换到基座坐标 (mm)
            base_xyz_mm = self.transform_cam_to_base(cam_xyz_mm)

            # 转换为米
            base_x_m = base_xyz_mm[0] / 1000.0
            base_y_m = base_xyz_mm[1] / 1000.0
            base_z_m = base_xyz_mm[2] / 1000.0

            # 发布 PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.base_frame
            pose_msg.pose.position.x = base_x_m
            pose_msg.pose.position.y = base_y_m
            pose_msg.pose.position.z = base_z_m
            # 默认朝下的姿态 (绕X轴旋转180度)
            pose_msg.pose.orientation.x = 1.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 0.0
            self.pose_pub.publish(pose_msg)

            # 发布 TF 用于可视化
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.base_frame
            t.child_frame_id = f'target_{label}'
            t.transform.translation.x = base_x_m
            t.transform.translation.y = base_y_m
            t.transform.translation.z = base_z_m
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)

            self.get_logger().info(
                f'[{label}] Cam: ({x_cam:.0f}, {y_cam:.0f}, {z_cam:.0f}) mm -> '
                f'Base: ({base_xyz_mm[0]:.1f}, {base_xyz_mm[1]:.1f}, {base_xyz_mm[2]:.1f}) mm',
                throttle_duration_sec=1.0
            )

        except Exception as e:
            self.get_logger().warn(f'解析 oak_result 失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = OakTfTransformNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
