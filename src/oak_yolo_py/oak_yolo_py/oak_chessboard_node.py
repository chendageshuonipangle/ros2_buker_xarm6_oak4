#!/usr/bin/env python3
"""
OAK 棋盘格检测节点 - 用于测试手眼标定

功能：
1. 使用 OAK-D Pro 检测 8x8 棋盘格
2. 使用 solvePnP 计算棋盘中心的 3D 坐标（相机坐标系）
3. 发布坐标到 /oak_result 话题（与 oak_yolo_node 格式一致）
4. 同时发布图像到 ROS2 话题

输出话题:
  /oak_result (std_msgs/String) 格式: "chessboard,1.0,x,y,z"
  /oak/rgb/image_raw (sensor_msgs/Image)
  /oak/depth/image_raw (sensor_msgs/Image)
"""

from pathlib import Path

import cv2
import depthai as dai
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# 棋盘参数：9x9 格子 => 8x8 内角点，每格 6mm
CHECKERBOARD = (8, 8)  # (cols, rows) 内角点数量
SQUARE_SIZE_MM = 6.0

# 默认相机内参（OAK-D Pro 1280x720）
# 如果有标定文件，会覆盖这些值
DEFAULT_K = np.array([
    [860.0, 0.0, 640.0],
    [0.0, 860.0, 360.0],
    [0.0, 0.0, 1.0]
], dtype=np.float32)
DEFAULT_DIST = np.zeros((5,), dtype=np.float32)


def load_camera_calib(calib_path: Path):
    """从 YAML 文件加载相机标定参数"""
    if not calib_path.exists():
        print(f"⚠️ 标定文件不存在: {calib_path}，使用默认内参")
        return DEFAULT_K, DEFAULT_DIST

    fs = cv2.FileStorage(str(calib_path), cv2.FILE_STORAGE_READ)
    camera_matrix = fs.getNode("camera_matrix").mat()
    dist_coeffs = fs.getNode("dist_coeffs").mat()
    fs.release()

    if camera_matrix is None or dist_coeffs is None:
        print("⚠️ 无法从标定文件读取参数，使用默认内参")
        return DEFAULT_K, DEFAULT_DIST

    return camera_matrix, dist_coeffs


def compute_chessboard_3d_center(corners: np.ndarray, K: np.ndarray, dist: np.ndarray):
    """使用 solvePnP 计算棋盘中心在相机坐标系下的 3D 坐标 (mm)"""
    # 构造棋盘在棋盘坐标系下的 3D 角点 (Z=0 平面)
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE_MM  # 单位: mm

    # 像素坐标
    imgp = corners.reshape(-1, 2).astype(np.float32)

    # 使用 solvePnP 估计棋盘相对于相机的位姿
    ok, rvec, tvec = cv2.solvePnP(objp, imgp, K, dist)
    if not ok:
        return None

    R, _ = cv2.Rodrigues(rvec)

    # 棋盘中心点在棋盘坐标系下的坐标 (所有角点 3D 坐标的平均值)
    center_obj = objp.mean(axis=0).reshape(3, 1)  # (3,1)

    # 转到相机坐标系: X_cam = R * X_obj + t
    center_cam = R @ center_obj + tvec  # (3,1)
    center_cam_mm = center_cam.flatten()

    return center_cam_mm


class OakChessboardPublisher(Node):
    def __init__(self):
        super().__init__('oak_chessboard_node')

        # 参数
        self.declare_parameter('show_window', True)
        self.declare_parameter('calib_file', '')

        self.show_window = self.get_parameter('show_window').get_parameter_value().bool_value
        calib_file = self.get_parameter('calib_file').get_parameter_value().string_value

        # 加载相机标定
        if calib_file:
            self.K, self.dist = load_camera_calib(Path(calib_file))
        else:
            self.K, self.dist = DEFAULT_K, DEFAULT_DIST
            self.get_logger().info('使用默认相机内参')

        # 发布器
        self.pub = self.create_publisher(String, '/oak_result', 10)
        self.rgb_pub = self.create_publisher(Image, '/oak/rgb/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/oak/depth/image_raw', 10)

        self.bridge = CvBridge()

        self.get_logger().info('OAK 棋盘格检测节点已启动')
        self.get_logger().info(f'  发布: /oak_result (格式: chessboard,1.0,x,y,z)')
        self.get_logger().info(f'  显示窗口: {self.show_window}')

    def publish_result(self, x_mm: float, y_mm: float, z_mm: float):
        """发布棋盘中心坐标"""
        msg = String()
        msg.data = f"chessboard,1.0000,{int(x_mm)},{int(y_mm)},{int(z_mm)}"
        self.pub.publish(msg)

    def publish_images(self, rgb_frame, depth_frame, annotated_frame):
        """发布图像到 ROS2 话题"""
        # RGB
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_frame, encoding='bgr8')
        rgb_msg.header.stamp = self.get_clock().now().to_msg()
        rgb_msg.header.frame_id = 'oak_rgb_camera_optical_frame'
        self.rgb_pub.publish(rgb_msg)

        # Depth (彩色化)
        if depth_frame is not None:
            h, w = rgb_frame.shape[:2]
            depth_resized = cv2.resize(depth_frame, (w, h), interpolation=cv2.INTER_NEAREST)
            valid_mask = depth_resized > 0
            if valid_mask.any():
                d_min = np.percentile(depth_resized[valid_mask], 5)
                d_max = np.percentile(depth_resized[valid_mask], 95)
            else:
                d_min, d_max = 0, 1
            depth_norm = np.clip((depth_resized - d_min) / max(d_max - d_min, 1), 0, 1)
            depth_u8 = (depth_norm * 255).astype(np.uint8)
            depth_color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_JET)

            depth_msg = self.bridge.cv2_to_imgmsg(depth_color, encoding='bgr8')
            depth_msg.header.stamp = self.get_clock().now().to_msg()
            depth_msg.header.frame_id = 'oak_rgb_camera_optical_frame'
            self.depth_pub.publish(depth_msg)


def main(args=None):
    print("=" * 50)
    print("  OAK 棋盘格检测节点 (测试手眼标定)")
    print("=" * 50)

    # 构建 pipeline
    print("🔧 构建 OAK RGB+Depth 流水线...")
    pipeline = dai.Pipeline()

    camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    camLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    camRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

    stereo = pipeline.create(dai.node.StereoDepth)
    camLeft.requestFullResolutionOutput().link(stereo.left)
    camRight.requestFullResolutionOutput().link(stereo.right)

    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setRectification(True)
    stereo.setExtendedDisparity(True)
    stereo.setSubpixel(True)

    try:
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    except AttributeError:
        print("ℹ️ 使用默认深度预设")

    q_depth = stereo.depth.createOutputQueue()
    q_rgb = camRgb.requestOutput((1280, 720), type=dai.ImgFrame.Type.BGR888i).createOutputQueue()

    print("🚀 启动相机...")
    pipeline.start()

    rclpy.init(args=args)
    node = OakChessboardPublisher()

    K = node.K
    dist = node.dist

    print("=" * 50)
    print(" 棋盘格检测中...")
    print(f" 棋盘: {CHECKERBOARD[0]}x{CHECKERBOARD[1]} 内角点, {SQUARE_SIZE_MM}mm/格")
    print(" 按 q 退出")
    print("=" * 50)

    if node.show_window:
        cv2.namedWindow('OAK Chessboard', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('OAK Chessboard', 1280, 720)

    try:
        while pipeline.isRunning():
            rclpy.spin_once(node, timeout_sec=0.0)

            # 阻塞式获取（与参考文件一致）
            inRgb = q_rgb.get()
            inDepth = q_depth.get()

            frame = inRgb.getCvFrame()
            depth_frame = inDepth.getFrame()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

            display = frame.copy()

            if ret:
                # 亚像素精化
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners_refined = cv2.cornerSubPix(
                    gray, corners,
                    winSize=(11, 11),
                    zeroZone=(-1, -1),
                    criteria=criteria
                )
                cv2.drawChessboardCorners(display, CHECKERBOARD, corners_refined, ret)

                # 计算 3D 中心
                center_cam_mm = compute_chessboard_3d_center(corners_refined, K, dist)
                if center_cam_mm is not None:
                    x, y, z = center_cam_mm
                    text = f"Center: X={x:.1f} Y={y:.1f} Z={z:.1f} mm"
                    cv2.putText(display, text, (20, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                    # 发布到 ROS2
                    node.publish_result(x, y, z)
                    print(f"🎯 Chessboard Center: X={x:.1f} Y={y:.1f} Z={z:.1f} mm")
            else:
                cv2.putText(display, "No chessboard detected", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            # 发布图像
            node.publish_images(frame, depth_frame, display)

            if node.show_window:
                cv2.imshow('OAK Chessboard', display)
                if cv2.waitKey(1) == ord('q'):
                    break

    except KeyboardInterrupt:
        print("\n[INFO] 用户中断")

    finally:
        if node.show_window:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
