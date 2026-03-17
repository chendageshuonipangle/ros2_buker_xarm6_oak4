import depthai as dai
import cv2
import numpy as np
from pathlib import Path
import sys
import collections
import time

# =========================
# ROS2 publish
# =========================
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

# ==========================================
# 1. 自动锁定 INT8 模型
#    兼容两种运行方式：
#    - 直接 python oak_yolo_node.py （使用源码目录）
#    - ros2 run oak_yolo_py oak_yolo_node （使用安装后的share目录）
# ==========================================
def _find_model_path() -> Path:
    candidates = []

    # 1) 脚本当前目录（源码运行时有效）
    try:
        current_dir = Path(__file__).parent.resolve()
        candidates.append(current_dir)
    except Exception:
        pass

    # 2) 包的 share 目录（通过 ament_index，在 ros2 run 时有效）
    try:
        share_dir = Path(get_package_share_directory("oak_yolo_py"))
        candidates.append(share_dir)
        candidates.append(share_dir / "models")
    except Exception:
        pass

    for base in candidates:
        if not base or not base.exists():
            continue
        archives = list(base.glob("*.tar.xz"))
        if archives:
            return archives[0]

    return None


model_path = _find_model_path()
if model_path is None:
    print("❌ 找不到 .tar.xz 模型包！请确认模型已放在源码目录或安装到 share/oak_yolo_py/models 下")
    sys.exit(1)

print(f"✅ 锁定模型: {model_path}")

# ==========================================
# 2. 核心算法：高级坐标平滑类 (防闪烁版)
# ==========================================
class SpatialSmoother:
    def __init__(self, history_size=10, max_missing_frames=30):
        self.history = collections.defaultdict(lambda: collections.deque(maxlen=history_size))
        self.last_seen = {}
        self.max_missing = max_missing_frames

    def update(self, object_id, coords):
        raw_x, raw_y, raw_z = coords

        current_time = time.time()
        to_remove = [k for k, t in self.last_seen.items() if current_time - t > 5.0]
        for k in to_remove:
            del self.history[k]
            del self.last_seen[k]

        if raw_z == 0:
            if object_id in self.history and len(self.history[object_id]) > 0:
                history = np.array(self.history[object_id])
                avg_coords = np.mean(history, axis=0)
                return int(avg_coords[0]), int(avg_coords[1]), int(avg_coords[2])
            return 0, 0, 0

        self.history[object_id].append(coords)
        self.last_seen[object_id] = current_time

        history = np.array(self.history[object_id])
        if len(history) >= 3:
            z_values = history[:, 2]
            avg_x = np.mean(history[:, 0])
            avg_y = np.mean(history[:, 1])
            avg_z = np.median(z_values)
        else:
            avg_coords = np.mean(history, axis=0)
            avg_x, avg_y, avg_z = avg_coords

        return int(avg_x), int(avg_y), int(avg_z)

smoother = SpatialSmoother(history_size=15)

def get_spatial_coords(depth_frame, bbox, padding=0.2):
    """
    输入: 深度图, 检测框(normalized)
    输出: X(mm), Y(mm), Z(mm)
    """
    if depth_frame is None:
        return 0, 0, 0

    h, w = depth_frame.shape[:2]
    xmin, ymin, xmax, ymax = bbox

    x1 = int(xmin * w); y1 = int(ymin * h)
    x2 = int(xmax * w); y2 = int(ymax * h)

    box_w = x2 - x1; box_h = y2 - y1
    pad_x = int(box_w * padding); pad_y = int(box_h * padding)

    roi_x1 = max(0, x1 + pad_x); roi_y1 = max(0, y1 + pad_y)
    roi_x2 = min(w, x2 - pad_x); roi_y2 = min(h, y2 - pad_y)

    if roi_x2 <= roi_x1 or roi_y2 <= roi_y1:
        return 0, 0, 0

    roi_depth = depth_frame[roi_y1:roi_y2, roi_x1:roi_x2]
    valid_pixels = roi_depth[(roi_depth > 0) & (roi_depth < 3000)]

    if valid_pixels.size == 0:
        return 0, 0, 0

    z_mm = int(np.median(valid_pixels))

    # 反投影（简化 pinhole）
    img_center_x = w / 2
    img_center_y = h / 2

    obj_center_x = x1 + (box_w / 2)
    obj_center_y = y1 + (box_h / 2)

    fx_approx = img_center_x / 0.687
    fy_approx = fx_approx

    x_mm = int((obj_center_x - img_center_x) * z_mm / fx_approx)
    y_mm = int((obj_center_y - img_center_y) * z_mm / fy_approx)

    return x_mm, y_mm, z_mm

# ==========================================
# ROS2 发布节点
# 话题:
#   /oak_result          - String: label,confidence,x,y,z
#   /oak/rgb/image_raw   - 原始 RGB 图像
#   /oak/depth/image_raw - 深度图像 (colormap)
#   /oak/rgb/image_annotated - 带检测框的 RGB 图像
# ==========================================
class OakResultPublisher(Node):
    def __init__(self):
        super().__init__('oak_result_publisher')
        
        # 声明参数
        self.declare_parameter('show_window', False)  # 是否显示 cv2 窗口
        self.declare_parameter('publish_rate', 30.0)  # 图像发布帧率限制
        
        self.show_window = self.get_parameter('show_window').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # 检测结果发布
        self.pub = self.create_publisher(String, '/oak_result', 10)
        
        # 图像话题发布
        self.pub_rgb = self.create_publisher(Image, '/oak/rgb/image_raw', 10)
        self.pub_depth = self.create_publisher(Image, '/oak/depth/image_raw', 10)
        self.pub_annotated = self.create_publisher(Image, '/oak/rgb/image_annotated', 10)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 帧率控制
        self.last_pub_time = time.time()
        self.min_interval = 1.0 / self.publish_rate
        
        self.get_logger().info(f'OAK Node 启动: show_window={self.show_window}, rate={self.publish_rate}Hz')
        self.get_logger().info('图像话题: /oak/rgb/image_raw, /oak/depth/image_raw, /oak/rgb/image_annotated')
    
    def should_publish(self):
        """帧率限制检查"""
        now = time.time()
        if now - self.last_pub_time >= self.min_interval:
            self.last_pub_time = now
            return True
        return False
    
    def publish_images(self, rgb_frame, depth_frame, annotated_frame):
        """发布图像到 ROS2 话题"""
        if not self.should_publish():
            return
        
        stamp = self.get_clock().now().to_msg()
        
        # 发布原始 RGB
        if rgb_frame is not None:
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_frame, encoding='bgr8')
            rgb_msg.header.stamp = stamp
            rgb_msg.header.frame_id = 'oak_rgb_camera_optical_frame'
            self.pub_rgb.publish(rgb_msg)
        
        # 发布深度图像 (colormap 可视化)
        if depth_frame is not None and rgb_frame is not None:
            # 深度帧分辨率可能与RGB不同，需要resize
            rgb_h, rgb_w = rgb_frame.shape[:2]
            depth_resized = cv2.resize(depth_frame, (rgb_w, rgb_h), interpolation=cv2.INTER_NEAREST)
            
            # 动态计算深度范围 (使用实际数据的有效范围)
            valid_depth = depth_resized[depth_resized > 0]
            if valid_depth.size > 0:
                max_depth = min(np.percentile(valid_depth, 99), 5000)  # 99分位或5m
            else:
                max_depth = 3000
            
            # 归一化深度图用于可视化
            depth_vis = np.clip(depth_resized, 0, max_depth)
            depth_vis = (depth_vis / max_depth * 255).astype(np.uint8)
            depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
            
            depth_msg = self.bridge.cv2_to_imgmsg(depth_color, encoding='bgr8')
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = 'oak_rgb_camera_optical_frame'
            self.pub_depth.publish(depth_msg)
        
        # 发布带标注的图像
        if annotated_frame is not None:
            ann_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            ann_msg.header.stamp = stamp
            ann_msg.header.frame_id = 'oak_rgb_camera_optical_frame'
            self.pub_annotated.publish(ann_msg)

# ==========================================
# 3. 构建 Pipeline
# ==========================================
pipeline = dai.Pipeline()

print("🔧 初始化传感器...")
camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
camLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
camRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

print("🔧 加载 INT8 模型...")
try:
    archive = dai.NNArchive(str(model_path))
    nn = pipeline.create(dai.node.DetectionNetwork).build(camRgb, archive)
    nn.setConfidenceThreshold(0.5)
except Exception as e:
    print(f"❌ 加载失败: {e}")
    sys.exit(1)

print("🔧 初始化深度引擎...")
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
    print("ℹ️ 使用默认深度预设 (PresetMode API 变动已忽略)")
    pass

q_nn = nn.out.createOutputQueue()
q_depth = stereo.depth.createOutputQueue()
q_rgb = camRgb.requestOutput((1280, 720), type=dai.ImgFrame.Type.BGR888i).createOutputQueue()

# ==========================================
# 4. 运行
# ==========================================
print("🚀 引擎启动！(带防闪烁坐标平滑 + ROS2 发布 /oak_result)")

rclpy.init(args=None)
ros_node = OakResultPublisher()

pipeline.start()

labelMap = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed", "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"]

try:
    while pipeline.isRunning():
        # 严格参考 chessboard_xarm_calib_collect.py: 使用阻塞式 get() 确保同步
        inRgb = q_rgb.get()
        inDepth = q_depth.get()
        inDet = q_nn.tryGet()  # 检测结果可以是非阻塞的

        frame = inRgb.getCvFrame()
        depth_frame = inDepth.getFrame()
        rgb_raw = frame.copy()  # 保存原始 RGB 用于发布

        if inDet is not None and isinstance(inDet, dai.ImgDetections):
            for det in inDet.detections:
                h, w = frame.shape[:2]
                x1 = int(det.xmin * w); y1 = int(det.ymin * h)
                x2 = int(det.xmax * w); y2 = int(det.ymax * h)

                label_idx = det.label
                label = labelMap[label_idx] if label_idx < len(labelMap) else str(label_idx)

                bbox_norm = (det.xmin, det.ymin, det.xmax, det.ymax)
                raw_x, raw_y, raw_z = get_spatial_coords(depth_frame, bbox_norm)

                smooth_x, smooth_y, smooth_z = smoother.update(label, (raw_x, raw_y, raw_z))

                # ✅ ROS2 发布：每个目标一条消息
                # 格式: label,confidence,x,y,z   (mm)
                msg = String()
                msg.data = f"{label},{float(det.confidence):.4f},{smooth_x},{smooth_y},{smooth_z}"
                ros_node.pub.publish(msg)

                # ---- 视觉标注 ----
                color = (0, 255, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

                dist_cm = smooth_z / 10.0

                if smooth_z > 0:
                    coord_text = f"X:{smooth_x} Y:{smooth_y} Z:{smooth_z}"
                    label_text = f"{label} {dist_cm:.1f}cm"
                    print(f"🎯 {label}: Raw Z={raw_z}mm -> Stable Z={smooth_z}mm")
                else:
                    coord_text = "Stabilizing..."
                    label_text = label

                (tw, th), _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                cv2.rectangle(frame, (x1, y1 - 20), (x1 + tw + 10, y1), color, -1)

                cv2.putText(frame, label_text, (x1 + 5, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

                cv2.putText(frame, coord_text, (x1, y2 + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # ✅ 发布图像到 ROS2 话题
        ros_node.publish_images(rgb_raw, depth_frame, frame)

        # 让 ROS2 内部处理（非阻塞）
        rclpy.spin_once(ros_node, timeout_sec=0.0)

        # 可选: 显示 cv2 窗口
        if ros_node.show_window:
            cv2.imshow("OAK 4 - Smoothed Spatial", frame)
            if cv2.waitKey(1) == ord('q'):
                break

finally:
    if ros_node.show_window:
        cv2.destroyAllWindows()
    ros_node.destroy_node()
    rclpy.shutdown()
