# Orange Detection Node - 使用说明

## 功能说明

这个ROS2节点使用OAK相机和YOLO11模型检测橙子（orange），并通过ROS2话题发布检测结果，供xArm6机械臂使用。

## 发布的话题

### 1. `/oak/orange_detection` (std_msgs/String)
完整的检测信息（JSON格式）：
```json
{
  "class": "orange",
  "confidence": 0.856,
  "bbox": [320, 240, 450, 380],
  "center_pixel": [385, 310],
  "distance_cm": 45.32,
  "distance_mm": 453,
  "timestamp": 1704182400
}
```

### 2. `/oak/orange_coordinates` (std_msgs/String)
简化的坐标信息（用于xArm6控制）：
```json
{
  "x_pixel": 385,
  "y_pixel": 310,
  "z_mm": 453,
  "confidence": 0.856
}
```

## 编译和运行

### 1. 编译包
```bash
cd ~/ros2_ws
colcon build --packages-select oak_yolo_py
source install/setup.bash
```

### 2. 运行节点
```bash
ros2 run oak_yolo_py oak_yolo_node
```

### 3. 查看话题
在另一个终端：
```bash
# 查看所有话题
ros2 topic list

# 监听橙子检测结果
ros2 topic echo /oak/orange_detection

# 监听坐标信息
ros2 topic echo /oak/orange_coordinates
```

## 参数配置

在 `oak_yolo_node.py` 中可以修改以下参数：

- `self.conf_thres = 0.5` - 置信度阈值（0-1）
- `self.target_class = 'orange'` - 目标检测类别
- `self.min_dist_cm = 30.0` - 最小检测距离（厘米）
- `self.max_dist_cm = 150.0` - 最大检测距离（厘米）

## 下一步：集成xArm6

创建一个订阅节点来接收橙子坐标并控制xArm6：

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class XArmControlNode(Node):
    def __init__(self):
        super().__init__('xarm_control_node')
        self.subscription = self.create_subscription(
            String,
            '/oak/orange_coordinates',
            self.orange_callback,
            10)
    
    def orange_callback(self, msg):
        data = json.loads(msg.data)
        x_pixel = data['x_pixel']
        y_pixel = data['y_pixel']
        z_mm = data['z_mm']
        confidence = data['confidence']
        
        self.get_logger().info(f'收到橙子坐标: ({x_pixel}, {y_pixel}, {z_mm}mm), 置信度: {confidence}')
        
        # TODO: 坐标转换和机械臂控制
        # 1. 像素坐标 -> 相机坐标
        # 2. 相机坐标 -> 机械臂基座坐标
        # 3. 调用xArm6 API移动到目标位置
```

## 故障排查

1. **找不到模型文件**
   - 确保 `yolo11m.rvc4.tar.xz` 在 `oak_yolo_py/oak_yolo_py/` 目录下

2. **相机连接失败**
   - 检查OAK相机USB连接
   - 运行 `depthai-python` 测试脚本验证相机

3. **没有检测结果**
   - 检查橙子是否在30-150cm范围内
   - 调整 `conf_thres` 降低置信度阈值
   - 确保光照条件良好
