# OAK4-D-Pro YOLO + 深度节点

当前节点默认使用 `/home/qluo/best.rvc4.tar.xz`（`yellow_peach` 分割归档）在 OAK4-D-Pro 的 RVC4 NPU 上推理，
本机只负责接收 RGB、检测框和对齐深度。深度话题是原始毫米值，不是伪彩色图，
可直接给 AprilTag 深度取点和 xArm6 手眼标定使用。

## 启动

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./start_arm_system.sh oak
```

无图形桌面时：

```bash
OAK_SHOW_WINDOW=false ./start_arm_system.sh oak
```

切换到 `/home/qluo/test.py` 使用的 17 点 Pose 模型：

```bash
OAK_MODEL_ARCHIVE=/home/qluo/yolov8l-pose.rvc4.tar.xz ./start_arm_system.sh oak
```

当前运行时需要 DepthAI `3.8.0` 和 `depthai-nodes 0.6.0`；两份归档都已安装到 ROS 包的
`share/oak_yolo_py/models/`。

## 帧率

默认请求 `60 FPS` 的相机和板载推理预览。实测黄桃模型约 `58.7 FPS`；完整 `1280x800`
RGB 和 `16UC1` 深度用于标定，分别以 30 FPS 和约 50 FPS 工作，避免大数据流拖慢实时窗口。

显示窗口包括 RGB+YOLO 和对齐深度；按空格保存 RGB、标注图、16 位深度 PNG 配对样本，
按 Q 退出。样本和工厂标定文件保存在 `~/oak_snapshots/`。

## ROS 2 输出

| 话题 | 类型 | 内容 |
|---|---|---|
| `/oak_result` | `std_msgs/String` | `label,confidence,x,y,z`，XYZ 单位 mm |
| `/oak/rgb/image_raw` | `sensor_msgs/Image` | `1280x800`, `bgr8` |
| `/oak/rgb/image_annotated` | `sensor_msgs/Image` | 板载 YOLO 结果叠框预览（模型输入尺寸） |
| `/oak/depth/image_raw` | `sensor_msgs/Image` | `1280x800`, `16UC1`，每像素 mm |
| `/oak/rgb/camera_info` | `sensor_msgs/CameraInfo` | RGB 内参和畸变 |
| `/oak/depth/camera_info` | `sensor_msgs/CameraInfo` | 与 RGB 对齐的深度内参 |
| `/oak_pose` | `std_msgs/String` | JSON 检测结果；Pose 模型包含关键点数组 |

读取深度时必须按 `16UC1` 处理；不要把它当成 `bgr8` 伪彩色图用于计算。

## 查看数据

```bash
ros2 topic echo /oak_result
ros2 topic hz /oak/rgb/image_raw
ros2 topic echo --once /oak/rgb/camera_info
```

`~/oak_snapshots/oak_factory_calibration.json` 包含三个相机的工厂内参、畸变和左右目外参，
可供后续 AprilTag 标定脚本读取。
