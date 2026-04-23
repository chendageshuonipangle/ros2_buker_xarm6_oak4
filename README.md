# ROS2 Bunker Mini 导航 + xArm6 视觉抓取系统

---

> ## ⚠️ 严重警告
>
> **机械臂控制与移动底盘导航绝对不可融为一体运行。**
>
> 两套系统在硬件资源、实时性要求、安全边界上存在根本性冲突：
> - 导航系统占用底盘全部运动控制权，机械臂同步运动将导致重心失稳、碰撞或硬件损毁
> - 两套 MoveIt2 / Nav2 实例共享 TF 树时存在不可预测的坐标系竞争
> - 任何"联合运动"尝试均可能造成人员伤害或设备永久损坏
>
> **本项目中"联合启动"仅指同时上电与话题可见，绝非允许同步运动。**
> **禁止在导航行进过程中操作机械臂，禁止在机械臂运动过程中发送导航目标。**
> **此警告没有例外，没有商量余地。**

---

本项目集成了三个子系统：
1. **Bunker Mini 移动机器人**：基于 RPLIDAR A1 + Nav2 的自主导航，支持 SLAM 建图与自定义路径规划算法
2. **xArm6 机械臂视觉抓取**：基于 OAK 相机 YOLO 目标检测 + MoveIt2 的全自动抓取
3. **联合启动**：小车与机械臂共享统一 TF 树，可同时运行

---

## 环境要求

- Ubuntu 24.04
- ROS2 Jazzy
- Python 3.12+
- 硬件：Bunker Mini（CAN 总线连接）、RPLIDAR A1、xArm6、OAK-D 相机

---

## 安装与编译

### 1. 克隆仓库

```bash
git clone <your-repo-url> ~/ros2_buker_xarm6_oak4
cd ~/ros2_buker_xarm6_oak4
```

### 2. 初始化 xArm SDK 子模块

```bash
cd src/xarm_ros2
git submodule update --init --recursive
# 若子模块 remote 指向错误，手动修复：
cd xarm_sdk/cxx
git remote set-url origin https://github.com/xArm-Developer/xArm-CPLUS-SDK.git
git fetch origin && git checkout master
cd ../../../..
```

### 3. 安装系统依赖

```bash
sudo apt-get install -y \
    libasio-dev \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-moveit
```

### 4. 配置环境变量

在 `~/.bashrc` 中添加：

```bash
export LINOROBOT2_BASE=bunker_mini
export LINOROBOT2_LASER_SENSOR=rplidar
```

```bash
source ~/.bashrc
```

### 5. 编译

```bash
cd ~/ros2_buker_xarm6_oak4
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

---

## 子系统三：小车 + 机械臂联合启动

### TF 树结构

项目通过统一的 `bunker_with_xarm6.urdf.xacro` 合并了小车和机械臂的 TF 树，只启动一个 `robot_state_publisher`，避免 `base_link` 冲突：

```
odom
  └── base_link              ← bunker_base_node 发布里程计
        ├── left_wheel_link
        ├── right_wheel_link
        ├── laser
        ├── imu_link
        └── xarm6_link0      ← fixed joint，挂载点 (0, 0, 0.16)
              └── link1 → link2 → link3 → link4 → link5 → link6
```

### 仿真模式（WSL2 / 无硬件）

```bash
source ~/ros2_buker_xarm6_oak4/install/setup.bash

# 一键启动小车 + xArm6 fake 控制器 + MoveIt2
ros2 launch bunker_xarm6_description combined_bringup.launch.py
```

可选参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `port_name` | `can0` | CAN 总线端口 |
| `is_bunker_mini` | `true` | 是否为 Bunker Mini |
| `rviz` | `false` | 是否启动 RViz |
| `add_gripper` | `false` | 是否添加夹爪 |

```bash
# 示例：启动并打开 RViz，带夹爪
ros2 launch bunker_xarm6_description combined_bringup.launch.py \
    rviz:=true add_gripper:=true
```

### 实机模式

```bash
# 终端 1：联合启动（小车底盘 + xArm6 MoveIt2）
ros2 launch bunker_xarm6_description combined_bringup.launch.py \
    port_name:=can0 is_bunker_mini:=true

# 终端 2：导航
./start_bunker_navigation.sh slam_nav

# 终端 3：视觉抓取
ros2 run oak_yolo_py oak_yolo_node
ros2 launch oaktf_trantoarm transform.launch.py target_label:=orange
ros2 run armtodeprition motion_planner_node
```

---

## 子系统一：Bunker Mini 导航

### 系统配置

| 项目 | 参数 |
|------|------|
| 底盘 | Bunker Mini (2WD) |
| 激光雷达 | RPLIDAR A1 |
| CAN 端口 | can0 |
| 机器人尺寸 | 620 × 520 × 200 mm |
| 轮距 | 460 mm |
| 车轮直径 | 165 mm |

### 快速启动

```bash
cd ~/ros2_buker_xarm6_oak4
./start_bunker_navigation.sh bringup
```

脚本会自动检测 CAN 接口状态并询问是否初始化。

> 若 RPLIDAR 启动超时，按 `Ctrl+C` 后重新运行（设备可能被上一进程占用）。

### 运行模式

#### 模式 1：先建图后导航

```bash
# 终端 1：启动硬件
./start_bunker_navigation.sh bringup

# 终端 2：SLAM 建图
./start_bunker_navigation.sh slam

# 建图完成后保存地图
./start_bunker_navigation.sh save_map

# 终端 2：切换为导航
./start_bunker_navigation.sh nav
```

#### 模式 2：边建图边导航

```bash
# 终端 1：启动硬件
./start_bunker_navigation.sh bringup

# 终端 2：SLAM + 导航
./start_bunker_navigation.sh slam_nav
```

#### 模式对比

| 特性 | 模式 1（传统） | 模式 2（SLAM Nav） |
|------|--------------|------------------|
| 定位节点 | AMCL | SLAM Toolbox |
| 地图来源 | 静态地图文件 | 实时生成 |
| 适用场景 | 已知环境 | 未知/动态环境 |
| 计算负载 | 低 | 高 |

### 自定义路径规划

系统支持用自定义 Python 算法替代 Nav2 默认规划器，通过 RViz 的 **2D Goal Pose** 触发。

```bash
# 终端 1
./start_bunker_navigation.sh bringup

# 终端 2
./start_bunker_navigation.sh slam_nav

# 终端 3：选择一个算法
./start_bunker_navigation.sh jps           # JPS 原始版（最快）
./start_bunker_navigation.sh jps_improved  # JPS 改进版（障碍物密集区更保守）
./start_bunker_navigation.sh astar_cost    # A* 代价感知（推荐，路径远离障碍物）
```

| 算法 | 考虑代价 | 速度 | 路径质量 |
|------|---------|------|---------|
| JPS | ❌ | 快 | 最短路径 |
| JPS Improved | ❌ | 快 | 略优于 JPS |
| A* Costmap | ✅ | 中 | 安全路径 |
| Nav2 Smac (默认) | ✅ | 快 | 安全路径 |

> **RViz 按钮区别**：`Nav2 Goal` 使用 Nav2 内置规划器；`2D Goal Pose` 使用上述自定义算法（需先启动对应节点）。

### 键盘遥控

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

`i` 前进 / `,` 后退 / `j` 左转 / `l` 右转 / `k` 停止

---

## 子系统二：xArm6 视觉抓取

### 系统架构

```
OAK 相机 (YOLO 检测)
    → /oak_result (label, conf, x, y, z 单位 mm)
        → oaktf_trantoarm (手眼标定坐标转换)
            → /target_pose_in_base (基座坐标系, 单位 m)
                → armtodeprition (MoveIt2 运动规划 + 夹爪控制)
```

### 启动步骤（YOLO 目标检测抓取）

```bash
# 终端 1：启动 xArm6 MoveIt2
ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py \
    robot_ip:=192.168.1.242 add_gripper:=true

# 终端 2：启动 OAK YOLO 检测
ros2 run oak_yolo_py oak_yolo_node

# 终端 3：启动坐标转换
ros2 launch oaktf_trantoarm transform.launch.py target_label:=orange

# 终端 4：启动运动规划（全自动模式）
ros2 run armtodeprition motion_planner_node
```

检测到目标后，机械臂自动执行抓取序列（间隔 3 秒防重复触发）。

如需手动模式：

```bash
ros2 run armtodeprition motion_planner_node --ros-args -p auto_execute:=false
# 手动触发
ros2 service call /execute_grasp std_srvs/srv/Trigger
```

### 启动步骤（棋盘格标定测试）

```bash
# 终端 1：xArm6 MoveIt2（同上）

# 终端 2：棋盘格检测
ros2 run oak_yolo_py oak_chessboard_node

# 终端 3：坐标转换
ros2 run oaktf_trantoarm transform_node --ros-args -p target_label:=chessboard

# 终端 4：运动规划
ros2 run armtodeprition motion_planner_node
```

### 话题与服务

| 名称 | 类型 | 说明 |
|------|------|------|
| `/oak_result` | std_msgs/String | 检测结果 (label,conf,x,y,z) |
| `/oak/rgb/image_raw` | sensor_msgs/Image | 原始 RGB 图像 |
| `/oak/rgb/image_annotated` | sensor_msgs/Image | 带检测框图像 |
| `/target_pose_in_base` | geometry_msgs/PoseStamped | 基座坐标系目标位姿 |
| `/execute_grasp` (服务) | std_srvs/Trigger | 执行完整抓取序列 |
| `/gripper_open` (服务) | std_srvs/Trigger | 打开夹爪 |
| `/gripper_close` (服务) | std_srvs/Trigger | 关闭夹爪 |

### 工作范围限制

| 方向 | 范围 |
|------|------|
| 水平距离 | 0.15 m ~ 0.65 m |
| Z 高度 | 0.05 m ~ 0.80 m |

### 更新手眼标定

```bash
# 1. 采集标定数据
python3 chessboard_xarm_calib_collect.py \
    --calib oak_camera_calib.yaml --base-frame link_base --tcp-frame link_tcp

# 2. 计算标定结果
python3 solve_oak_xarm_handeye.py --npz chessboard_xarm_calib_points.npz

# 3. 将输出的 R 和 t 更新到 oaktf_trantoarm/launch/transform.launch.py
```

---

## 调参指南

### 导航避障

| 问题 | 解决方法 |
|------|---------|
| 机器人撞障碍物 | 增大 `inflation_radius`，降低 `cost_scaling_factor`，改用 `astar_cost` |
| 机器人无法移动（全是障碍） | 减小 `inflation_radius`，增大 `cost_scaling_factor` |
| 自定义算法路径穿障碍物 | 改用 `astar_cost`，增大 `cost_weight` 参数 |
| SLAM 建图效果差 | 调整 `slam.yaml` 中 `minimum_travel_distance` 和 `map_update_interval` |

---

## 故障排查

### CAN 未连接

```bash
# 手动初始化
cd ~/ros2_buker_xarm6_oak4/src/ugv_sdk/scripts/
bash bringup_can2usb_500k.bash

# 验证
candump can0
```

### RPLIDAR 未检测到

```bash
ls -l /dev/rplidar  # 若无设备，重新插拔 USB
```

### 常用诊断命令

```bash
ros2 topic echo /bunker_status --once   # 底盘状态（control_mode 应为 1）
ros2 topic hz /scan                     # 激光雷达频率
ros2 topic hz /odom                     # 里程计频率
ros2 topic list                         # 所有话题
ros2 run tf2_tools view_frames          # TF 树
```

---

## 关键配置文件

| 文件 | 说明 |
|------|------|
| `linorobot2_base/config/ekf.yaml` | EKF 传感器融合参数 |
| `linorobot2_bringup/config/laser_filter.yaml` | 雷达过滤（前方 200°）|
| `linorobot2_navigation/config/navigation.yaml` | Nav2 参数 |
| `linorobot2_navigation/config/slam.yaml` | SLAM Toolbox 参数 |
| `path_planning/path_planning/algorithms/astar_costmap.py` | A* 代价感知算法 |
| `oaktf_trantoarm/launch/transform.launch.py` | 手眼标定矩阵 |
| `armtodeprition/armtodeprition/motion_planner_node.py` | 运动规划与障碍物配置 |

---

## 文件结构

```
ros2_buker_xarm6_oak4/
├── start_bunker_navigation.sh
├── maps/
└── src/
    ├── bunker_ros2/            # Bunker Mini 底盘驱动
    ├── ugv_sdk/                # UGV SDK
    ├── linorobot2/             # 导航框架
    ├── sllidar_ros2/           # RPLIDAR 驱动
    ├── path_planning/          # 自定义路径规划
    │   └── path_planning/algorithms/
    │       ├── astar_costmap.py
    │       ├── jps.py
    │       └── jps_improved.py
    ├── oak_yolo_py/            # OAK 相机 YOLO 检测
    ├── oaktf_trantoarm/        # 手眼标定坐标转换
    ├── armtodeprition/         # MoveIt2 运动规划
    └── xarm_ros2/              # xArm SDK
```

---

## 参考资料

- [Nav2 文档](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [linorobot2](https://github.com/linorobot/linorobot2)
- [xArm ROS2](https://github.com/xArm-Developer/xarm_ros2)
