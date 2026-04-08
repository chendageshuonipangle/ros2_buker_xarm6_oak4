⚠️⚠️⚠️请谨慎使用该项目，纯ai来的，只是为了满足项目功能的推进，只在实验室跑起来ok⚠️⚠️⚠️

#单独编译安装时请先移除buker_ros2和ugv_sdk和xarm6_ros2的包以免编译报错然后红温，因为这两个相对需要的依赖和内容复杂
#请使用完成对依赖的更新，以确保上述包的正常编译
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
# Bunker Mini 导航系统完整指南

Bunker Mini 机器人 + RPLIDAR A1 激光雷达 + Linorobot2 导航框架

---

## 快速启动

### 最简单的启动方式

```bash
# 启动机器人硬件（会自动询问是否初始化 CAN）
cd ~/ros2_ws
./start_bunker_navigation.sh bringup
```

**⚠️ 重要提示：**
- 脚本会自动检查并询问是否初始化 CAN 接口
- 如果 RPLIDAR 启动失败（超时错误），按 `Ctrl+C` 停止后重新运行
- 原因：设备可能被之前的进程占用

**系统会自动启动：**
- ✓ Bunker Mini 底盘驱动
- ✓ RPLIDAR A1 激光雷达
- ✓ EKF 里程计融合
- ✓ 机器人状态发布
- ✓ Bunker Mini 3D 模型（RViz2 可视化）

---

## 系统配置

- **机器人**: Bunker Mini (2WD)
- **激光雷达**: RPLIDAR A1
- **CAN 端口**: can0
- **机器人尺寸**: 620mm × 520mm × 200mm
- **轮距**: 460mm
- **车轮直径**: 165mm
- **环境变量**: 
  - `LINOROBOT2_BASE=bunker_mini`
  - `LINOROBOT2_LASER_SENSOR=rplidar`

---

## 三种运行模式

### 模式 1: 先建图后导航（传统模式）

```bash
# 终端 1: 启动硬件（会自动询问是否初始化 CAN）
./start_bunker_navigation.sh bringup

# 终端 2: 启动 SLAM 建图
./start_bunker_navigation.sh slam

# 遥控机器人建图完成后，保存地图
./start_bunker_navigation.sh save_map

# 终端 2: 启动导航（使用保存的地图）
./start_bunker_navigation.sh nav
```

**定位方式**: AMCL（粒子滤波）  
**地图来源**: 预先保存的静态地图

### 模式 2: 边建图边导航（SLAM Navigation）

```bash
# 终端 1: 启动硬件（会自动询问是否初始化 CAN）
./start_bunker_navigation.sh bringup

# 终端 2: 启动 SLAM + 导航
./start_bunker_navigation.sh slam_nav
```

**定位方式**: SLAM Toolbox（实时定位）  
**地图来源**: SLAM Toolbox 实时生成的动态地图

### 模式对比

| 特性 | 传统模式 | SLAM Navigation |
|------|---------|-----------------|
| 定位节点 | AMCL | SLAM Toolbox |
| 地图来源 | Map Server (静态) | SLAM Toolbox (动态) |
| 适用场景 | 已知环境 | 未知/变化环境 |
| 地图更新 | 不更新 | 实时更新 |
| 计算负载 | 较低 | 较高 |

---

## 启动脚本使用

```bash
./start_bunker_navigation.sh <模式> [地图路径]

模式:
  bringup      - 只启动机器人硬件 (底盘、雷达、TF)
  slam         - 只启动 SLAM 建图
  nav          - 只启动导航 (需要已有地图)
  slam_nav     - SLAM + 导航同时运行 (边建图边导航)
  jps          - 启动 JPS 原始版路径规划节点
  jps_improved - 启动 JPS 改进版路径规划节点 (带密度启发)
  astar_cost   - 启动 A* 代价感知路径规划节点 (远离障碍物)
  save_map     - 保存当前地图

示例:
  ./start_bunker_navigation.sh bringup
  ./start_bunker_navigation.sh slam
  ./start_bunker_navigation.sh nav /home/pc-24/ros2_ws/maps/my_map.yaml
  ./start_bunker_navigation.sh slam_nav
  ./start_bunker_navigation.sh jps
  ./start_bunker_navigation.sh jps_improved
  ./start_bunker_navigation.sh astar_cost
  ./start_bunker_navigation.sh save_map
```

---

## 自定义路径规划算法

### 概述

系统支持使用自定义 Python 路径规划算法替代 Nav2 默认的 Smac Planner。通过监听 RViz 的 "2D Goal Pose" 按钮，使用自定义算法规划路径，然后调用 Nav2 的 FollowPath 执行。

### 支持的算法

| 算法 | 启动命令 | 说明 | 文件位置 |
|------|---------|------|---------|
| JPS 原始版 | `./start_bunker_navigation.sh jps` | Jump Point Search，快速但不考虑代价 | `src/path_planning/path_planning/algorithms/jps.py` |
| JPS 改进版 | `./start_bunker_navigation.sh jps_improved` | 带密度启发的 JPS | `src/path_planning/path_planning/algorithms/jps_improved.py` |
| A* 代价感知 | `./start_bunker_navigation.sh astar_cost` | 考虑代价地图，路径远离障碍物 | `src/path_planning/path_planning/algorithms/astar_costmap.py` |
| A* 标准版 | 代码中可选 | 标准 A* 算法 | `src/path_planning/path_planning/algorithms/astar.py` |

### 使用方法

```bash
# 终端 1: 启动机器人硬件（会自动询问是否初始化 CAN）
./start_bunker_navigation.sh bringup

# 终端 2: 启动 SLAM + 导航
./start_bunker_navigation.sh slam_nav

# 终端 3: 启动自定义路径规划节点（选择一个）
./start_bunker_navigation.sh jps           # JPS 原始版
./start_bunker_navigation.sh jps_improved  # JPS 改进版
./start_bunker_navigation.sh astar_cost    # A* 代价感知（推荐）

# 在 RViz 中使用 "2D Goal Pose" 按钮设置目标点
```

### 算法对比

| 特性 | JPS | JPS Improved | A* Costmap | Nav2 Smac Planner |
|------|-----|--------------|------------|-------------------|
| 语言 | Python | Python | Python | C++ |
| 考虑代价 | ❌ | ❌ | ✅ | ✅ |
| 远离障碍物 | ❌ | ❌ | ✅ | ✅ |
| 速度 | 快 | 快 | 中等 | 快 |
| 路径质量 | 最短路径 | 最短路径 | 安全路径 | 安全路径 |
| 触发方式 | 2D Goal Pose | 2D Goal Pose | 2D Goal Pose | Nav2 Goal |

### RViz 按钮区别

| 按钮 | 使用的算法 | 说明 |
|------|-----------|------|
| **Nav2 Goal** | Nav2 Smac Planner 2D (C++) | Nav2 内置算法，考虑代价地图 |
| **2D Goal Pose** | 自定义 Python 算法 | 需要先启动 jps/jps_improved/astar_cost 节点 |

### 工作原理

```
┌─────────────────────────────────────────────────────────────────┐
│                    自定义路径规划流程                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  RViz "2D Goal Pose" ──→ /goal_pose 话题                       │
│                              │                                  │
│                              ▼                                  │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ path_planning/jps_planner_node                           │   │
│  │  1. 订阅 /global_costmap/costmap (代价地图)              │   │
│  │  2. 从 TF 获取机器人位置                                  │   │
│  │  3. 调用 JPS/A* 算法规划路径                             │   │
│  │  4. 路径插值（每 10cm 一个点）                           │   │
│  │  5. 发布路径到 /plan 和 /custom_path                     │   │
│  │  6. 调用 Nav2 FollowPath action 执行路径                 │   │
│  └─────────────────────────────────────────────────────────┘   │
│                              │                                  │
│                              ▼                                  │
│  Nav2 Controller Server (Regulated Pure Pursuit) ──→ /cmd_vel  │
│                              │                                  │
│                              ▼                                  │
│                        Bunker Base                              │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 算法文件说明

#### 1. A* 代价感知算法 (`src/path_planning/path_planning/algorithms/astar_costmap.py`)

```python
# 核心参数
obstacle_threshold = 253  # 代价 >= 253 为障碍物
cost_weight = 0.05        # 代价权重，越大越远离障碍物

# 代价计算
移动代价 = 基础代价 + cost_weight × 代价地图值
```

- 考虑代价地图的值（0-252）
- 高代价区域会增加路径代价
- 路径会自动远离障碍物膨胀区域

#### 2. JPS 算法 (`src/path_planning/path_planning/algorithms/jps.py`)

- Jump Point Search，跳点搜索
- 只考虑可通行/不可通行
- 速度快，但可能擦边走

#### 3. JPS 改进版 (`src/path_planning/path_planning/algorithms/jps_improved.py`)

- 带密度启发的 JPS
- 在障碍物密集区域更保守
- 速度快，路径质量略好于原始 JPS

### 可视化

规划的路径会发布到以下话题，可在 RViz 中添加 Path 显示：
- `/plan` - 规划的路径
- `/custom_path` - 同上，用于可视化

---

## SLAM Navigation 架构

当使用 `slam_nav` 模式时，系统架构如下：

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     bringup.launch.py (硬件层)                          │
├─────────────────────────────────────────────────────────────────────────┤
│  RPLIDAR A1 → /scan → laser_filter → /scan_filtered                    │
│  Bunker Base → odom/unfiltered → EKF → /odom                           │
│  URDF → robot_state_publisher → TF                                     │
└─────────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────────┐
│                 slam_navigation.launch.py (SLAM + 导航)                 │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌────────────────────────────────────────────────────────────────┐    │
│  │ SLAM Toolbox (替代 AMCL + Map Server)                          │    │
│  │  - 输入: /scan_filtered, /odom                                 │    │
│  │  - 输出: /map (动态更新), TF: map→odom                         │    │
│  └────────────────────────────────────────────────────────────────┘    │
│                              ↓                                          │
│  ┌────────────────────────────────────────────────────────────────┐    │
│  │ Nav2 导航栈                                                     │    │
│  │  - Global Costmap ← /map (来自 SLAM Toolbox)                   │    │
│  │  - Local Costmap ← /scan_filtered                              │    │
│  │  - Planner Server (Smac Planner 2D)                            │    │
│  │  - Controller Server (Regulated Pure Pursuit)                   │    │
│  │  - BT Navigator                                                 │    │
│  └────────────────────────────────────────────────────────────────┘    │
│                              ↓                                          │
│                         /cmd_vel → Bunker Base                         │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 可视化监控

### 测试 Bunker Mini 3D 模型
```bash
# 查看 URDF 是否正确生成
source ~/.bashrc
source ~/ros2_ws/install/setup.bash
ros2 launch linorobot2_description description.launch.py rviz:=true
```

这将启动 RViz2 并显示 Bunker Mini 的 3D 模型，包括：
- 底盘（620×520×200mm）
- 左右驱动轮（直径165mm）
- RPLIDAR A1 安装位置（前方中心，抬高）

### 启动 RViz2 查看实时数据
```bash
# 终端 1：启动机器人
./start_bunker_navigation.sh bringup

# 终端 2：启动 RViz2
rviz2
```

**在 RViz2 中添加显示：**
- `RobotModel` - 查看 Bunker Mini 3D 模型
- `LaserScan` (话题: `/scan`) - 查看激光雷达数据
- `TF` - 查看坐标系变换
- `Odometry` (话题: `/odom`) - 查看里程计轨迹

---

## 控制机器人

### 键盘控制
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**控制键：** `i`前进 `,`后退 `j`左转 `l`右转 `k`停止

---

## 关键配置文件

| 文件 | 作用 |
|------|------|
| `linorobot2_base/config/ekf.yaml` | EKF 传感器融合 |
| `linorobot2_bringup/config/laser_filter.yaml` | 雷达过滤 (前方 200°) |
| `linorobot2_navigation/config/navigation.yaml` | Nav2 参数 |
| `linorobot2_navigation/config/slam.yaml` | SLAM Toolbox 参数 |
| `linorobot2_description/urdf/robots/bunker_mini.urdf.xacro` | 机器人模型 |
| `path_planning/path_planning/jps_planner_node.py` | 自定义路径规划节点 |
| `path_planning/path_planning/algorithms/astar_costmap.py` | A* 代价感知算法 |
| `path_planning/path_planning/algorithms/jps_improved.py` | JPS 改进版算法 |

---

## 调参指南

### 如果机器人撞障碍物
1. 增大 `inflation_radius`（当前 60cm）
2. 降低 `cost_scaling_factor`（当前 1.5）
3. 增大 `cost_penalty`（当前 10.0）
4. 使用 `astar_cost` 算法（考虑代价）

### 如果机器人走不了（全是障碍）
1. 减小 `inflation_radius`
2. 增大 `cost_scaling_factor`

### 如果自定义算法路径穿过障碍物
1. 使用 `astar_cost` 而不是 `jps`
2. 调整 `astar_costmap.py` 中的 `cost_weight` 参数（增大会更远离障碍物）

### 如果 SLAM 建图效果差
1. 调整 `slam.yaml` 中的 `minimum_travel_distance`
2. 调整 `map_update_interval`
3. 确保雷达数据质量良好

---

## 常用命令

```bash
# 初始化 CAN 总线（现已集成到 bringup 模式，会自动询问）
# 手动初始化：cd ~/ros2_ws/src/ugv_sdk/scripts/ && bash bringup_can2usb_500k.bash

# 检查 CAN 连接
candump can0

# 检查 RPLIDAR 设备
ls -l /dev/rplidar

# 检查底盘状态 (control_mode 应为 1)
ros2 topic echo /bunker_status --once

# 查看激光数据
ros2 topic echo /scan --once

# 查看里程计
ros2 topic echo /odom --once

# 查看所有话题
ros2 topic list

# 查看话题频率
ros2 topic hz /scan
ros2 topic hz /odom

# 键盘遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 查看 TF 树
ros2 run tf2_tools view_frames

# 保存地图
./start_bunker_navigation.sh save_map
```

---

## 故障排查

### CAN 未连接
```bash
# 方法 1：使用启动脚本（推荐）
./start_bunker_navigation.sh bringup  # 会自动询问

# 方法 2：手动初始化
cd ~/ros2_ws/src/ugv_sdk/scripts/
bash bringup_can2usb_500k.bash
```

### RPLIDAR 未检测到
```bash
ls -l /dev/rplidar  # 检查设备
# 如果没有，重新插拔 USB
```

### 没有激光数据
```bash
ros2 topic hz /scan  # 检查话题频率
# 如果超时，按 Ctrl+C 停止后重新启动
```

---

## 首次安装（已完成）

如需重新安装 RPLIDAR 驱动：

```bash
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
sudo cp sllidar_ros2/scripts/rplidar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
cd ~/ros2_ws
colcon build --packages-select sllidar_ros2
source install/setup.bash
```

配置环境变量（添加到 `~/.bashrc`）：
```bash
export LINOROBOT2_BASE=bunker_mini
export LINOROBOT2_LASER_SENSOR=rplidar
```

---

## 文件结构

```
~/ros2_ws/
├── start_bunker_navigation.sh           # 启动脚本
├── Bunker_Mini_Navigation_Guide.md      # 本文件
├── maps/                                 # 保存的地图
└── src/
    ├── bunker_ros2/                     # Bunker Mini 驱动
    ├── linorobot2/                      # 导航框架
    ├── sllidar_ros2/                    # RPLIDAR 驱动
    └── path_planning/                   # 自定义路径规划算法包
        ├── package.xml
        ├── setup.py
        └── path_planning/
            ├── jps_planner_node.py
            ├── bunker_path_planner.py
            └── algorithms/
                ├── astar_costmap.py
                ├── jps.py
                └── jps_improved.py
```

---

## 参考资料

- [Nav2 官方文档](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [linorobot2](https://github.com/linorobot/linorobot2)

---

# xArm6 + OAK 视觉抓取系统

## 系统架构

```
┌─────────────────────┐
│  oak_yolo_py        │  OAK 相机目标检测 (depthai 3.0)
│  /oak_result        │  输出: label,conf,x,y,z (相机坐标, mm)
└─────────┬───────────┘
          │
          ▼
┌─────────────────────┐
│  oaktf_trantoarm    │  坐标转换 (相机→基座)
│  /target_pose_in_base│ 输出: PoseStamped (基座坐标, m)
└─────────┬───────────┘
          │
          ▼
┌─────────────────────┐
│  armtodeprition     │  MoveIt2 运动规划 + 夹爪控制
│  /execute_grasp     │  服务: 执行抓取序列
└─────────────────────┘
```

---

## 启动步骤 (棋盘格测试)

用于测试手眼标定和坐标转换是否正确。

### 终端 1: 启动 xArm6 MoveIt2 (真机)
```bash
ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py \
    robot_ip:=192.168.1.242 \
    add_gripper:=true
```

### 终端 2: 启动 OAK 棋盘格检测
```bash
source ~/ros2_ws/install/setup.bash
ros2 run oak_yolo_py oak_chessboard_node
```

### 终端 3: 启动坐标转换节点
```bash
source ~/ros2_ws/install/setup.bash
ros2 run oaktf_trantoarm transform_node --ros-args -p target_label:=chessboard
```

### 终端 4: 启动运动规划节点 (全自动模式)
```bash
source ~/ros2_ws/install/setup.bash
ros2 run armtodeprition motion_planner_node
```

> **注意**: 默认为全自动模式，收到目标位姿后会自动执行抓取序列。

如需手动模式：
```bash
ros2 run armtodeprition motion_planner_node --ros-args -p auto_execute:=false
```

手动触发抓取：
```bash
ros2 service call /execute_grasp std_srvs/srv/Trigger
```

---

## 启动步骤 (YOLO 目标检测)

用于实际目标检测和抓取。

### 终端 1: 启动 xArm6 MoveIt2 (真机)
```bash
ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py \
    robot_ip:=192.168.1.242 \
    add_gripper:=true
```

### 终端 2: 启动 OAK YOLO 检测
```bash
source ~/ros2_ws/install/setup.bash
ros2 run oak_yolo_py oak_yolo_node
```

### 终端 3: 启动坐标转换节点
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch oaktf_trantoarm transform.launch.py target_label:=orange
```

### 终端 4: 启动运动规划节点 (全自动模式)
```bash
source ~/ros2_ws/install/setup.bash
ros2 run armtodeprition motion_planner_node
```

> **全自动模式**: 检测到目标后，机械臂会自动执行抓取序列（间隔3秒防止重复触发）

---

## 话题和服务

### 话题
| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/oak_result` | std_msgs/String | OAK 检测结果 (label,conf,x,y,z) |
| `/oak/rgb/image_raw` | sensor_msgs/Image | 原始 RGB 图像 |
| `/oak/depth/image_raw` | sensor_msgs/Image | 深度图像 (JET colormap) |
| `/oak/rgb/image_annotated` | sensor_msgs/Image | 带检测框的 RGB 图像 |
| `/target_pose_in_base` | geometry_msgs/PoseStamped | 基座坐标系下的目标位姿 |

### 服务
| 服务名 | 类型 | 说明 |
|--------|------|------|
| `/execute_grasp` | std_srvs/Trigger | 执行完整抓取序列 |
| `/gripper_open` | std_srvs/Trigger | 打开夹爪 |
| `/gripper_close` | std_srvs/Trigger | 关闭夹爪 |

---

## 手动测试

```bash
# 查看 OAK 图像 (推荐用 rqt_image_view)
ros2 run rqt_image_view rqt_image_view
# 选择话题: /oak/rgb/image_raw, /oak/depth/image_raw, /oak/rgb/image_annotated

# 或用 RViz2 添加 Image 显示

# 如需显示 cv2 窗口 (调试用)
ros2 run oak_yolo_py oak_yolo_node --ros-args -p show_window:=true

# 测试夹爪
ros2 service call /gripper_open std_srvs/srv/Trigger
ros2 service call /gripper_close std_srvs/srv/Trigger

# 执行抓取
ros2 service call /execute_grasp std_srvs/srv/Trigger

# 查看坐标转换结果
ros2 topic echo /target_pose_in_base
```

---

## 参数配置

### oaktf_trantoarm 参数
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `target_label` | orange | 过滤的目标类别 |
| `base_frame` | link_base | 机械臂基座坐标系 |
| `r_base_cam` | [...] | 手眼标定旋转矩阵 (9元素) |
| `t_base_cam_mm` | [...] | 手眼标定平移向量 (mm) |

### armtodeprition 参数
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `planning_group` | xarm6 | MoveIt 规划组 |
| `approach_height` | 0.05m | 接近高度 |
| `grasp_height_offset` | 0.02m | 抓取高度偏移 |
| `auto_execute` | true | 全自动执行模式 |
| `auto_execute_interval` | 3.0s | 自动执行间隔 |

---

## 工作范围限制

系统自动检查并限制目标位置到安全范围内：

| 参数 | 范围 | 说明 |
|------|------|------|
| 水平距离 | 0.15m ~ 0.65m | 防止超出机械臂可达范围 |
| Z 高度 | 0.05m ~ 0.80m | 防止碰撞桌面或超高 |

---

## 障碍物配置

系统已预配置障碍物 (在 `armtodeprition/motion_planner_node.py`):

1. **左侧障碍物**: 底座左侧 7.5cm, 尺寸 12.2×10.2×16cm
2. **后方墙壁**: 底座后方 15cm
3. **左下角障碍物**: 前方 10cm, 左侧 20cm, 尺寸 15×15×20cm

如需修改，编辑 `add_workspace_obstacles()` 函数。

---

## 更新手眼标定矩阵

```bash
# 1. 采集标定数据
python3 ~/桌面/机械臂和相机相配合/chessboard_xarm_calib_collect.py \
    --calib oak_camera_calib.yaml --base-frame link_base --tcp-frame link_tcp

# 2. 计算标定结果
python3 ~/桌面/机械臂和相机相配合/solve_oak_xarm_handeye.py \
    --npz chessboard_xarm_calib_points.npz

# 3. 将输出的 R 和 t 更新到 oaktf_trantoarm/launch/transform.launch.py
```

---

## 机械臂相关文件结构

```
~/ros2_ws/src/
├── oak_yolo_py/                    # OAK 相机检测
├── oaktf_trantoarm/                # 坐标转换
│   ├── oaktf_trantoarm/
│   │   └── transform_node.py
│   └── launch/
│       └── transform.launch.py
├── armtodeprition/                 # 运动规划
│   ├── armtodeprition/
│   │   └── motion_planner_node.py
│   └── launch/
│       └── motion_planner.launch.py
└── xarm_ros2/                      # xArm SDK
```

---

## 注意事项

1. **末端执行器**: 使用 `add_gripper:=true` 启动 MoveIt，确保夹爪模型参与碰撞检测
2. **坐标单位**: OAK 输出 mm，ROS/MoveIt 使用 m
3. **姿态**: 默认抓取姿态为末端朝下 (绕X轴旋转180度)
