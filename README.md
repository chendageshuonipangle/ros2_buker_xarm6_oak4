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
 > **禁止在导航行进过程中操作机械臂，禁止在机械臂运动过程中发送导航目标。**
> **此警告没有例外，没有商量余地。**
>
> 本项目已移除 `bunker_xarm6_description` 联合启动包及所有相关 launch 文件。
> 原因：该包提供的"联合启动"功能在技术上无法保证安全，两套系统共享 TF 树会引发不可预测的坐标系竞争，同步运动存在人员伤害与设备损毁风险，因此彻底删除以防止误用。

---

本项目集成了两个子系统：
1. **Bunker Mini 移动机器人**：基于 RPLIDAR A1 + Nav2 的自主导航，支持 SLAM 建图与自定义路径规划算法（`ROS_DOMAIN_ID=20`）
2. **xArm6 机械臂视觉抓取**：OAK-D-SR 眼在手上标定 + NPU 目标检测 + MoveIt2 全自动抓取（`ROS_DOMAIN_ID=40`）

两套系统用不同的 `ROS_DOMAIN_ID` 强制隔离，节点互相看不见，这是上面那条警告的
技术保障手段。启动脚本已固化各自的 Domain，不要手动混用。

运维细节、标定流程与故障排查见 [`Bunker_Mini_Navigation_Guide.md`](Bunker_Mini_Navigation_Guide.md)。

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
sudo chmod a+rw /dev/ttyUSB0
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
OAK-D-SR (RVC2 NPU 上跑 YOLOv6r2) + 立体深度
    → /oak_result (label, conf, x, y, z 单位 mm)
        → target_to_base (眼在手上外参 + TF 树)
            → /target_pose_in_base (link_base 坐标系, 单位 m)
                → armtodeprition (MoveIt2 运动规划 + 夹爪控制)
```

相机装在 xArm6 末端法兰（eye-in-hand），外参已标定并固化在
`src/xarm_oak_handeye/config/eye_in_hand_result.json`，随包安装，无需每次重标。

### 一键启动（推荐）

```bash
cd ~/ros2_ws
./start_peach_grasp.sh check     # 七项环境自检
./start_peach_grasp.sh vision    # 只看检测，机械臂不会动
./start_peach_grasp.sh           # 全部启动，检测到稳定目标即自动抓取
```

脚本固定 `ROS_DOMAIN_ID=40`，按依赖顺序拉起 MoveIt2 → 手眼 TF + NPU 检测 →
运动规划，每层等前一层就绪，Ctrl+C 一次退出全部。

**默认是自动抓取**：预览窗口出现 `TARGET STABLE` 后就会动臂，启动前手要离开
工作区。要逐次确认改成手动：

```bash
AUTO_EXECUTE=false ./start_peach_grasp.sh   # 主终端
./start_peach_grasp.sh trigger              # 另开终端，触发一次
./start_peach_grasp.sh status               # 查链路是否通、目标点是否在发布
```

常用参数（全部通过环境变量传入）：

| 变量 | 默认 | 说明 |
|---|---|---|
| `AUTO_EXECUTE` | `true` | 自动抓取；`false` 改为手动 trigger |
| `TARGET_LABEL` | `peach` | 目标类别 |
| `CONFIDENCE` | `0.75` | 检测置信度阈值 |
| `GRIPPER_CLOSE_DEG` | `42.0` | 夹爪闭合角度，满闭合 48.7 |
| `TWIST_DEG` | `45.0` | 夹住后 joint6 拧转幅度，用于扭断果柄 |
| `TWIST_CYCLES` | `2` | 拧转轮数；`TWIST_ENABLE=false` 可关闭 |
| `PAYLOAD_KG` | `0.3` | 果子重量，用于力矩补偿，防止误报 C31 故障 |
| `MAX_DEPTH_MM` | `700` | 深度上限，掐掉远景误报 |

重启前务必先清干净，否则两个规划节点会同时向 MoveIt 发轨迹，
表现为全程 `CONTROL_FAILED`：

```bash
./start_peach_grasp.sh stop --with-moveit
```

机械臂报 C31 故障后（控制器会被停用，之后所有轨迹都被拒）：

```bash
./start_peach_grasp.sh recover
```

完整说明（四层启动顺序、三道安全闸、抓取动作序列、故障排查、
手眼标定复标流程）见 [`Bunker_Mini_Navigation_Guide.md`](Bunker_Mini_Navigation_Guide.md) 第 13、14 节。

### 手动分步启动（调试用）

```bash
# 终端 1：启动 xArm6 MoveIt2
ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py \
    robot_ip:=192.168.1.242 add_gripper:=true add_oak_d_sr:=true

# 终端 2：手眼 TF + NPU 检测 + 坐标转换
ros2 launch xarm_oak_handeye handeye_target_transform.launch.py \
    target_label:=peach confidence:=0.75

# 终端 3：运动规划
ros2 launch armtodeprition motion_planner.launch.py auto_execute:=false

# 终端 4：手动触发
ros2 service call /execute_grasp std_srvs/srv/Trigger
```

三个终端都需要先 `export ROS_DOMAIN_ID=40`，否则节点互相看不见。

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

| 方向 | 默认范围 |
|------|------|
| X | 0.10 m ~ 0.80 m |
| Y | -0.50 m ~ 0.50 m |
| Z | 0.00 m ~ 0.80 m |

限位在 `motion_planner.launch.py` 里用 `target_x_min` 等参数配置。
默认范围偏宽，视觉偶发的边缘坏点可能通过预检，建议按实际工作台收紧。

### 更新手眼标定

标定已固化，正常使用不需要重标。需要重标时用 `xarm_oak_handeye` 包
（AprilTag 板，眼在手上）：

```bash
export ROS_DOMAIN_ID=40
SESSION_DIR="$HOME/oak_handeye/session_$(date +%Y%m%d_%H%M%S)"

# 1. 采集：停稳机械臂后按 SPACE，残余运动会被自动拒绝
ros2 run xarm_oak_handeye collect_eye_in_hand --output-dir "$SESSION_DIR"

# 2. 求解
ros2 run xarm_oak_handeye solve_eye_in_hand \
    --samples "$SESSION_DIR/eye_in_hand_samples.npz"

# 3. 验收合格后把结果复制到 config/ 并重新编译
cp "$SESSION_DIR/eye_in_hand_result.json" \
   src/xarm_oak_handeye/config/eye_in_hand_result.json
colcon build --packages-select xarm_oak_handeye
```

验收标准与详细步骤见 `Bunker_Mini_Navigation_Guide.md` 第 13 节。

当前固化结果：21/23 样本，`translation_rms=6.393 mm`、`rotation_rms=0.286 deg`、
姿态跨度 55.8°。**6.4 mm 残差是 xArm6 正运动学的绝对精度，不是标定误差**，
不必继续压：同一 TCP 位姿重复采样时视觉噪声比残差小约 20 倍，且
TSAI / PARK / HORAUD / DANIILIDIS 四种算法结果全部落在 6.29–6.30 mm。

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
| `xarm_oak_handeye/config/eye_in_hand_result.json` | 手眼标定外参（已固化，随包安装）|
| `xarm_oak_handeye/xarm_oak_handeye/oak_peach_detector.py` | NPU 检测 + 立体深度三维定位 |
| `armtodeprition/armtodeprition/motion_planner_node.py` | 运动规划、夹爪行程、安全预检 |
| `armtodeprition/launch/motion_planner.launch.py` | 限位、自动抓取、夹爪角度参数 |
| `start_peach_grasp.sh` | 视觉抓取一键启动（Domain 40）|
| `start_bunker_navigation.sh` | 底盘导航一键启动（Domain 20）|

---

## 文件结构

```
ros2_buker_xarm6_oak4/
├── start_bunker_navigation.sh   # 底盘导航一键启动 (Domain 20)
├── start_arm_system.sh          # 机械臂分模式启动 (Domain 40)
├── start_peach_grasp.sh         # 视觉抓取一键启动 (Domain 40)
├── Bunker_Mini_Navigation_Guide.md   # 运维手册：分域/标定/抓取/排查
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
    ├── oak_yolo_py/            # OAK 相机 YOLO 检测与模型文件
    ├── xarm_oak_handeye/       # 眼在手上标定 + NPU 检测 + 坐标转换
    │   ├── config/             #   已固化外参
    │   └── xarm_oak_handeye/
    │       ├── collect_eye_in_hand.py
    │       ├── solve_eye_in_hand.py
    │       ├── oak_peach_detector.py
    │       └── target_to_base.py
    ├── oaktf_trantoarm/        # 旧版坐标转换（棋盘格流程保留）
    ├── armtodeprition/         # MoveIt2 运动规划
    └── xarm_ros2/              # xArm SDK
```

---

## 参考资料

- [Nav2 文档](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [linorobot2](https://github.com/linorobot/linorobot2)
- [xArm ROS2](https://github.com/xArm-Developer/xarm_ros2)
