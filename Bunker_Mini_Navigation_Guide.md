# Bunker + xArm6 双系统 ROS_DOMAIN_ID 分域启动说明

更新时间：2026-05-15

本文档记录当前 `~/ros2_ws` 工作空间中针对 **Bunker 底盘导航系统** 与 **xArm6 + OAK 机械臂视觉抓取系统** 的分域启动修改。

目标：以后不再手动执行 `export ROS_DOMAIN_ID=20` 或 `export ROS_DOMAIN_ID=40`，所有分域逻辑由启动脚本自动完成，避免两个 RViz2、TF、robot_state_publisher、MoveIt2、Nav2 等节点互相冲突。

---

## 1. 核心结论

当前系统被固定分成两个 ROS 2 Domain：

| 系统 | ROS_DOMAIN_ID | 启动脚本 | 作用 |
|---|---:|---|---|
| Bunker 底盘 / RPLIDAR / SLAM / Nav2 / RViz2 | `20` | `./start_bunker_navigation.sh` | 底盘导航域 |
| xArm6 / OAK YOLO / 坐标转换 / 机械臂运动规划 / RViz2 | `40` | `./start_arm_system.sh` | 机械臂抓取域 |

以后启动时直接运行对应脚本即可，不需要手动 export：

```bash
./start_bunker_navigation.sh bringup
./start_bunker_navigation.sh nav

./start_arm_system.sh moveit
./start_arm_system.sh oak
./start_arm_system.sh tf
./start_arm_system.sh planner
```

---

## 2. 本次修改的文件

### 2.1 修改文件：`start_bunker_navigation.sh`

路径：

```bash
/home/qluo/ros2_ws/start_bunker_navigation.sh
```

修改内容：在脚本开头固定加入：

```bash
export ROS_DOMAIN_ID=20
```

同时启动时会打印：

```text
ROS_DOMAIN_ID=20
```

这意味着所有通过该脚本启动的底盘、雷达、SLAM、Nav2、RViz2、路径记录、路点标记等节点都会进入 Domain 20。

### 2.2 新增文件：`start_arm_system.sh`

路径：

```bash
/home/qluo/ros2_ws/start_arm_system.sh
```

脚本内部固定加入：

```bash
export ROS_DOMAIN_ID=40
```

该脚本用于启动机械臂相关节点，所有通过该脚本启动的 xArm6、OAK、坐标转换、运动规划节点都会进入 Domain 40。

脚本已经设置可执行权限。

---

## 3. 为什么要这样改

如果两个系统都在默认 Domain 或同一个 Domain 中运行，会出现这些问题：

1. 两个 RViz2 互相看到不属于自己的 TF、RobotModel、Marker、Path。
2. 两个 `robot_state_publisher` 可能发布同名 `base_link`、`laser`、`camera_link` 等 TF，造成模型闪烁或跳变。
3. Nav2 和 MoveIt2 的话题/action/service 混在一起，排查困难。
4. 手动切换 `export ROS_DOMAIN_ID=20/40` 容易出错。
5. 一个终端里忘记切换 Domain，会导致节点跑到错误网络中。

现在脚本内部强制设置 Domain，避免人为错误。

---

## 4. 底盘导航系统启动方式（Domain 20）

### 4.1 启动硬件

终端 1：

```bash
cd ~/ros2_ws
sudo chmod a+rw /dev/ttyUSB0
./start_bunker_navigation.sh bringup
```

说明：

- 该脚本会自动设置 `ROS_DOMAIN_ID=20`。
- `sudo chmod a+rw /dev/ttyUSB0` 用于给 RPLIDAR 串口临时授权。
- 如果使用 `/dev/rplidar` udev 规则并且权限正常，可以不每次执行 chmod。

### 4.2 SLAM 建图

终端 2：

```bash
cd ~/ros2_ws
./start_bunker_navigation.sh slam
```

说明：

- 自动使用 `ROS_DOMAIN_ID=20`。
- 用于运行 SLAM 建图。

### 4.3 地图导航

终端 2 或新终端：

```bash
cd ~/ros2_ws
./start_bunker_navigation.sh nav
```

说明：

- 自动使用 `ROS_DOMAIN_ID=20`。
- 默认地图：

```bash
/home/qluo/ros2_ws/maps/my_map.yaml
```

如果需要指定地图：

```bash
./start_bunker_navigation.sh nav /home/qluo/ros2_ws/maps/my_map.yaml
```

### 4.4 其他底盘模式

当前 `start_bunker_navigation.sh` 支持：

```bash
./start_bunker_navigation.sh bringup
./start_bunker_navigation.sh bringup_with_imu
./start_bunker_navigation.sh bringup_no_imu
./start_bunker_navigation.sh bringup_no_imu_lite
./start_bunker_navigation.sh slam
./start_bunker_navigation.sh slam_nav
./start_bunker_navigation.sh nav
./start_bunker_navigation.sh nav_lite
./start_bunker_navigation.sh nav_origin
./start_bunker_navigation.sh save_map
```

这些模式全部自动运行在 `ROS_DOMAIN_ID=20`。

---

## 5. 机械臂视觉抓取系统启动方式（Domain 40）

### 5.1 启动 xArm6 MoveIt2

终端 3：

```bash
cd ~/ros2_ws
./start_arm_system.sh moveit
```

等价于原来的：

```bash
ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py \
    robot_ip:=192.168.1.242 add_gripper:=true
```

但现在会自动使用：

```bash
ROS_DOMAIN_ID=40
```

### 5.2 启动 OAK YOLO 检测

终端 4：

```bash
cd ~/ros2_ws
./start_arm_system.sh oak
```

等价于原来的：

```bash
ros2 run oak_yolo_py oak_yolo_node
```

### 5.3 启动坐标转换

终端 5：

```bash
cd ~/ros2_ws
./start_arm_system.sh tf
```

默认目标类别是：

```text
orange
```

等价于：

```bash
ros2 launch oaktf_trantoarm transform.launch.py target_label:=orange
```

如果要换目标类别：

```bash
./start_arm_system.sh tf orange
```

### 5.4 启动运动规划节点

终端 6：

```bash
cd ~/ros2_ws
./start_arm_system.sh planner
```

等价于原来的：

```bash
ros2 run armtodeprition motion_planner_node
```

---

## 6. 推荐完整启动流程

### 6.1 底盘导航流程

终端 1：

```bash
cd ~/ros2_ws
sudo chmod a+rw /dev/ttyUSB0
./start_bunker_navigation.sh bringup
```

终端 2：

```bash
cd ~/ros2_ws
./start_bunker_navigation.sh nav
```

如果需要建图，则终端 2 改为：

```bash
./start_bunker_navigation.sh slam
```

### 6.2 机械臂抓取流程

终端 3：

```bash
cd ~/ros2_ws
./start_arm_system.sh moveit
```

终端 4：

```bash
cd ~/ros2_ws
./start_arm_system.sh oak
```

终端 5：

```bash
cd ~/ros2_ws
./start_arm_system.sh tf
```

终端 6：

```bash
cd ~/ros2_ws
./start_arm_system.sh planner
```

---

## 7. 验证分域是否生效

### 7.1 验证底盘域

```bash
ROS_DOMAIN_ID=20 ros2 node list
```

应看到底盘相关节点，例如：

```text
/bunker
/robot_state_publisher
/sllidar_node
/laser_filter
/laser_filter_slam
/ekf_filter_node
/path_recorder
/waypoint_marker
/map_server
/amcl
/controller_server
/planner_server
/bt_navigator
```

不应该看到 xArm6、OAK、机械臂运动规划相关节点。

### 7.2 验证机械臂域

```bash
ROS_DOMAIN_ID=40 ros2 node list
```

应看到机械臂相关节点，例如：

```text
xArm6 / MoveIt2 相关节点
oak_yolo_node
坐标转换节点
motion_planner_node
```

不应该看到 Bunker、Nav2、SLAM、RPLIDAR 相关节点。

### 7.3 验证脚本强制覆盖当前终端 Domain

即使当前终端误设为：

```bash
export ROS_DOMAIN_ID=99
```

运行底盘脚本仍会显示：

```bash
./start_bunker_navigation.sh help
```

输出应包含：

```text
ROS_DOMAIN_ID=20
```

运行机械臂脚本：

```bash
./start_arm_system.sh help
```

输出应包含：

```text
ROS_DOMAIN_ID=40
```

---

## 8. 注意事项

### 8.1 脚本内 export 不会污染当前终端

脚本里的：

```bash
export ROS_DOMAIN_ID=20
```

或：

```bash
export ROS_DOMAIN_ID=40
```

只影响脚本进程和它启动的 ROS 节点，不会永久改变当前终端环境。

这是正确行为。

### 8.2 手动 ros2 命令需要指定 Domain

如果你不是通过脚本启动，而是在终端里手动执行 `ros2 node list`、`ros2 topic list`、`ros2 topic echo`，需要指定要看的 Domain。

查看底盘域：

```bash
ROS_DOMAIN_ID=20 ros2 topic list
```

查看机械臂域：

```bash
ROS_DOMAIN_ID=40 ros2 topic list
```

如果直接运行：

```bash
ros2 node list
```

它只会使用当前终端已有的 `ROS_DOMAIN_ID`，可能看不到你想看的节点。

### 8.3 数据采集脚本需要在底盘域运行

`collect_nav_data.sh` 是采集底盘导航数据的脚本，应在 Domain 20 下运行。

推荐：

```bash
cd ~/ros2_ws
ROS_DOMAIN_ID=20 ./collect_nav_data.sh improved 300
```

### 8.4 不要混用原始命令和分域脚本

比如不要在未设置 Domain 的终端里直接运行：

```bash
ros2 run oak_yolo_py oak_yolo_node
```

推荐统一使用：

```bash
./start_arm_system.sh oak
```

同理，底盘统一使用：

```bash
./start_bunker_navigation.sh bringup
```

---

## 9. 故障排查

### 9.1 RViz 看不到另一个系统

这是正常的。

底盘 RViz 在 Domain 20，机械臂 RViz 在 Domain 40，两个 RViz 默认互相不可见。

### 9.2 `ros2 node list` 没有节点

先确认你查的是哪个 Domain：

```bash
ROS_DOMAIN_ID=20 ros2 node list
ROS_DOMAIN_ID=40 ros2 node list
```

### 9.3 Nav2 没有反应

确认你在底盘域：

```bash
ROS_DOMAIN_ID=20 ros2 action list
ROS_DOMAIN_ID=20 ros2 topic list
```

确认 Nav2：

```bash
ROS_DOMAIN_ID=20 ros2 lifecycle get /bt_navigator
ROS_DOMAIN_ID=20 ros2 lifecycle get /controller_server
ROS_DOMAIN_ID=20 ros2 lifecycle get /planner_server
```

### 9.4 OAK 或机械臂节点看不到

确认你在机械臂域：

```bash
ROS_DOMAIN_ID=40 ros2 node list
ROS_DOMAIN_ID=40 ros2 topic list
```

### 9.5 两边节点又混在一起了

检查是否有人绕过脚本，直接手动启动了节点。

正确做法：

```bash
./start_bunker_navigation.sh ...
./start_arm_system.sh ...
```

不要手动在同一个 Domain 里启动两个系统。

---

## 10. 当前验证结果

已执行语法检查：

```bash
bash -n start_bunker_navigation.sh
bash -n start_arm_system.sh
```

结果：无语法错误。

已验证强制分域：

```text
bunker: ROS_DOMAIN_ID=20
arm: ROS_DOMAIN_ID=40
```

视觉抓取链路（Domain 40）已在真机验证：

```text
手眼标定    used=21/23, orientation_span=55.8 deg
            translation_rms=6.393 mm, rotation_rms=0.286 deg
检测        peach conf 0.91-0.96, spread 2.1-14.3 mm (稳定窗口内)
帧率        检测 ~18 fps, 预览 ~5 fps (USB2 上限)
抓取        预检通过 -> STAGE 1..4 全程无失败 -> 回 hold-up 位
```

`bash -n start_peach_grasp.sh` 无语法错误；`check` 七项自检全绿；
`stop --with-moveit` 已验证能清掉顽固的 `move_group` 进程。

---

## 11. 最终记忆版命令

### 底盘

```bash
cd ~/ros2_ws
sudo chmod a+rw /dev/ttyUSB0
./start_bunker_navigation.sh bringup
```

```bash
cd ~/ros2_ws
./start_bunker_navigation.sh nav
```

### 机械臂

```bash
cd ~/ros2_ws
./start_arm_system.sh moveit
```

```bash
cd ~/ros2_ws
./start_arm_system.sh oak
```

```bash
cd ~/ros2_ws
./start_arm_system.sh tf
```

```bash
cd ~/ros2_ws
./start_arm_system.sh planner
```

---

### 视觉抓取（Domain 40）

```bash
cd ~/ros2_ws
./start_peach_grasp.sh stop --with-moveit   # 重启前清干净
./start_peach_grasp.sh                      # 全部启动，自动抓取
```

---

## 12. 当前文件状态

| 文件 | 状态 |
|---|---|
| `/home/qluo/ros2_ws/start_bunker_navigation.sh` | 已修改，固定 `ROS_DOMAIN_ID=20` |
| `/home/qluo/ros2_ws/start_arm_system.sh` | 已新增，固定 `ROS_DOMAIN_ID=40`，分模式手动启动 |
| `/home/qluo/ros2_ws/start_peach_grasp.sh` | 已新增，视觉抓取一键启动（见第 14 节） |
| `/home/qluo/ros2_ws/Bunker_Mini_Navigation_Guide.md` | 已替换为本文档 |
| `src/armtodeprition/armtodeprition/motion_planner_node.py` | 已修改，防重复实例、运动串行、夹爪角度参数、目标时效 |
| `src/armtodeprition/launch/motion_planner.launch.py` | 已修改，补 `auto_execute` 与夹爪角度参数 |
| `src/xarm_oak_handeye/xarm_oak_handeye/oak_peach_detector.py` | 已新增，NPU 检测 + 深度三维定位 + 预览窗口 |
| `src/xarm_oak_handeye/xarm_oak_handeye/target_to_base.py` | 已新增，相机坐标转 `link_base` 并发幽灵 TF |
| `src/xarm_oak_handeye/launch/handeye_target_transform.launch.py` | 已新增，TF + 检测 + 转换三节点 |
| `src/xarm_oak_handeye/config/eye_in_hand_result.json` / `.yaml` | 已固化外参，随包安装 |

---

## 13. xArm6 + OAK-D-SR 眼在手上标定指南（Domain 40）

本节用于重新标定安装在 xArm6 末端法兰上的 OAK-D-SR。标定模式为
**眼在手上（eye-in-hand）**：相机随末端运动，AprilTag 板固定在工作台，
最终求取固定变换：

```text
link_tcp -> oak_rgb_camera_optical_frame
```

OAK-D-SR 安装板 STL 已纳入机械臂模型，并已按实际安装方向从法兰外侧观察
顺时针旋转 90 度。标定完成并验收后，还可以把相机本体的保守碰撞包络加载进
MoveIt2。

### 13.1 固定参数与验收标准

当前工具的默认参数对应现场的 DFOPTIX AprilTag 板：

| 项目 | 参数 |
|---|---:|
| 阵列 | 6 x 6 |
| 黑色标签方块边长 | 16.5 mm |
| 标定板总尺寸 | 133.65 mm |
| 方块中心间距 | 23.43 mm |
| 相机 | OAK-D-SR `CAM_B`，1280 x 800，自动曝光 |
| 机械臂 IP | `192.168.1.242` |
| ROS 2 Domain | `40` |

验收时以求解器输出为准：

| 指标 | 合格要求 |
|---|---:|
| 使用样本数 | 至少 12 个姿态 |
| 姿态跨度 | 至少 20 deg，建议大于 60 deg |
| 单帧网格点 | 至少 30 / 36，建议 36 / 36 |
| 单帧 PnP 重投影误差 | 小于 0.6 px |
| 手眼平移 RMS | 小于 3 mm |
| 手眼旋转 RMS | 小于 1 deg |

只要 `translation_rms` 大于 3 mm，就不要把该结果发布为正式 TF，也不要加载到
MoveIt 的相机碰撞包络。当前 2026-08-12 的同步采集结果为 `6.337 mm`，因此属于
需要补采的结果，而不是可用的正式外参。

### 13.2 标定前准备

1. 让 AprilTag 板牢固固定在工作台或场地中，整个采集过程都不能移动它。
2. 检查 OAK-D-SR、法兰安装板和电缆均已固定；规划机械臂姿态时为相机和电缆留出余量。
3. 启动过程中只能打开一个使用 OAK 的程序。采集器、预览程序和 YOLO 节点不能同时抢占相机。
4. 启动真机 MoveIt2。该终端需要一直保持运行：

```bash
cd ~/ros2_ws
./start_arm_system.sh moveit
```

在另一个终端检查真机关节状态与 TCP TF。两条命令都必须能获得数据，不能在
xArm 关机或未连接时采集：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=40

ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo link_base link_tcp
```

第二条命令持续显示 TCP 位姿即表示正常。按 `Ctrl+C` 退出该检查即可。

### 13.3 检查相机与标定板

关闭其他 OAK 程序后，在 Domain 40 终端启动预览：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=40

ros2 run xarm_oak_handeye test_apriltag_board
```

预览窗口应显示绿色的网格点和姿态轴。优先让板占据画面的中间至中等大小范围，
避免太远、反光、过暗或只剩很少方块。状态栏理想状态为：

```text
outer-square grid=36/36  reproj=约 0.2 至 0.4 px
```

按 `Q` 或 `ESC` 关闭预览后再启动采集器。以下 Qt 字体提示和
`drawFrameAxes ... endpoints are out of frame` 警告仅影响界面字体或坐标轴绘制，
不影响已接受的样本数据。

### 13.4 同步采集数据

以下命令会创建一个带时间戳的新目录；请在同一个终端完成采集和求解，以便保留
`SESSION_DIR` 变量：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=40

SESSION_DIR="$HOME/oak_handeye/session_$(date +%Y%m%d_%H%M%S)_sync"
ros2 run xarm_oak_handeye collect_eye_in_hand --output-dir "$SESSION_DIR"
```

采样操作：

1. 将机械臂移动到一个安全姿态，确保相机、安装板与电缆均不会碰撞。
2. 等机械臂完全停止至少 1 秒。
3. 确认窗口有可接受的绿色网格，然后按一次 `Space`。
4. 终端必须显示 `sample N: accepted 1 frames`，才计入一组有效数据。
5. 重复 12 至 18 个姿态，推荐 15 个以上。让板依次位于图像中心和边缘，并改变相机
   的 roll、pitch、yaw 与距离；不要只在一个平面平移机械臂。
6. 采完按 `S` 保存，再按 `Q` 或 `ESC` 退出。

采集器会为每一个新图像，使用该图像时间戳查询对应的
`link_base -> link_tcp` TF。不要用旧版本采集器或手工把图像与当前 TCP 位姿配对，
否则会产生数毫米到厘米级的手眼误差。

成功保存后会显示类似：

```text
saved 15 samples: /home/qluo/oak_handeye/session_YYYYMMDD_HHMMSS_sync/eye_in_hand_samples.npz
```

如果空格后没有接受样本，终端末尾会包含原因：

| 输出片段 | 处理方法 |
|---|---|
| `board=...` 或 `grid=...` | 调整光照、对焦、距离或板在画面中的位置，确保至少 30 个方块。 |
| `reprojection=...` | 减少反光、模糊和强透视，重新对准标定板。 |
| `tf=...` | 检查 xArm 已上电且 MoveIt2 正在运行；重新确认 `/joint_states` 与 TCP TF。 |
| `no_frame=...` | 确认 OAK USB 连接正常，且没有另一个相机程序在运行。 |

### 13.5 求解与验收

采集完成后，在保留 `SESSION_DIR` 的同一个终端运行：

```bash
ros2 run xarm_oak_handeye solve_eye_in_hand \
  --samples "$SESSION_DIR/eye_in_hand_samples.npz"
```

如果已经关闭原终端，则把目录替换为实际目录：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=40

ros2 run xarm_oak_handeye solve_eye_in_hand \
  --samples ~/oak_handeye/session_YYYYMMDD_HHMMSS_sync/eye_in_hand_samples.npz
```

求解器会输出：

```text
result: .../eye_in_hand_result.json
quality: used=15/15, orientation_span=..., translation_rms=... mm, rotation_rms=... deg
```

同一目录中会生成：

```text
eye_in_hand_samples.npz       原始同步样本
images/sample_*.png           每个姿态的保存画面
eye_in_hand_result.json       程序使用的完整标定结果
eye_in_hand_result.yaml       便于人工读取的 TCP 到相机变换
```

若结果不合格，不要覆盖或删除原始 session。保留它用于比较，然后新建一个 session
重新采集。优先增加姿态方向变化和有效方块覆盖，而不是在相近姿态反复采很多帧。

### 13.6 发布合格外参并验算 TF

仅对已经通过第 13.1 节验收的结果执行。以下命令发布 TCP 到相机的静态 TF，并将
来自 OAK 的目标结果转换到 `link_base`：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=40

ros2 launch xarm_oak_handeye handeye_target_transform.launch.py \
  result:=~/oak_handeye/session_YYYYMMDD_HHMMSS_sync/eye_in_hand_result.json \
  target_label:=orange
```

另开一个 Domain 40 终端确认发布的相机 TF：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=40

ros2 run tf2_ros tf2_echo link_tcp oak_rgb_camera_optical_frame
```

验算方法：保持标定板或一个固定目标不动，缓慢将机械臂移动到 3 个不同姿态。若由
视觉计算出的固定目标在 `link_base` 坐标系中持续跳动超过几毫米，应停止使用该结果，
检查板是否移动、是否有图像模糊，并重新采集。

### 13.7 将已验收结果加载为 MoveIt 相机碰撞包络

先在原先运行 `./start_arm_system.sh moveit` 的终端按 `Ctrl+C` 彻底停止旧 MoveIt2。
不要同时启动两套 MoveIt/robot_state_publisher。然后在新 Domain 40 终端运行：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=40

ros2 run xarm_oak_handeye launch_moveit_with_oak \
  --result ~/oak_handeye/session_YYYYMMDD_HHMMSS_sync/eye_in_hand_result.json \
  --robot-ip 192.168.1.242 \
  --camera-safety-radius-m 0.065
```

这会加载实际 OAK-D-SR 法兰安装板 STL，并在已标定的相机位置加入半径 `65 mm` 的
保守碰撞包络。不要把该半径调小到相机实体、接头和安装误差之外；相机线缆的路径仍
需要通过工作空间禁入区和人工检查保护。

### 13.8 常用复标定命令速查

```bash
# 1. 启动真机 MoveIt2（保持运行）
cd ~/ros2_ws && ./start_arm_system.sh moveit
```

```bash
# 2. 采集新 session
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=40
SESSION_DIR="$HOME/oak_handeye/session_$(date +%Y%m%d_%H%M%S)_sync"
ros2 run xarm_oak_handeye collect_eye_in_hand --output-dir "$SESSION_DIR"
```

```bash
# 3. 求解当前 session
ros2 run xarm_oak_handeye solve_eye_in_hand \
  --samples "$SESSION_DIR/eye_in_hand_samples.npz"
```

### 13.9 自动复采已验证姿态

当已经有一组手动采集的同步样本后，可以让 MoveIt2 低速重放这些**已经实际到达且
确认看得到标定板**的 TCP 姿态。自动程序不根据未验收外参凭空生成新工作空间点；它
只把已有样本当作安全种子，逐段由 MoveIt2 进行碰撞规划。每个执行点停稳后自动采集
5 帧并将各帧与其图像时间戳对应的 TCP TF 配对。

自动复采的目的主要是降低单帧视觉噪声、验证可重复性；它本身不会增加原始 14 个
姿态的几何覆盖。若自动复采后的平移 RMS 仍大于 3 mm，需要从新的方向和距离补充
人工确认的种子姿态，而不是无限重放同一批姿态。

保持普通 `./start_arm_system.sh moveit` 的 MoveIt2 终端运行。自动程序会把 OAK-D-SR
相对 TCP 的 `80 mm` 保守相机包络作为附着碰撞体加入当前规划场景，并同时加入固定
AprilTag 板禁入体，因此不需要为了自动复采而再启动第二套 MoveIt2。

另开一个 Domain 40 终端先执行纯规划预演。该命令**不会
发送任何机械臂运动**，但会检查全部种子终点是否可达、是否与安装板、相机包络和标定板
禁入体碰撞：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=40

ros2 run xarm_oak_handeye auto_eye_in_hand \
  --seed-samples ~/oak_handeye/session_20260812_190715_sync/eye_in_hand_samples.npz \
  --seed-result ~/oak_handeye/session_20260812_190715_sync/eye_in_hand_result.json
```

纯规划成功时，终端最后会显示：

```text
dry-run passed: all selected seed poses are reachable; no robot motion was commanded
```

确认周围无人、标定板和电缆固定、急停可用后，执行下列命令才会让机械臂低速移动。它会
新建 session，逐点采集；任何一个点无法接受标定板时会停止，不会继续盲目运动：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=40

AUTO_SESSION="$HOME/oak_handeye/auto_session_$(date +%Y%m%d_%H%M%S)"
ros2 run xarm_oak_handeye auto_eye_in_hand \
  --seed-samples ~/oak_handeye/session_20260812_190715_sync/eye_in_hand_samples.npz \
  --seed-result ~/oak_handeye/session_20260812_190715_sync/eye_in_hand_result.json \
  --output-dir "$AUTO_SESSION" \
  --execute \
  --allow-preliminary-result
```

自动采集结束后求解新数据：

```bash
ros2 run xarm_oak_handeye solve_eye_in_hand \
  --samples "$AUTO_SESSION/eye_in_hand_samples.npz"
```

---

## 14. 视觉抓取一键启动（Domain 40）

标定已验收固化后，日常抓取不需要再手动开四个终端。`start_peach_grasp.sh`
按依赖顺序拉起全部四层，每层等前一层就绪才继续，Ctrl+C 一次退出全部。

固化的手眼外参已随包安装到
`install/xarm_oak_handeye/share/xarm_oak_handeye/config/eye_in_hand_result.json`，
脚本直接读取，无需再指定 session 路径。

### 14.1 快速开始

```bash
cd ~/ros2_ws

# 先自检，不启动任何节点
./start_peach_grasp.sh check

# 只看检测，机械臂不会动（推荐先跑这个确认目标点合理）
./start_peach_grasp.sh vision

# 全部启动，检测到稳定目标即自动抓取
./start_peach_grasp.sh
```

默认是**自动抓取**：画面出现 `TARGET STABLE` 后就会动臂，启动前手要离开工作区。
先跑 `vision` 模式确认目标点合理，再跑 `all`。

脚本自己会 `source` 环境并设置 `ROS_DOMAIN_ID=40`，不需要在当前终端预先设置。

### 14.2 四层启动顺序

| 层 | 内容 | 就绪判据 |
|---|---|---|
| 1 | xArm6 MoveIt2 真机控制 | `/move_group` 节点出现 |
| 2 | 手眼标定 TF + NPU 检测 | `oak_peach_detector` 出现且 TF 可查 |
| 3 | 运动规划节点 | `arm_motion_planner_node` 出现 |
| 4 | 状态监控 | 跟随规划日志，任一子进程退出即整体收尾 |

顺序不能颠倒：规划节点若先于 TF 启动，会拿到不完整的变换树。

启动前自检会拒绝带病启动，共七项：工作区、OAK 连接与 USB 速率、模型文件、
手眼外参、三类残留进程、机械臂网络。任一项红灯即中止（`vision` 模式只告警）。

### 14.3 触发抓取

默认 `AUTO_EXECUTE=true`：检测端判稳后规划节点每 3 s 轮询一次，拿到新鲜目标点
就自动执行。**动臂不需要任何人工确认，启动前确认工作区无人。**

需要逐次确认时关掉自动，改手动触发：

```bash
AUTO_EXECUTE=false ./start_peach_grasp.sh
```

手动模式下，预览窗口显示 `TARGET STABLE` 时另开一个终端：

```bash
cd ~/ros2_ws
./start_peach_grasp.sh trigger
```

用脚本而不是手敲 `ros2 service call`：新终端如果忘了 `source` 或忘了设
`ROS_DOMAIN_ID=40`，服务名解析不到，命令静默失败，现象就是"机械臂完全不动"
且规划节点日志里一行都不多。脚本自己带好环境，并会把服务返回值打出来。

返回 `success=True` 并带上目标点坐标表示已触发。其他返回：

| 返回 | 含义 |
|---|---|
| `没有可用目标点` | 检测未稳定（画面不是 TARGET STABLE），或点被限位拒绝 |
| `目标点已过期 Ns` | 距上次稳定检测超过 2s，目标或臂可能已移动，重新对准再触发 |
| `正在执行抓取` | 上一次还没结束 |

臂不动时先确认链路通不通：

```bash
cd ~/ros2_ws
./start_peach_grasp.sh status
```

它会分别检查检测节点、规划节点、`/execute_grasp` 服务是否在线，并真的抓一帧
`/target_pose_in_base` 打出坐标。四项全绿才说明"不动"是触发问题而非链路问题。

确认多次抓取都正常后，再放开自动模式：

```bash
AUTO_EXECUTE=true ./start_peach_grasp.sh
```

### 14.4 可调环境变量

| 变量 | 默认 | 说明 |
|---|---|---|
| `AUTO_EXECUTE` | `true` | 检测到稳定目标自动抓取；设 `false` 改为手动 `trigger` |
| `TARGET_LABEL` | `peach` | 目标类别 |
| `CONFIDENCE` | `0.75` | 检测置信度阈值 |
| `FPS` | `18.0` | 检测与深度帧率，USB2 实测上限 |
| `PREVIEW_FPS` | `5.0` | 预览帧率，只供 GUI |
| `MIN_DEPTH_MM` / `MAX_DEPTH_MM` | `150` / `700` | 深度有效窗口 |
| `SHOW` | `true` | 是否开预览窗口 |
| `FULLSCREEN` | `true` | 预览是否全屏 |
| `GRIPPER_CLOSE_DEG` | `42.0` | 夹爪闭合角度 (deg)，满闭合 48.7 |
| `ROBOT_IP` | `192.168.1.242` | 机械臂 IP |

```bash
# USB3 口下跑满帧率
FPS=30.0 PREVIEW_FPS=10.0 ./start_peach_grasp.sh

# 无显示器环境
SHOW=false ./start_peach_grasp.sh
```

### 14.5 预览窗口说明

| 显示 | 含义 |
|---|---|
| `TARGET STABLE` + 绿框 | 目标已稳定，坐标已发布，可以抓 |
| `TARGET SETTLING` + 橙框 | 正在判稳或抖动过大，已拦下不发布 |
| `NO TARGET` | 未检测到目标 |
| 框下 `settling 3/8` | 已累积 3 帧，需连续 8 帧 |
| 框下 `jitter 45mm > 15mm` | 散布超限，判为噪声 |

按 `f` 切换全屏，按 `q` 只关预览、检测继续运行。

预览是独立的低速流（默认 5fps），检测与深度走全速（18fps）。USB2 带宽有限，
把预览降速几乎不影响检测（实测代价约 0.4fps），但若让预览也走全速，
整条管线会被拖到 1fps 以下。

### 14.6 三道安全闸

抓取链路上有三处过滤，目的是避免单帧检测噪声驱动真实运动。

**闸 1：时间一致性（检测端）+ 时效（规划端）**
目标点必须连续 8 帧稳定在 15 mm 内才发布，且发布的是这 8 帧均值而非最新帧。
规划端另有 2 s 时效窗口：超过 2 s 的目标点一律丢弃（自动与手动同一规则），
避免拿"目标曾经在哪"去驱动运动。
目标丢失立即清空窗口。神经网络检测框天生抖动，而这个话题下游直接驱动机械臂，
所以单帧噪声绝不能形成运动指令。

**闸 2：深度窗口（检测端）**
只接受 150–700 mm 的深度。模型在空场景下会以 0.5 左右置信度误报在数米外的
背景上，深度门限从源头掐掉这类误报。

**闸 3：运动前预检（规划端）**
动臂之前先算出 Point A 与 Point B，两点都要过限位检查，再各调一次
`/compute_ik` 确认可达。任一项不过就放弃本次抓取，臂不动。

到达 A 点后还会用**实际**位姿重算 B 点并再验一次。因为规划器为满足姿态约束
常会挑到与规划值偏离较大的解（实测规划 A 为 `y=0`、实际到 `y=-0.295`），
基于规划 A 预检的 B 不能代表真实 B。这道复检才是真正挡住
`CONTROL_FAILED` 卡死的检查。

### 14.7 抓取动作序列

| 阶段 | 动作 |
|---|---|
| 预检 | 校验 A/B 限位与可达性 |
| STAGE 1 | PTP 到 Point A（目标后方 27 cm），张开夹爪 |
| STAGE 2 | 前进 8 cm 到 Point B，继承 A 点实际姿态，闭合夹爪至 42° |
| STAGE 3 | PTP 退回 Point A |
| STAGE 4 | 关节运动回 hold-up 位（相机正视前方，便于找下一个目标），松开夹爪 |

夹爪行程对照（`drive_joint`，MoveIt 界面显示角度、话题里是弧度）：

| 角度 | 弧度 | 含义 |
|---|---|---|
| 0° | 0.000 | 全张，STAGE 1 与收尾用 |
| 42° | 0.733 | 当前抓取用值，`GRIPPER_CLOSE_DEG` 默认 |
| 48.7° | 0.850 | 全闭，SRDF `close` 状态，也是裁剪上限 |

Point A 为 `target.x - 0.27`（夹爪 17 cm + 间隙 10 cm），
Point B 前进量比后退量少 2 cm，终点停在 `target.x - 0.19`，避免夹爪顶到目标。

夹爪 `drive_joint` 的单位是弧度，MoveIt 界面显示的是角度：SRDF 里 `open=0` rad、
`close=0.85` rad 即 48.7°，对应界面上的全张与全闭。抓取用 42°（0.733 rad），
留一点余量夹住而不压坏。节点会把该值裁剪到 0.85 rad 以内，参数写错也不会顶到机械限位。

STAGE 2 刻意继承 A 点实际姿态而不强行修正到理想姿态：实测若在 A 点姿态已有偏差，
强行要求 B 点摆正会导致规划器原地不动。闭合夹爪前会校验实际位移，
不足 1 cm 判定为未前进并中止。

### 14.8 已知限制

**接近方向是硬编码的水平进给。** `TARGET_RPY = [180, -90, 0]`，从 -X 方向水平
推进。目标平放在台面上时，这个方向并不理想，可能出现反复预检失败。
台面未纳入碰撞模型（`add_workspace_obstacles` 只加了后方安全墙
`rear_safety_wall`），依赖预检和限位保护，不依赖碰撞检测。

**目标限位仍然偏宽。** `target_x_max` 默认 0.80，比实际工作台需要的范围大。
限位太松时，视觉偶发的边缘坏点能通过预检走到臂上，历史上出现过走到
`y=-0.295` 后 `CONTROL_FAILED` 卡死。收紧需要现场量出果子的实际摆放范围，
用 `target_x_max` 等参数改，暂未固化。

**USB2 限制检测帧率。** 设备端时间戳显示 NN 实际以 30 fps 推理，
但主机只收到约 18 fps，差额掉在 USB2 回传上。插到 USB3 口即可跑满。
另外 shave 数并非越多越好：实测 5 shaves 为 20.2 fps，10 shaves 反而降到
13.2 fps，因为 shave 会与 ISP 争抢 CMX 资源。

### 14.9 不要留下两个规划节点

`CONTROL_FAILED` 最常见的原因不是机械臂坏了，而是**同时有两个
`arm_motion_planner_node` 在向 MoveIt 发轨迹**。MoveIt 对第二个轨迹直接回
`Cannot push a new trajectory while another is being executed`，向上冒成
`CONTROL_FAILED`，日志看起来和真实硬件故障一模一样。

已做三重防护：

- 节点启动时检查图上是否已有同名节点，有则拒绝启动并打印清理命令
- 同一节点内 `move_ptp` / `move_joints` 共用一把 `motion_lock`，串行下发
- 脚本自检会扫 `motion_planner_node`、`oak_peach_detector`、`move_group`
  三类残留进程，发现就拒绝启动

重启前先清干净：

```bash
./start_peach_grasp.sh stop                 # 只停视觉与规划
./start_peach_grasp.sh stop --with-moveit    # 连 MoveIt / ros2_control 一起停
./start_peach_grasp.sh check                 # 确认三项残留检查全绿
```

`stop` 只匹配 xArm 相关进程名，不会影响底盘 Domain 20 的节点。

### 14.10 故障排查

| 现象 | 原因与处理 |
|---|---|
| `没检测到 OAK 相机` | 相机被其他程序占用，先关掉别的 depthai 程序 |
| 一直 `TARGET SETTLING` | 目标在动或深度不稳；放稳目标，距离控制在 300–450 mm |
| `没有可用目标点` | 检测未稳定，或点被限位拒绝，看规划节点日志的限位告警 |
| `Point B 不可达` | 目标在工作空间边缘，把目标往底座方向挪近 |
| `CONTROL_FAILED` | 先查是否有两个规划节点在抢（见 14.9）；确认唯一后再清错误重新使能 |

机械臂报错后恢复：

```bash
export ROS_DOMAIN_ID=40
ros2 service list | grep xarm          # 先确认服务名
ros2 service call /xarm/clean_error xarm_msgs/srv/Call
ros2 service call /xarm/motion_enable xarm_msgs/srv/SetInt16ById "{id: 8, data: 1}"
ros2 service call /xarm/set_state xarm_msgs/srv/SetInt16 "{data: 0}"
```

停止残留节点（不会误杀底盘节点）：

```bash
./start_peach_grasp.sh stop
```

日志按启动时间保存在 `log/peach_grasp_<时间戳>/`，分
`1_moveit.log`、`2_vision.log`、`3_planner.log`。

### 14.11 脚本模式速查

| 命令 | 作用 |
|---|---|
| `./start_peach_grasp.sh` | 全部启动，默认自动抓取 |
| `./start_peach_grasp.sh vision` | 只启动 TF + 检测，臂不会动 |
| `./start_peach_grasp.sh check` | 只做七项环境自检 |
| `./start_peach_grasp.sh status` | 查节点/服务在线情况，并抓一帧目标点坐标 |
| `./start_peach_grasp.sh trigger` | 手动触发一次抓取（自带环境，免手工 source） |
| `./start_peach_grasp.sh stop` | 停视觉与规划，保留 MoveIt |
| `./start_peach_grasp.sh stop --with-moveit` | 连 MoveIt / ros2_control 一起停 |
| `./start_peach_grasp.sh help` | 帮助 |

### 14.12 本轮改动记录

按定位到的问题逐条修，都已在真机跑通：

| 问题 | 根因 | 处理 |
|---|---|---|
| 全程 `CONTROL_FAILED`，臂回不来 | 旧的规划节点没退出，两个节点同时向 MoveIt 发轨迹 | 同名节点检测拒绝启动 + `motion_lock` 串行 + 自检扫残留 |
| `auto_execute:=false` 不生效 | launch 没声明该参数，传值被静默丢弃 | launch 补声明并下传 |
| 臂完全不动、日志一行不多 | 触发终端没 `source` / 没设 Domain 40，`service call` 静默失败 | 新增 `trigger` 模式自带环境 |
| `status` 报假绿灯 | `ros2 topic echo` 把警告写到 stdout，按空判定误判成功 | 改按 `position:` 字段判定 |
| 归零位相机朝向不利于找目标 | 原归零位相机不是正视 | STAGE 4 改 hold-up 位 `[0,0,0,0,-90,0]` |
| 前进一下过头 | 前进量等于后退量，夹爪顶到目标 | 新增 `ADVANCE_GAP`，前进量少 2 cm |
| 夹爪按满闭合夹 | 硬编码 `0.85` rad = 48.7° | 改 `GRIPPER_CLOSE_DEG` 参数，默认 42° |
| 拿旧目标点动臂 | 无时效检查 | 自动与手动统一 2 s 时效窗口 |
| Ctrl-C 报 `rcl_shutdown already called` | 重复调用 `rclpy.shutdown()` | 四个节点统一 `if rclpy.ok()` |

标定侧结论：残差 6.4 mm 是 xArm6 正运动学的绝对精度，不是标定 bug，不再压。
证据是同一 TCP 位姿重复采样时视觉噪声比残差小约 20 倍，且四种求解算法
（TSAI / PARK / HORAUD / DANIILIDIS）结果全部落在 6.29–6.30 mm。
不要用神经网络检测框去验证标定精度：检测框自身在跳，这样只会越测越不准。
