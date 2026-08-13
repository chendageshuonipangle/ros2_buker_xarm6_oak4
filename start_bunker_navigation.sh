#!/bin/bash

# Bunker Mini + RPLIDAR A1 导航系统启动脚本
# 支持三种模式: bringup (硬件), slam (建图), navigation (导航), slam_nav (边建图边导航)

MODE=${1:-"help"}
MAP_PATH=${2:-"/home/qluo/ros2_ws/maps/my_map.yaml"}

# 底盘导航系统固定使用 ROS_DOMAIN_ID=20，避免与机械臂/OAK/RViz2 节点互相干扰。
export ROS_DOMAIN_ID=20

echo "=========================================="
echo "  Bunker Mini 导航系统启动脚本"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "=========================================="
echo ""

# Source ROS2 workspace
source ~/ros2_ws/install/setup.bash

show_help() {
    echo "用法: ./start_bunker_navigation.sh <模式> [地图路径]"
    echo ""
    echo "模式:"
    echo "  bringup          - 只启动机器人硬件 (底盘、雷达、TF，含 IMU)"
    echo "  bringup_with_imu - 只启动机器人硬件，强制启用 YESENSE IMU"
    echo "  bringup_no_imu   - 只启动机器人硬件，禁用 YESENSE IMU (对比用)"
    echo "  bringup_no_imu_lite - 不带 IMU，AMCL 粒子数减半 (max=1500, min=400)"
    echo "  bringup_no_imu_n10  - 只启动机器人硬件，使用 LSLIDAR N10 雷达，禁用 IMU"
    echo "  slam         - 只启动 SLAM 建图"
    echo "  nav          - 只启动导航 (需要已有地图)"
    echo "  nav_lite     - 导航，AMCL 粒子数减半 (max=1500)"
    echo "  nav_origin   - 导航，弱化 AMCL 参数 (Origin 版，用于对比实验)"
    echo "  slam_nav     - SLAM + 导航同时运行 (边建图边导航)"
    echo "  jps          - 启动 JPS 原始版路径规划节点"
    echo "  jps_improved - 启动 JPS 改进版路径规划节点 (带密度启发)"
    echo "  astar_cost   - 启动 A* 代价感知路径规划节点 (远离障碍物)"
    echo "  explore      - 实验性自动探索建图节点 (安全保守版)"
    echo "  save_map     - 保存当前地图"
    echo ""
    echo "示例:"
    echo "  ./start_bunker_navigation.sh bringup"
    echo "  ./start_bunker_navigation.sh bringup_with_imu"
    echo "  ./start_bunker_navigation.sh bringup_no_imu"
    echo "  ./start_bunker_navigation.sh bringup_no_imu_lite"
    echo "  ./start_bunker_navigation.sh bringup_no_imu_n10"
    echo "  ./start_bunker_navigation.sh slam"
    echo "  ./start_bunker_navigation.sh nav /home/qluo/ros2_ws/maps/my_map.yaml"
    echo "  ./start_bunker_navigation.sh slam_nav"
    echo "  ./start_bunker_navigation.sh jps"
    echo "  ./start_bunker_navigation.sh jps_improved"
    echo "  ./start_bunker_navigation.sh explore"
    echo "  ./start_bunker_navigation.sh save_map"
    echo ""
    echo "使用自定义 JPS 算法导航:"
    echo "  1. 终端1: ./start_bunker_navigation.sh bringup"
    echo "  2. 终端2: ./start_bunker_navigation.sh slam_nav"
    echo "  3. 终端3: ./start_bunker_navigation.sh jps          # 原始 JPS"
    echo "     或者:  ./start_bunker_navigation.sh jps_improved # 改进 JPS"
    echo "  4. 在 RViz 中使用 '2D Goal Pose' 设置目标点"
    echo ""
}

init_can() {
    echo "正在初始化 CAN 接口..."
    cd ~/ros2_ws/src/ugv_sdk/scripts/
    bash bringup_can2usb_500k.bash
    cd ~/ros2_ws
    echo ""
}

check_can() {
    echo "检查 CAN 接口..."
    if ip link show can0 &> /dev/null; then
        echo "✓ CAN 接口 can0 已就绪"
        return 0
    else
        echo "✗ CAN 接口 can0 未找到"
        return 1
    fi
}

ask_init_can() {
    check_can
    if [ $? -eq 0 ]; then
        echo ""
        read -p "是否重新初始化 CAN 接口？(y/N): " -n 1 -r
        echo ""
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            init_can
        fi
    else
        echo ""
        read -p "CAN 接口未就绪，是否初始化？(Y/n): " -n 1 -r
        echo ""
        if [[ ! $REPLY =~ ^[Nn]$ ]]; then
            init_can
        else
            echo "跳过 CAN 初始化，可能无法启动底盘"
            echo ""
        fi
    fi
}

case $MODE in
    "bringup")
        ask_init_can
        echo ""
        echo "[BRINGUP] 启动机器人硬件 (含 YESENSE IMU)..."
        echo "按 Ctrl+C 停止"
        echo ""
        ros2 launch linorobot2_bringup bringup.launch.py \
            use_bunker:=true \
            is_bunker_mini:=true \
            use_yesense_imu:=true \
            port_name:=can0 2>&1 | while IFS= read -r line; do
            echo "$line"
            if [[ "$line" == *"Detected protocol: UNKONWN"* ]] || [[ "$line" == *"bunker_base_node"*"has died"* ]]; then
                echo ""
                echo "========================================"
                echo "⚠️  CAN 通信错误！"
                echo "请重新插拔 CAN 总线，然后重新运行此脚本"
                echo "========================================"
                echo ""
            fi
        done
        ;;

    "bringup_with_imu")
        ask_init_can
        echo ""
        echo "[WITH_IMU] 启动机器人硬件 (强制启用 YESENSE IMU，用于对比数据采集)..."
        echo "按 Ctrl+C 停止"
        echo ""
        ros2 launch linorobot2_bringup bringup.launch.py \
            use_bunker:=true \
            is_bunker_mini:=true \
            use_yesense_imu:=true \
            port_name:=can0 2>&1 | while IFS= read -r line; do
            echo "[WITH_IMU] $line"
            if [[ "$line" == *"Detected protocol: UNKONWN"* ]] || [[ "$line" == *"bunker_base_node"*"has died"* ]]; then
                echo ""
                echo "========================================"
                echo "⚠️  CAN 通信错误！"
                echo "请重新插拔 CAN 总线，然后重新运行此脚本"
                echo "========================================"
                echo ""
            fi
        done
        ;;

    "bringup_no_imu")
        ask_init_can
        echo ""
        echo "[NO_IMU] 启动机器人硬件 (禁用 YESENSE IMU，用于对比数据采集)..."
        echo "按 Ctrl+C 停止"
        echo ""
        ros2 launch linorobot2_bringup bringup.launch.py \
            use_bunker:=true \
            is_bunker_mini:=true \
            use_yesense_imu:=false \
            port_name:=can0 2>&1 | while IFS= read -r line; do
            echo "[NO_IMU] $line"
            if [[ "$line" == *"Detected protocol: UNKONWN"* ]] || [[ "$line" == *"bunker_base_node"*"has died"* ]]; then
                echo ""
                echo "========================================"
                echo "⚠️  CAN 通信错误！"
                echo "请重新插拔 CAN 总线，然后重新运行此脚本"
                echo "========================================"
                echo ""
            fi
        done
        ;;

    "bringup_no_imu_lite")
        ask_init_can
        echo ""
        echo "[NO_IMU_LITE] 启动机器人硬件 (禁用 IMU，AMCL 粒子数减半 max=1500 min=400)..."
        echo "导航时使用: ./start_bunker_navigation.sh nav_lite"
        echo "按 Ctrl+C 停止"
        echo ""
        ros2 launch linorobot2_bringup bringup.launch.py \
            use_bunker:=true \
            is_bunker_mini:=true \
            use_yesense_imu:=false \
            port_name:=can0 2>&1 | while IFS= read -r line; do
            echo "[NO_IMU_LITE] $line"
            if [[ "$line" == *"Detected protocol: UNKONWN"* ]] || [[ "$line" == *"bunker_base_node"*"has died"* ]]; then
                echo ""
                echo "========================================"
                echo "⚠️  CAN 通信错误！"
                echo "请重新插拔 CAN 总线，然后重新运行此脚本"
                echo "========================================"
                echo ""
            fi
        done
        ;;

    "bringup_no_imu_n10")
        ask_init_can
        echo ""
        echo "[N10] 启动机器人硬件 (LSLIDAR N10 雷达，禁用 YESENSE IMU)..."
        echo "按 Ctrl+C 停止"
        echo ""
        ros2 launch linorobot2_bringup bringup.launch.py \
            use_bunker:=true \
            is_bunker_mini:=true \
            use_yesense_imu:=false \
            use_lslidar_n10:=true \
            port_name:=can0 2>&1 | while IFS= read -r line; do
            echo "[N10] $line"
            if [[ "$line" == *"Detected protocol: UNKONWN"* ]] || [[ "$line" == *"bunker_base_node"*"has died"* ]]; then
                echo ""
                echo "========================================"
                echo "⚠️  CAN 通信错误！"
                echo "请重新插拔 CAN 总线，然后重新运行此脚本"
                echo "========================================"
                echo ""
            fi
        done
        ;;
    
    "slam")
        echo "启动 SLAM 建图..."
        echo "按 Ctrl+C 停止"
        echo ""
        ros2 launch linorobot2_navigation slam.launch.py
        ;;
    
    "nav")
        echo "启动导航..."
        echo "地图: $MAP_PATH"
        echo "按 Ctrl+C 停止"
        echo ""
        ros2 launch linorobot2_navigation navigation.launch.py \
            map:=$MAP_PATH
        ;;

    "nav_lite")
        echo "[LITE] 启动导航 (AMCL 粒子数减半: max=1500, min=400)..."
        echo "地图: $MAP_PATH"
        echo "按 Ctrl+C 停止"
        echo ""
        ros2 launch linorobot2_navigation navigation.launch.py \
            map:=$MAP_PATH \
            params_file:=$(ros2 pkg prefix linorobot2_navigation)/share/linorobot2_navigation/config/navigation_lite.yaml
        ;;

    "nav_origin")
        echo "[ORIGIN] 启动导航 (弱化 AMCL，用于与 improved 对比)..."
        echo "地图: $MAP_PATH"
        echo "按 Ctrl+C 停止"
        echo ""
        ros2 launch linorobot2_navigation navigation.launch.py \
            map:=$MAP_PATH \
            params_file:=$(ros2 pkg prefix linorobot2_navigation)/share/linorobot2_navigation/config/navigation_origin.yaml
        ;;
    
    "slam_nav")
        echo "启动 SLAM + 导航 (边建图边导航)..."
        echo "按 Ctrl+C 停止"
        echo ""
        ros2 launch linorobot2_navigation slam_navigation.launch.py
        ;;
    
    "jps")
        echo "启动 JPS 原始版路径规划节点..."
        echo "在 RViz 中使用 '2D Goal Pose' 设置目标点"
        echo "按 Ctrl+C 停止"
        echo ""
        ros2 run path_planning jps_planner_node --ros-args -p algorithm:=jps
        ;;
    
    "jps_improved")
        echo "启动 JPS 改进版路径规划节点..."
        echo "在 RViz 中使用 '2D Goal Pose' 设置目标点"
        echo "按 Ctrl+C 停止"
        echo ""
        ros2 run path_planning jps_planner_node --ros-args -p algorithm:=jps_improved
        ;;
    
    "astar_cost")
        echo "启动 A* 代价感知路径规划节点..."
        echo "路径会远离高代价区域（红色膨胀区域）"
        echo "在 RViz 中使用 '2D Goal Pose' 设置目标点"
        echo "按 Ctrl+C 停止"
        echo ""
        ros2 run path_planning jps_planner_node --ros-args -p algorithm:=astar_costmap
        ;;

    "explore")
        echo "启动实验性自动探索建图节点 (安全保守版)..."
        echo "请先启动: ./start_bunker_navigation.sh bringup"
        echo "再启动:   ./start_bunker_navigation.sh slam_nav"
        echo "该节点只发送 Nav2 目标点，不直接控制 /cmd_vel"
        echo "按 Ctrl+C 停止"
        echo ""
        ros2 run path_planning frontier_explorer
        ;;
    
    "save_map")
        echo "保存地图到 ~/ros2_ws/maps/my_map..."
        ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/my_map
        echo "地图已保存!"
        ;;
    
    "help"|*)
        show_help
        ;;
esac
