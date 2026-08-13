#!/bin/bash

# xArm6 + OAK + 视觉抓取系统启动脚本
# 固定使用 ROS_DOMAIN_ID=40，避免与 Bunker/Nav2/RViz2 节点互相干扰。

MODE=${1:-"help"}
TARGET_LABEL=${2:-"orange"}
OAK_SHOW_WINDOW=${OAK_SHOW_WINDOW:-true}

export ROS_DOMAIN_ID=40

echo "=========================================="
echo "  xArm6 + OAK 视觉抓取系统启动脚本"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "=========================================="
echo ""

# Source ROS2 workspace
source ~/ros2_ws/install/setup.bash

show_help() {
    echo "用法: ./start_arm_system.sh <模式> [target_label]"
    echo ""
    echo "模式:"
    echo "  moveit   - 启动 xArm6 MoveIt2 真机控制"
    echo "  oak      - 启动 OAK YOLO 检测节点（板载RVC4推理，默认显示画面）"
    echo "  tf       - 启动 OAK 到机械臂坐标转换"
    echo "  planner  - 启动运动规划节点（全自动模式）"
    echo "  help     - 显示帮助"
    echo ""
    echo "参数:"
    echo "  target_label - tf 模式使用的目标类别，默认: orange"
    echo ""
    echo "示例:"
    echo "  ./start_arm_system.sh moveit"
    echo "  ./start_arm_system.sh oak"
    echo "  ./start_arm_system.sh tf orange"
    echo "  ./start_arm_system.sh planner"
    echo ""
}

case $MODE in
    "moveit")
        echo "[ARM_MOVEIT] 启动 xArm6 MoveIt2 真机控制..."
        echo "robot_ip: 192.168.1.242"
        echo "add_gripper: true"
        echo "add_arc_gripper: true"
        echo "按 Ctrl+C 停止"
        echo ""
        ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py \
            robot_ip:=192.168.1.242 \
            add_gripper:=true \
            add_arc_gripper:=true \
            add_oak_d_sr:=true
        ;;

    "oak")
        echo "[ARM_OAK] 启动 OAK YOLO 检测节点..."
        echo "show_window: $OAK_SHOW_WINDOW"
        echo "按 Ctrl+C 停止"
        echo ""
        ros2 run oak_yolo_py oak_yolo_node --ros-args -p show_window:=$OAK_SHOW_WINDOW
        ;;

    "tf")
        echo "[ARM_TF] 启动 OAK 到机械臂坐标转换..."
        echo "target_label: $TARGET_LABEL"
        echo "按 Ctrl+C 停止"
        echo ""
        ros2 launch oaktf_trantoarm transform.launch.py \
            target_label:=$TARGET_LABEL
        ;;

    "planner")
        echo "[ARM_PLANNER] 启动运动规划节点（全自动模式）..."
        echo "按 Ctrl+C 停止"
        echo ""
        ros2 run armtodeprition motion_planner_node
        ;;

    "help"|*)
        show_help
        ;;
esac
