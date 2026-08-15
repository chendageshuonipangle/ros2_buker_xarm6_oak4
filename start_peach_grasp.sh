#!/bin/bash
#
# xArm6 + OAK-D-SR 视觉抓取一键启动脚本
#
# 按依赖顺序拉起四层：MoveIt2 真机 -> 手眼标定 TF -> NPU 检测 -> 运动规划。
# 每层都等前一层就绪才继续，避免规划节点先起来收到半成品 TF。
# Ctrl+C 一次退出全部子进程。
#
# 固定 ROS_DOMAIN_ID=40，与 Bunker 底盘 (Domain 20) 隔离。

set -o pipefail

# ==================== 可调参数 ====================
ROBOT_IP="${ROBOT_IP:-192.168.1.242}"
TARGET_LABEL="${TARGET_LABEL:-peach}"
CONFIDENCE="${CONFIDENCE:-0.65}"
# 视野里有多个目标时抓最近的那个。超过这个帧间跳变判为换了目标，重新判稳，
# 避免把两个果子的坐标平均到一起。
TRACK_TOLERANCE_MM="${TRACK_TOLERANCE_MM:-60.0}"
# 检测/深度帧率。USB2 实测上限约 18；接 USB3 后可设 30。
FPS="${FPS:-18.0}"
PREVIEW_FPS="${PREVIEW_FPS:-5.0}"
MIN_DEPTH_MM="${MIN_DEPTH_MM:-150.0}"
MAX_DEPTH_MM="${MAX_DEPTH_MM:-700.0}"
FULLSCREEN="${FULLSCREEN:-true}"
SHOW="${SHOW:-true}"
# 默认关闭自动抓取：先看清目标点，再手动放开。
AUTO_EXECUTE="${AUTO_EXECUTE:-true}"
# 夹爪闭合角度 (deg)。MoveIt 界面 0=全张, 48.7=全闭。
GRIPPER_CLOSE_DEG="${GRIPPER_CLOSE_DEG:-42.0}"
# 拧转摘果：夹住后只转 joint6 把果柄扭断。
TWIST_ENABLE="${TWIST_ENABLE:-true}"
TWIST_DEG="${TWIST_DEG:-45.0}"
TWIST_CYCLES="${TWIST_CYCLES:-2}"
# 夹住后等夹持稳定再拧；负载告知控制器，避免搬运途中误报 C31。
GRIP_SETTLE_S="${GRIP_SETTLE_S:-1.0}"
PAYLOAD_KG="${PAYLOAD_KG:-0.3}"
# 拧转期间放宽碰撞检测(0=关)，拧完还原。拧断果柄本身就是在跟外力顶牛，
# 控制器会把这股关节电流判成碰撞 (C31) 并停用轨迹控制器。
TWIST_COLLISION_SENSITIVITY="${TWIST_COLLISION_SENSITIVITY:-0}"
NORMAL_COLLISION_SENSITIVITY="${NORMAL_COLLISION_SENSITIVITY:-3}"
# 带着果子退回/回家时的速度与加速度比例，降下来避免电流尖峰。
LOADED_VELOCITY_SCALE="${LOADED_VELOCITY_SCALE:-0.15}"
LOADED_ACCELERATION_SCALE="${LOADED_ACCELERATION_SCALE:-0.08}"
# 果子可信范围 X 上限；臂停在 target.x-0.27 处，所以这里比臂自身可达更远。
TARGET_X_MAX="${TARGET_X_MAX:-0.95}"
# TCP 自身 X 上限，约束 Point A/B，这条才是真正守可达性的。
ARM_X_MAX="${ARM_X_MAX:-0.80}"
# PTP 到位容差；MoveIt 报成功不等于真到了（C31 中断轨迹时臂会停在半路）。
POSE_REACH_TOLERANCE_M="${POSE_REACH_TOLERANCE_M:-0.02}"
# Point A 实际落点相对果子的横向偏离上限，超了前进就是抓空。
LATERAL_ERROR_MAX_M="${LATERAL_ERROR_MAX_M:-0.05}"
# 空闲且不在 hold-up 位多久后自动回家，防止歪着卡死看不到工作区。0 关闭。
IDLE_HOME_TIMEOUT_S="${IDLE_HOME_TIMEOUT_S:-20.0}"
# ==================================================

export ROS_DOMAIN_ID=40
WS="$HOME/ros2_ws"
LOG_DIR="$WS/log/peach_grasp_$(date +%Y%m%d_%H%M%S)"
PIDS=()
# 装到 install 下的 xArm 服务开关文件；源码在 src/armtodeprition/config/。
XARM_EXTRA_API_PARAMS="${XARM_EXTRA_API_PARAMS:-$WS/install/armtodeprition/share/armtodeprition/config/xarm_extra_api_params.yaml}"

usage() {
    cat <<'EOF'
用法: ./start_peach_grasp.sh [模式]

模式:
  all       (默认) 一键启动全部四层
  vision    只启动 TF + 检测，不启动 MoveIt 和规划（安全，臂不会动）
  check     只做环境自检，不启动任何节点
  trigger   触发一次抓取（自动 source + Domain 40，省去手工配环境）
  recover   机械臂报错后清错误并重新激活控制器
  status    查看目标点是否真的在发布、服务是否在线
  stop      停掉视觉与规划节点（MoveIt 保留）
  stop --with-moveit
            连 MoveIt / ros2_control 一起停（不影响底盘 Domain 20）
  help      显示帮助

常用环境变量:
  AUTO_EXECUTE=false  关闭自动抓取，改为手动 trigger（默认 true 自动）
  TARGET_LABEL=peach  目标类别
  FPS=30.0            检测帧率（换 USB3 后可用）
  SHOW=false          不开预览窗口
  CONFIDENCE=0.9      提高置信度阈值（默认 0.65）
  TRACK_TOLERANCE_MM=80
                      多目标时判定"还是同一个目标"的帧间跳变上限 mm（默认 60）
  GRIPPER_CLOSE_DEG=45 夹爪闭合角度 deg（默认 42，满闭合 48.7）
  TWIST_DEG=60        joint6 拧转幅度 deg（默认 45）
  TWIST_CYCLES=3      拧转轮数（默认 2）
  TWIST_ENABLE=false  关闭拧转
  GRIP_SETTLE_S=1.5   夹住后等待多久再拧转（默认 1.0s）
  PAYLOAD_KG=0.5      果子重量 kg（默认 0.3，用于力矩补偿）
  NORMAL_COLLISION_SENSITIVITY=4
                      平时的碰撞灵敏度 0~5（默认 3，越大越敏感）
  TWIST_COLLISION_SENSITIVITY=1
                      拧转期间的灵敏度（默认 0=关闭，拧完自动还原）
  LOADED_VELOCITY_SCALE=0.2
                      带果子回程速度比例（默认 0.15）
  TARGET_X_MAX=1.0    果子 X 上限 m（默认 0.95，臂停在 target.x-0.27）
  ARM_X_MAX=0.85      TCP 自身 X 上限 m（默认 0.80，约束 Point A/B）
  POSE_REACH_TOLERANCE_M=0.03
                      PTP 到位容差 m（默认 0.02，超差判未到位并回家）
  LATERAL_ERROR_MAX_M=0.08
                      Point A 横向偏离果子的上限 m（默认 0.05）
  IDLE_HOME_TIMEOUT_S=0
                      关闭空闲自动回家（默认 20s 后回 hold-up）

示例:
  ./start_peach_grasp.sh                    # 全部启动，手动触发抓取
  ./start_peach_grasp.sh vision             # 只看检测，臂不动
  ./start_peach_grasp.sh stop --with-moveit # 彻底清干净再重启
  AUTO_EXECUTE=false ./start_peach_grasp.sh # 关掉自动，改手动 trigger
  FPS=30.0 ./start_peach_grasp.sh           # USB3 下跑满帧率

手动触发一次抓取（另开终端，推荐用这个而不是手敲 service call）:
  ./start_peach_grasp.sh trigger
EOF
}

log() { echo -e "\033[1;36m[$(date +%H:%M:%S)]\033[0m $*"; }
ok()  { echo -e "\033[1;32m  ✓\033[0m $*"; }
bad() { echo -e "\033[1;31m  ✗\033[0m $*"; }
warn(){ echo -e "\033[1;33m  !\033[0m $*"; }

cleanup() {
    echo ""
    log "正在关闭所有子进程..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -INT "$pid" 2>/dev/null
        fi
    done
    # 给 MoveIt 和 depthai 时间干净收尾
    sleep 3
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -TERM "$pid" 2>/dev/null
        fi
    done
    log "日志保存在 $LOG_DIR"
    exit 0
}
trap cleanup INT TERM

# 等某个 ROS 节点出现
wait_for_node() {
    local pattern="$1" timeout="${2:-40}" waited=0
    while [ "$waited" -lt "$timeout" ]; do
        if ros2 node list 2>/dev/null | grep -q "$pattern"; then
            return 0
        fi
        sleep 2
        waited=$((waited + 2))
        printf "."
    done
    return 1
}

# 等某个 TF 变换可用
wait_for_tf() {
    local parent="$1" child="$2" timeout="${3:-20}"
    if timeout "$timeout" ros2 run tf2_ros tf2_echo "$parent" "$child" 2>/dev/null \
        | grep -q "Translation"; then
        return 0
    fi
    return 1
}

preflight() {
    log "环境自检"
    local fail=0

    if [ ! -d "$WS/install" ]; then
        bad "找不到 $WS/install，请先 colcon build"
        return 1
    fi
    ok "工作区 $WS"

    # OAK 相机与 USB 速率
    local usb_info
    usb_info=$(python3 - <<'PY' 2>/dev/null
import depthai as dai
try:
    devices = dai.Device.getAllAvailableDevices()
    if not devices:
        print("NONE")
    else:
        with dai.Device() as dev:
            print(dev.getUsbSpeed().name)
except Exception as exc:
    print(f"ERR {exc}")
PY
)
    case "$usb_info" in
        NONE)
            bad "没检测到 OAK 相机，检查 USB 连接"
            fail=1
            ;;
        SUPER*)
            ok "OAK 已连接，USB $usb_info（可跑满帧率）"
            ;;
        HIGH)
            ok "OAK 已连接，USB HIGH (USB2, 480Mbps)"
            warn "USB2 检测上限约 18fps。插到 USB3 口可达 30fps"
            ;;
        ERR*)
            bad "OAK 访问失败: $usb_info"
            warn "相机可能被其他程序占用，先关掉其他 depthai 程序"
            fail=1
            ;;
        *)
            warn "OAK 状态未知: $usb_info"
            ;;
    esac

    # 模型文件
    local model="$WS/src/oak_yolo_py/oak_yolo_py/best_ckpt.rvc2.tar.xz"
    if [ -f "$model" ]; then
        ok "检测模型存在"
    else
        bad "找不到模型 $model"
        fail=1
    fi

    # 手眼标定结果
    local calib="$WS/install/xarm_oak_handeye/share/xarm_oak_handeye/config/eye_in_hand_result.json"
    if [ -f "$calib" ]; then
        ok "手眼标定外参已安装"
    else
        bad "找不到标定结果，请先 colcon build xarm_oak_handeye"
        fail=1
    fi

    # xArm 服务开关文件：缺了它 set_tcp_load / set_collision_sensitivity
    # 压根不会被创建，负载申报永远"服务不可用"，搬运途中必跳 C31。
    if [ -f "$XARM_EXTRA_API_PARAMS" ]; then
        ok "xArm 服务开关配置已安装"
    else
        bad "找不到 $XARM_EXTRA_API_PARAMS"
        warn "请先 colcon build --packages-select armtodeprition"
        fail=1
    fi

    # 残留进程检查：两个规划节点同时发轨迹会让 MoveIt 全程 CONTROL_FAILED
    local stale
    stale=$(pgrep -f "motion_planner_node" | tr '\n' ' ')
    if [ -n "$stale" ]; then
        bad "已有 motion_planner_node 在运行 (pid: $stale)"
        warn "两个规划节点会抢机械臂，MoveIt 会一直报 CONTROL_FAILED"
        warn "先执行: ./start_peach_grasp.sh stop"
        fail=1
    else
        ok "无残留规划节点"
    fi

    stale=$(pgrep -f "oak_peach_detector" | tr '\n' ' ')
    if [ -n "$stale" ]; then
        bad "已有 oak_peach_detector 在运行 (pid: $stale)，会占住相机"
        warn "先执行: ./start_peach_grasp.sh stop"
        fail=1
    else
        ok "无残留检测节点"
    fi

    if [ "$MODE" != "vision" ]; then
        stale=$(pgrep -f "moveit_ros_move_group/move_group" | tr '\n' ' ')
        if [ -n "$stale" ]; then
            bad "已有 move_group 在运行 (pid: $stale)"
            warn "先关掉旧的 MoveIt 终端，或执行: ./start_peach_grasp.sh stop --with-moveit"
            fail=1
        else
            ok "无残留 move_group"
        fi
    fi

    # 机械臂网络可达
    if ping -c 1 -W 2 "$ROBOT_IP" >/dev/null 2>&1; then
        ok "机械臂 $ROBOT_IP 网络可达"
    else
        warn "机械臂 $ROBOT_IP ping 不通（只跑 vision 模式可忽略）"
    fi

    return $fail
}

start_moveit() {
    log "[1/4] 启动 xArm6 MoveIt2 真机控制 (robot_ip=$ROBOT_IP)"
    # extra_robot_api_params_path 打开 xarm_api 默认关闭的服务
    # (set_tcp_load / set_collision_sensitivity)。不传这个文件，负载申报
    # 会一直报"服务不可用"，控制器按零负载前馈力矩，搬运途中必跳 C31。
    # ros2 launch 拒绝空值 (malformed launch argument)，所以文件不存在时
    # 必须整个参数都不传，而不是传空字符串。
    local extra_args=()
    if [ -f "$XARM_EXTRA_API_PARAMS" ]; then
        extra_args+=("extra_robot_api_params_path:=$XARM_EXTRA_API_PARAMS")
        echo "      xArm 服务开关: $XARM_EXTRA_API_PARAMS"
    else
        warn "找不到 $XARM_EXTRA_API_PARAMS，负载申报与碰撞灵敏度将不可用"
    fi
    ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py \
        robot_ip:="$ROBOT_IP" \
        add_gripper:=true \
        add_arc_gripper:=true \
        add_oak_d_sr:=true \
        "${extra_args[@]}" \
        > "$LOG_DIR/1_moveit.log" 2>&1 &
    PIDS+=($!)

    printf "      等待 move_group 就绪 "
    if wait_for_node "/move_group" 60; then
        echo ""
        ok "MoveIt2 已就绪"
    else
        echo ""
        bad "MoveIt2 启动超时，见 $LOG_DIR/1_moveit.log"
        return 1
    fi
}

start_vision() {
    log "[2/4] 启动手眼标定 TF + NPU 检测 (label=$TARGET_LABEL, conf=$CONFIDENCE)"
    echo "      检测 ${FPS}fps / 预览 ${PREVIEW_FPS}fps / 深度窗口 ${MIN_DEPTH_MM}-${MAX_DEPTH_MM}mm"
    echo "      多目标: 抓最近的那个 (跳变 >${TRACK_TOLERANCE_MM}mm 视为换目标，重新判稳)"
    ros2 launch xarm_oak_handeye handeye_target_transform.launch.py \
        target_label:="$TARGET_LABEL" \
        confidence:="$CONFIDENCE" \
        track_tolerance_mm:="$TRACK_TOLERANCE_MM" \
        fps:="$FPS" \
        preview_fps:="$PREVIEW_FPS" \
        min_depth_mm:="$MIN_DEPTH_MM" \
        max_depth_mm:="$MAX_DEPTH_MM" \
        fullscreen:="$FULLSCREEN" \
        show:="$SHOW" \
        > "$LOG_DIR/2_vision.log" 2>&1 &
    PIDS+=($!)

    printf "      等待检测节点就绪 "
    if wait_for_node "oak_peach_detector" 40; then
        echo ""
        ok "检测节点已就绪，发布 /oak_result"
    else
        echo ""
        bad "检测节点启动超时，见 $LOG_DIR/2_vision.log"
        return 1
    fi

    if wait_for_tf link_tcp oak_rgb_camera_optical_frame 10; then
        ok "手眼标定 TF 已发布 (link_tcp -> oak_rgb_camera_optical_frame)"
    else
        warn "TF 暂未收到，若只跑 vision 模式属正常（缺 MoveIt 的 link_tcp）"
    fi
}

start_planner() {
    log "[3/4] 启动运动规划节点 (auto_execute=$AUTO_EXECUTE)"
    if [ "$AUTO_EXECUTE" = "true" ]; then
        warn "自动抓取已开启：检测到稳定目标后会自动执行，手要离开工作区"
    else
        ok "自动抓取关闭，需手动调用 /execute_grasp 触发"
    fi
    echo "      限位: 果子 X≤${TARGET_X_MAX}m / TCP X≤${ARM_X_MAX}m"
    if [ "$TWIST_ENABLE" = "true" ]; then
        echo "      拧转摘果: joint6 ±${TWIST_DEG}° × ${TWIST_CYCLES} 轮 (夹持稳定 ${GRIP_SETTLE_S}s 后开始)"
    else
        echo "      拧转摘果: 已关闭"
    fi
    ros2 launch armtodeprition motion_planner.launch.py \
        auto_execute:="$AUTO_EXECUTE" \
        gripper_close_deg:="$GRIPPER_CLOSE_DEG" \
        twist_enable:="$TWIST_ENABLE" \
        twist_deg:="$TWIST_DEG" \
        twist_cycles:="$TWIST_CYCLES" \
        grip_settle_s:="$GRIP_SETTLE_S" \
        payload_kg:="$PAYLOAD_KG" \
        twist_collision_sensitivity:="$TWIST_COLLISION_SENSITIVITY" \
        normal_collision_sensitivity:="$NORMAL_COLLISION_SENSITIVITY" \
        loaded_velocity_scale:="$LOADED_VELOCITY_SCALE" \
        loaded_acceleration_scale:="$LOADED_ACCELERATION_SCALE" \
        target_x_max:="$TARGET_X_MAX" \
        arm_x_max:="$ARM_X_MAX" \
        pose_reach_tolerance_m:="$POSE_REACH_TOLERANCE_M" \
        lateral_error_max_m:="$LATERAL_ERROR_MAX_M" \
        idle_home_timeout_s:="$IDLE_HOME_TIMEOUT_S" \
        > "$LOG_DIR/3_planner.log" 2>&1 &
    PIDS+=($!)

    printf "      等待规划节点就绪 "
    if wait_for_node "arm_motion_planner_node" 30; then
        echo ""
        ok "规划节点已就绪"
    else
        echo ""
        bad "规划节点启动超时，见 $LOG_DIR/3_planner.log"
        return 1
    fi
}

monitor() {
    log "[4/4] 系统就绪"
    echo ""
    echo "=========================================="
    echo "  实时状态 (Ctrl+C 退出全部)"
    echo "=========================================="
    echo ""
    echo "预览窗口: 绿框=目标稳定可抓, 橙框=抖动中(已拦下), 灰框=看见但不是最近的"
    echo "          按 f 切换全屏, 按 q 关预览(检测继续)"
    echo ""
    if [ "$AUTO_EXECUTE" = "true" ]; then
        echo "自动抓取已开启：检测到稳定目标即自动执行，手要离开工作区"
        echo "临时关闭: AUTO_EXECUTE=false ./start_peach_grasp.sh"
        echo ""
    else
        echo "手动触发抓取（另开终端）:"
        echo "  cd ~/ros2_ws && ./start_peach_grasp.sh trigger"
        echo ""
        echo "确认目标点真的在发布:"
        echo "  cd ~/ros2_ws && ./start_peach_grasp.sh status"
        echo ""
    fi
    echo "查看目标点坐标:"
    echo "  ros2 topic echo /target_pose_in_base"
    echo ""
    echo "日志目录: $LOG_DIR"
    echo "------------------------------------------"

    # 跟随规划节点日志，抓取过程一目了然
    tail -f "$LOG_DIR/3_planner.log" 2>/dev/null &
    PIDS+=($!)

    while true; do
        sleep 5
        # 任一关键子进程退出就报警
        for pid in "${PIDS[@]}"; do
            if ! kill -0 "$pid" 2>/dev/null; then
                warn "有子进程已退出，检查 $LOG_DIR"
                sleep 2
                cleanup
            fi
        done
    done
}

stop_all() {
    log "停止相关节点"
    local patterns=(
        "oak_peach_detector"
        "motion_planner_node"
        "handeye_target_transform"
        "publish_handeye_tf"
        "target_to_base"
    )
    if [ "$1" = "--with-moveit" ]; then
        # 只匹配 xArm 的 MoveIt 栈，不会碰底盘 (Domain 20) 的节点
        patterns+=(
            "moveit_ros_move_group/move_group"
            "xarm6_moveit_realmove"
            "ros2_control_node"
        )
        warn "同时停止 MoveIt / ros2_control"
    fi

    for pat in "${patterns[@]}"; do
        pkill -INT -f "$pat" 2>/dev/null
    done
    sleep 3

    # verify: 残留的规划节点是下次 CONTROL_FAILED 的根源，必须确认真的没了
    local left=0
    for pat in "${patterns[@]}"; do
        local pids
        pids=$(pgrep -f "$pat" | tr '\n' ' ')
        if [ -n "$pids" ]; then
            warn "$pat 仍在运行 (pid: $pids)，升级为 SIGTERM"
            pkill -TERM -f "$pat" 2>/dev/null
            left=1
        fi
    done
    if [ "$left" = "1" ]; then
        sleep 2
        for pat in "${patterns[@]}"; do
            if pgrep -f "$pat" >/dev/null 2>&1; then
                bad "$pat 仍未退出，需手动 kill -9"
            fi
        done
    fi
    ok "已停止"
    if [ "$1" != "--with-moveit" ]; then
        echo "      MoveIt 未动。要一起停: ./start_peach_grasp.sh stop --with-moveit"
    fi
}

show_status() {
    log "链路状态自查"

    if ros2 node list 2>/dev/null | grep -q "oak_peach_detector"; then
        ok "检测节点在线"
    else
        bad "检测节点不在线"
    fi
    if ros2 node list 2>/dev/null | grep -q "arm_motion_planner_node"; then
        ok "规划节点在线"
    else
        bad "规划节点不在线"
    fi
    if ros2 service list 2>/dev/null | grep -q "/execute_grasp"; then
        ok "/execute_grasp 服务在线"
    else
        bad "/execute_grasp 服务不在线"
    fi

    # 这两个是 C31 的关键：负载不申报、灵敏度不能放宽，就只能靠事后恢复。
    local svc
    svc=$(ros2 service list 2>/dev/null)
    if echo "$svc" | grep -q "/xarm/set_tcp_load"; then
        ok "/xarm/set_tcp_load 在线（负载可申报）"
    else
        bad "/xarm/set_tcp_load 不在线：MoveIt 启动时没带 extra_robot_api_params_path"
        warn "重启：./start_peach_grasp.sh stop --with-moveit && ./start_peach_grasp.sh"
    fi
    if echo "$svc" | grep -q "/xarm/set_collision_sensitivity"; then
        ok "/xarm/set_collision_sensitivity 在线（拧转可放宽碰撞检测）"
    else
        bad "/xarm/set_collision_sensitivity 不在线，拧转仍会跳 C31"
    fi

    echo ""
    log "抓取 3s 内 /target_pose_in_base 的实际数据"
    local sample
    # ros2 topic echo 把 "does not appear to be published yet" 警告打到 stdout，
    # 直接判空会误报成功，必须按真实字段 position 判定。
    sample=$(timeout 5 ros2 topic echo /target_pose_in_base --once 2>/dev/null \
        | grep -v "^WARNING")
    if echo "$sample" | grep -q "position:"; then
        ok "目标点正在发布 (link_base 坐标, 单位 m)"
        echo "$sample" | sed -n '/position/,/orientation/p' | sed 's/^/      /'
    else
        bad "5s 内收不到 /target_pose_in_base"
        warn "检查：目标是否在画面内且显示 TARGET STABLE；TF link_tcp->camera 是否发布"
    fi
}

trigger_grasp() {
    log "触发一次抓取"
    if ! ros2 service list 2>/dev/null | grep -q "/execute_grasp"; then
        bad "/execute_grasp 不在线，规划节点没起来或 Domain 不对"
        return 1
    fi
    # 手敲 service call 常见的坑是新终端没 source、没设 Domain 40，
    # 结果服务名解析不到、命令静默失败，看起来像"机械臂不动"。
    local out
    out=$(ros2 service call /execute_grasp std_srvs/srv/Trigger 2>&1)
    echo "$out" | sed 's/^/      /'
    if echo "$out" | grep -q "success=True"; then
        ok "已触发，抓取过程看主终端日志"
    else
        bad "触发未成功"
        warn "若提示「没有可用目标点」：画面必须是 TARGET STABLE，且点要在限位内"
    fi
}

MODE="${1:-all}"

case "$MODE" in
    help|-h|--help)
        usage
        exit 0
        ;;
    stop)
        stop_all "$2"
        exit 0
        ;;
esac

echo "=========================================="
echo "  xArm6 + OAK-D-SR 视觉抓取系统"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID   模式: $MODE"
echo "=========================================="
echo ""

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
mkdir -p "$LOG_DIR"

case "$MODE" in
    check)
        preflight
        exit $?
        ;;
    status)
        show_status
        exit $?
        ;;
    trigger)
        trigger_grasp
        exit $?
        ;;
    recover)
        log "清除机械臂错误并重新激活控制器"
        if ! ros2 service list 2>/dev/null | grep -q "/recover_arm"; then
            bad "/recover_arm 不在线，规划节点没起来"
            exit 1
        fi
        out=$(ros2 service call /recover_arm std_srvs/srv/Trigger 2>&1)
        echo "$out" | sed 's/^/      /'
        echo "$out" | grep -q "success=True" && ok "已恢复" || bad "恢复失败，见文档 14.10 节"
        exit 0
        ;;
    vision)
        preflight || warn "自检有问题，仍继续（vision 模式不动机械臂）"
        echo ""
        start_vision || cleanup
        echo ""
        log "vision 模式就绪，机械臂不会动。Ctrl+C 退出"
        tail -f "$LOG_DIR/2_vision.log" 2>/dev/null &
        PIDS+=($!)
        wait
        ;;
    all)
        preflight || { bad "自检未通过，中止启动"; exit 1; }
        echo ""
        start_moveit || cleanup
        echo ""
        start_vision || cleanup
        echo ""
        start_planner || cleanup
        echo ""
        monitor
        ;;
    *)
        bad "未知模式: $MODE"
        echo ""
        usage
        exit 1
        ;;
esac
