#!/bin/bash
# 用法: ./collect_nav_data.sh <session_label> [duration_sec]
# 例子: ./collect_nav_data.sh with_imu 120
#       ./collect_nav_data.sh no_imu 120

SESSION_LABEL="${1:-unknown}"
DURATION="${2:-120}"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
OUTPUT_DIR="${HOME}/ros2_ws/nav_data/${TIMESTAMP}_${SESSION_LABEL}"
DADIAN="${HOME}/ros2_ws/maps/dadian.txt"

echo "=========================================="
echo "  Bunker 导航数据采集"
echo "  会话标签 : ${SESSION_LABEL}"
echo "  采集时长 : ${DURATION} 秒"
echo "  输出目录 : ${OUTPUT_DIR}"
echo "=========================================="

source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

mkdir -p "${OUTPUT_DIR}"

# 写入元信息
{
  echo "session_label=${SESSION_LABEL}"
  echo "start_time=${TIMESTAMP}"
  echo "duration_sec=${DURATION}"
  echo ""
  echo "--- ekf imu0_config ---"
  grep -A 6 "imu0_config" ~/ros2_ws/src/linorobot2/linorobot2_base/config/ekf.yaml 2>/dev/null
  echo ""
  echo "--- yesense gyro_deadband ---"
  grep "gyro_deadband" ~/ros2_ws/src/yesense_ros2/yesense_std_ros2/config/yesense_net_config.yaml 2>/dev/null
  echo ""
  echo "--- runtime ekf imu0_config ---"
  timeout 5 ros2 param get /ekf_filter_node imu0_config 2>/dev/null || echo "(not running)"
  echo ""
  echo "--- runtime yesense deadband ---"
  timeout 5 ros2 param get /yesense_pub gyro_deadband_rad_s 2>/dev/null || echo "(not running)"
} > "${OUTPUT_DIR}/meta.txt"

echo "[meta] meta.txt 已写入"

# 复制路点文件到输出目录（便于溯源）
cp "${DADIAN}" "${OUTPUT_DIR}/dadian.txt" 2>/dev/null || true

# 启动 Python 采集器
python3 - "${OUTPUT_DIR}" "${DURATION}" "${SESSION_LABEL}" "${DADIAN}" << 'PYEOF' &
import sys, os, math, csv, time, threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
try:
    from bunker_msgs.msg import BunkerStatus
    HAS_BUNKER = True
except ImportError:
    HAS_BUNKER = False

OUTPUT_DIR  = sys.argv[1]
DURATION    = float(sys.argv[2])
SESSION     = sys.argv[3]
DADIAN_PATH = sys.argv[4]
ARRIVAL_THRESHOLD = 0.5   # metres

# ── 解析 dadian.txt ──────────────────────────────────────────
def parse_waypoints(path):
    """从 ros2 topic echo /clicked_point 格式文本中提取 (x, y) 列表."""
    waypoints = []
    try:
        with open(path) as f:
            lines = f.readlines()
        i = 0
        while i < len(lines):
            if lines[i].strip().startswith('x:'):
                x = float(lines[i].strip().split(':')[1])
                if i + 1 < len(lines) and lines[i+1].strip().startswith('y:'):
                    y = float(lines[i+1].strip().split(':')[1])
                    waypoints.append((x, y))
                i += 2
            else:
                i += 1
    except Exception as e:
        print(f"[WARN] 解析 dadian.txt 失败: {e}", flush=True)
    return waypoints

WAYPOINTS = parse_waypoints(DADIAN_PATH)
print(f"[wp] 已加载 {len(WAYPOINTS)} 个路点，到达阈值 {ARRIVAL_THRESHOLD}m", flush=True)

# ── 工具函数 ─────────────────────────────────────────────────
def stamp_sec(s):
    return s.sec + s.nanosec * 1e-9

def quat_yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

def open_csv(name, hdr):
    f = open(os.path.join(OUTPUT_DIR, name), 'w', newline='')
    w = csv.writer(f)
    w.writerow(hdr)
    return f, w

# ── 主节点 ───────────────────────────────────────────────────
class Col(Node):
    def __init__(self):
        super().__init__('bunker_collector')
        self._t0   = time.time()
        self._lock = threading.Lock()
        self._done = False

        # 路点状态
        self._pending = list(range(len(WAYPOINTS)))   # 未到达路点下标

        tqos = QoSProfile(depth=1,
                          reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sqos = QoSProfile(depth=10)

        # 打开 CSV
        self._fa, self._wa = open_csv('amcl_pose.csv',
            ['stamp','x','y','yaw_deg','cov_xx','cov_yy','cov_tt_deg2'])
        self._fo, self._wo = open_csv('odom.csv',
            ['stamp','x','y','yaw_deg','vx','vy','wz'])
        self._fr, self._wr = open_csv('odom_raw.csv',
            ['stamp','x','y','yaw_deg','vx','vy','wz'])
        self._fi, self._wi = open_csv('imu.csv',
            ['stamp','wx','wy','wz','ax','ay','az','q_x','q_y','q_z','q_w'])
        self._ft, self._wt = open_csv('tf_map_odom.csv',
            ['stamp','tx','ty','tz','rz_deg'])
        self._fwp, self._wwp = open_csv('waypoint_errors.csv',
            ['stamp','wp_idx','wp_x','wp_y',
             'robot_x','robot_y','robot_yaw_deg','dist_error_m'])
        if HAS_BUNKER:
            self._fb, self._wb = open_csv('bunker_status.csv',
                ['stamp','linear_vel','angular_vel','control_mode','battery_v'])

        self._cnt = {k: 0 for k in ('a','o','r','i','t','b','wp')}

        # 订阅
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._ca, tqos)
        self.create_subscription(Odometry, '/odom',              self._co, sqos)
        self.create_subscription(Odometry, '/odom/unfiltered',   self._cr, sqos)
        self.create_subscription(Imu,      '/imu/data',          self._ci, sqos)
        self.create_subscription(TFMessage,'/tf',                self._ct, sqos)
        if HAS_BUNKER:
            self.create_subscription(
                BunkerStatus, '/bunker_status', self._cb, sqos)

        self.create_timer(10.0,           self._prog)
        self.create_timer(DURATION + 1.0, self._end)

    # ── /amcl_pose ──────────────────────────────────────────
    def _ca(self, msg):
        t   = stamp_sec(msg.header.stamp)
        p   = msg.pose.pose.position
        yaw = math.degrees(quat_yaw(msg.pose.pose.orientation))
        c   = msg.pose.covariance
        with self._lock:
            self._wa.writerow([
                f'{t:.6f}', f'{p.x:.6f}', f'{p.y:.6f}', f'{yaw:.4f}',
                f'{c[0]:.8f}', f'{c[7]:.8f}',
                f'{math.degrees(math.sqrt(max(c[35], 0))):.4f}'])
            self._cnt['a'] += 1
            # 路点到达检测
            self._check_waypoints(t, p.x, p.y, yaw)

    def _check_waypoints(self, t, rx, ry, ryaw_deg):
        """在持有 _lock 的情况下调用。"""
        arrived = []
        for idx in self._pending:
            wx, wy = WAYPOINTS[idx]
            dist = math.hypot(rx - wx, ry - wy)
            if dist <= ARRIVAL_THRESHOLD:
                self._wwp.writerow([
                    f'{t:.6f}', idx + 1,
                    f'{wx:.6f}', f'{wy:.6f}',
                    f'{rx:.6f}', f'{ry:.6f}',
                    f'{ryaw_deg:.4f}', f'{dist:.4f}'])
                self._fwp.flush()
                self._cnt['wp'] += 1
                arrived.append(idx)
                print(f"  [WP] #{idx+1:02d} 到达  dist={dist:.3f}m  "
                      f"pos=({rx:.3f},{ry:.3f})", flush=True)
        for idx in arrived:
            self._pending.remove(idx)

    # ── /odom ────────────────────────────────────────────────
    def _wodom(self, w, msg):
        t   = stamp_sec(msg.header.stamp)
        p   = msg.pose.pose.position
        yaw = math.degrees(quat_yaw(msg.pose.pose.orientation))
        v   = msg.twist.twist
        w.writerow([f'{t:.6f}', f'{p.x:.6f}', f'{p.y:.6f}', f'{yaw:.4f}',
                    f'{v.linear.x:.6f}', f'{v.linear.y:.6f}',
                    f'{v.angular.z:.6f}'])

    def _co(self, msg):
        with self._lock:
            self._wodom(self._wo, msg)
            self._cnt['o'] += 1

    def _cr(self, msg):
        with self._lock:
            self._wodom(self._wr, msg)
            self._cnt['r'] += 1

    # ── /imu/data ────────────────────────────────────────────
    def _ci(self, msg):
        t = stamp_sec(msg.header.stamp)
        g = msg.angular_velocity
        a = msg.linear_acceleration
        q = msg.orientation
        with self._lock:
            self._wi.writerow([
                f'{t:.6f}',
                f'{g.x:.8f}', f'{g.y:.8f}', f'{g.z:.8f}',
                f'{a.x:.6f}', f'{a.y:.6f}', f'{a.z:.6f}',
                f'{q.x:.8f}', f'{q.y:.8f}', f'{q.z:.8f}', f'{q.w:.8f}'])
            self._cnt['i'] += 1

    # ── /tf ─────────────────────────────────────────────────
    def _ct(self, msg):
        for tf in msg.transforms:
            if (tf.header.frame_id == 'map'
                    and tf.child_frame_id == 'odom'):
                t  = stamp_sec(tf.header.stamp)
                tr = tf.transform.translation
                rz = math.degrees(quat_yaw(tf.transform.rotation))
                with self._lock:
                    self._wt.writerow([
                        f'{t:.6f}', f'{tr.x:.6f}',
                        f'{tr.y:.6f}', f'{tr.z:.6f}', f'{rz:.4f}'])
                    self._cnt['t'] += 1

    # ── /bunker_status ───────────────────────────────────────
    def _cb(self, msg):
        t = time.time()
        with self._lock:
            self._wb.writerow([
                f'{t:.3f}',
                f'{msg.linear_velocity:.4f}',
                f'{msg.angular_velocity:.4f}',
                msg.control_mode,
                f'{msg.battery_voltage:.2f}'])
            self._cnt['b'] += 1

    # ── 进度 / 结束 ──────────────────────────────────────────
    def _prog(self):
        elapsed = time.time() - self._t0
        with self._lock:
            c  = dict(self._cnt)
            remain = len(self._pending)
        print(f"  [{SESSION}] +{elapsed:.0f}s  "
              f"a={c['a']} o={c['o']} r={c['r']} i={c['i']} "
              f"t={c['t']} b={c['b']}  wp={c['wp']}/{len(WAYPOINTS)}  "
              f"pending={remain}",
              flush=True)

    def _end(self):
        if self._done:
            return
        self._done = True
        files = [self._fa, self._fo, self._fr,
                 self._fi, self._ft, self._fwp]
        if HAS_BUNKER:
            files.append(self._fb)
        for f in files:
            f.flush()
            f.close()
        elapsed = time.time() - self._t0
        with self._lock:
            c      = dict(self._cnt)
            remain = len(self._pending)
        print(f"\n  [DONE] {SESSION}  {elapsed:.1f}s  "
              f"a={c['a']} o={c['o']} r={c['r']} i={c['i']} "
              f"t={c['t']} b={c['b']}  wp={c['wp']}/{len(WAYPOINTS)}",
              flush=True)
        if remain:
            missed = [i+1 for i in self._pending]
            print(f"  [WP]  未到达路点: {missed}", flush=True)
        print(f"  输出: {OUTPUT_DIR}", flush=True)
        raise SystemExit(0)


rclpy.init()
node = Col()
try:
    rclpy.spin(node)
except (KeyboardInterrupt, SystemExit):
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
PYEOF

COLLECTOR_PID=$!
echo "[collector] PID=${COLLECTOR_PID}  运行 ${DURATION}s 后自动停止"
echo ""
echo "  现在请在另一个终端启动导航并行走路线"
echo "  提前停止: kill ${COLLECTOR_PID}  或 Ctrl+C"
echo ""

wait "${COLLECTOR_PID}" 2>/dev/null

echo ""
echo "=========================================="
echo "  输出目录: ${OUTPUT_DIR}"
echo "  路点误差: cat ${OUTPUT_DIR}/waypoint_errors.csv"
echo "  快速查看: wc -l ${OUTPUT_DIR}/*.csv"
echo "=========================================="
