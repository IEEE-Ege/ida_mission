#!/usr/bin/env python3
# =============================================================================
# mission_node.py — Zincirleme Görev Durum Makinesi (FDIR entegre)
# =============================================================================
# Akış (KTR §2 ile hizalı, otomatik geçişli):
#
#   INIT ─(/system/health=true)─► IDLE ─/mission/start─► P1_TRANSIT
#     (sağlık kapısı)                                        │ GN1→…→GN4
#                                                            ▼
#                              P2_NAVIGATE (kapı tespiti + NavigateThroughPoses)
#                                                            │ ≥2 kapı + GN5
#                                                            ▼
#                              P3_SCAN ─(hedef renk)─► P3_KAMIKAZE
#                                                            │ LiDAR mesafe VEYA
#                                                            │ IMU çarpma (|a|>3G)
#                                                            ▼
#                                              COMPLETE (HOLD + DISARM)
#
#   Her durumdan:
#     /system/safe_mode | /estop/active | /failsafe/triggered ─► SAFE_MODE
#       (cmd_vel nötr + MAVROS HOLD + /estop/command=true fiziksel güç kesimi)
#     YKİ heartbeat kaybı > eşik ─► CONNECTION_FAIL (HOLD → RTL → SAFE_MODE)
#
# SAFE_MODE mandallıdır; çıkış yalnızca /system/reset (operatör) ile.
# =============================================================================

import math
import time
from enum import Enum, auto
from typing import List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data

from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String, Bool, Int32, Header
from action_msgs.msg import GoalStatus
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import WaypointList

from ida_msgs.msg import BuoyDetection, BuoyDetectionArray, MissionState
from .gate_calculator import (nearest_buoy_by_color, _yaw_to_quaternion,
                              find_gate_pairs, gate_midpoint_pose)

_G = 9.80665   # m/s²


class State(Enum):
    INIT            = auto()   # sağlık kontrolü — hepsi OK olmadan start yok
    IDLE            = auto()   # başlatmaya hazır
    P1_TRANSIT      = auto()   # GN1→…→GN4 sıralı waypoint
    P2_NAVIGATE     = auto()   # kapı tespiti + NavigateThroughPoses
    P3_SCAN         = auto()
    P3_KAMIKAZE     = auto()
    COMPLETE        = auto()
    FAILSAFE        = auto()
    SAFE_MODE       = auto()   # mandallı — /system/reset ile çıkılır
    CONNECTION_FAIL = auto()   # YKİ bağlantı kaybı
    RETURNING       = auto()   # operatör RTL komutu — Pixhawk eve dönüyor
    HOLD            = auto()   # operatör duraklat (HOLD) — görev beklemede


class PID:
    """Basit PID — bayrak windup koruması ile."""
    def __init__(self, kp: float, ki: float, kd: float,
                 i_clamp: float = 1.0, out_clamp: float = math.inf):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_clamp   = i_clamp
        self.out_clamp = out_clamp
        self.integral  = 0.0
        self.last_err  = 0.0
        self.last_t    = None

    def reset(self):
        self.integral = 0.0
        self.last_err = 0.0
        self.last_t   = None

    def step(self, error: float, now: float) -> float:
        if self.last_t is None:
            self.last_t = now
            self.last_err = error
            return self.kp * error
        dt = max(now - self.last_t, 1e-3)
        self.integral = max(-self.i_clamp,
                            min(self.i_clamp, self.integral + error * dt))
        derivative = (error - self.last_err) / dt
        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        out = max(-self.out_clamp, min(self.out_clamp, out))
        self.last_err = error
        self.last_t   = now
        return out


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')

        # ── Parametreler ──────────────────────────────────────────────────
        self.declare_parameter('target_color', 'red')
        self.declare_parameter('odom_frame',   'odom')

        # ÖZEL/YARIŞMA VERİSİ ÇIKARILDI: gerçek görev noktaları config'ten
        # (mission_params.yaml) verilir. Buradaki varsayılanlar yer tutucudur.
        self.declare_parameter('p1_waypoints',
                               [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('p2_goal',  [0.0, 0.0])   # GN5 <-- config'ten doldurun

        self.declare_parameter('min_gate_width', 2.0)
        self.declare_parameter('max_gate_width', 12.0)

        # P3 görsel servo
        self.declare_parameter('p3_conf_threshold',     0.50)
        self.declare_parameter('p3_img_center_x',       320.0)
        self.declare_parameter('p3_kp_yaw',            -0.004)
        self.declare_parameter('p3_ki_yaw',             0.0)
        self.declare_parameter('p3_kd_yaw',            -0.0008)
        self.declare_parameter('p3_base_linear_vel',    0.5)
        self.declare_parameter('p3_max_linear_vel',     1.5)
        self.declare_parameter('p3_approach_distance', 6.0)
        self.declare_parameter('p3_contact_distance',  0.8)
        self.declare_parameter('p3_target_loss_timeout', 0.7)
        self.declare_parameter('p3_kamikaze_timeout',  30.0)
        self.declare_parameter('p3_scan_angular_vel',   0.4)

        # ── Yeni: güvenlik / FDIR / bağlantı / IMU / retry / upload ────────
        self.declare_parameter('require_health', True)      # INIT sağlık kapısı
        self.declare_parameter('imu_contact_g', 3.0)        # |a| eşiği (×G)
        self.declare_parameter('imu_contact_duration', 0.10) # s
        self.declare_parameter('complete_settle_sec', 2.0)  # temas→DISARM gecikmesi
        self.declare_parameter('hold_mode', 'HOLD')
        self.declare_parameter('rtl_mode',  'RTL')
        self.declare_parameter('yki_heartbeat_timeout', 5.0)  # CONNECTION_FAIL eşiği
        self.declare_parameter('conn_rtl_timeout', 30.0)      # HOLD→RTL
        self.declare_parameter('conn_safe_timeout', 90.0)     # RTL→SAFE_MODE
        self.declare_parameter('p1_goal_timeout', 60.0)       # NavigateToPose retry eşiği
        self.declare_parameter('p1_max_retry', 1)
        self.declare_parameter('costmap_clear_radius', 3.0)
        self.declare_parameter('enable_mission_upload', True) # YKİ waypoint yükleme
        self.declare_parameter('enable_pixhawk_backup', True) # Pixhawk AUTO yedeği
        self.declare_parameter('enable_gate_detection', True) # P2 kapı tespiti

        self._target_color = self.get_parameter('target_color').value
        self._odom_frame   = self.get_parameter('odom_frame').value

        wp_flat = list(self.get_parameter('p1_waypoints').value)
        if len(wp_flat) % 2 != 0:
            self.get_logger().error('p1_waypoints çift sayıda eleman içermeli.')
            wp_flat = wp_flat[: len(wp_flat) - 1]
        self._p1_waypoints: List[Tuple[float, float]] = [
            (wp_flat[i], wp_flat[i + 1]) for i in range(0, len(wp_flat), 2)
        ]
        p2_goal = list(self.get_parameter('p2_goal').value)
        self._p2_goal: Tuple[float, float] = (p2_goal[0], p2_goal[1])

        self._min_gw = self.get_parameter('min_gate_width').value
        self._max_gw = self.get_parameter('max_gate_width').value

        self._conf_thr     = self.get_parameter('p3_conf_threshold').value
        self._img_cx       = self.get_parameter('p3_img_center_x').value
        self._base_lin     = self.get_parameter('p3_base_linear_vel').value
        self._max_lin      = self.get_parameter('p3_max_linear_vel').value
        self._approach_d   = self.get_parameter('p3_approach_distance').value
        self._contact_d    = self.get_parameter('p3_contact_distance').value
        self._loss_timeout = self.get_parameter('p3_target_loss_timeout').value
        self._kamikaze_to  = self.get_parameter('p3_kamikaze_timeout').value
        self._scan_omega   = self.get_parameter('p3_scan_angular_vel').value

        self._require_health = self.get_parameter('require_health').value
        self._imu_contact_a  = self.get_parameter('imu_contact_g').value * _G
        self._imu_contact_dt = self.get_parameter('imu_contact_duration').value
        self._settle_sec     = self.get_parameter('complete_settle_sec').value
        self._hold_mode      = self.get_parameter('hold_mode').value
        self._rtl_mode       = self.get_parameter('rtl_mode').value
        self._yki_hb_to      = self.get_parameter('yki_heartbeat_timeout').value
        self._conn_rtl_to    = self.get_parameter('conn_rtl_timeout').value
        self._conn_safe_to   = self.get_parameter('conn_safe_timeout').value
        self._p1_goal_to     = self.get_parameter('p1_goal_timeout').value
        self._p1_max_retry   = int(self.get_parameter('p1_max_retry').value)
        self._clear_radius   = self.get_parameter('costmap_clear_radius').value
        self._en_upload      = self.get_parameter('enable_mission_upload').value
        self._en_backup      = self.get_parameter('enable_pixhawk_backup').value
        self._en_gates       = self.get_parameter('enable_gate_detection').value

        self._yaw_pid = PID(
            kp=self.get_parameter('p3_kp_yaw').value,
            ki=self.get_parameter('p3_ki_yaw').value,
            kd=self.get_parameter('p3_kd_yaw').value,
            i_clamp=200.0, out_clamp=1.2)

        # ── Durum ─────────────────────────────────────────────────────────
        self._state          = State.INIT
        self._prev_state     = State.INIT   # CONNECTION_FAIL sonrası dönüş için
        self._mission_start  = 0.0
        self._state_enter    = 0.0
        self._robot_x = self._robot_y = 0.0
        self._robot_vx = self._robot_vy = 0.0

        self._p1_idx           = 0
        self._p1_retry         = 0
        self._p1_goal_sent_t   = 0.0
        self._nav_goal_handle  = None
        self._nav_in_flight    = False

        self._latest_buoys: List[BuoyDetection] = []
        self._target_last_seen_t   = 0.0
        self._target_last_odom: Optional[Tuple[float, float]] = None
        self._target_last_distance = float('inf')
        self._target_last_bbox_cx  = -1.0
        self._target_locked_blind  = False

        # Güvenlik / bağlantı / IMU / GPS
        self._health_ok      = not self._require_health
        self._safe_mode_ext  = False       # /system/safe_mode
        self._estop_active   = False
        self._imu_high_a_t0  = None        # yüksek ivme başlangıcı
        self._imu_contact    = False
        self._yki_last_hb    = 0.0
        self._yki_hb_seen    = False
        self._complete_t0    = None
        self._disarmed       = False
        self._launch_xy: Optional[Tuple[float, float]] = None
        self._gps_origin: Optional[Tuple[float, float]] = None

        # ── Nav2 ──────────────────────────────────────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._ntp_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        sensor_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                history=QoSHistoryPolicy.KEEP_LAST, depth=5)

        # ── Abonelikler ───────────────────────────────────────────────────
        self.create_subscription(BuoyDetectionArray, '/buoy_detections', self._buoy_cb, sensor_qos)
        self.create_subscription(Odometry, '/mavros/local_position/odom', self._odom_cb, sensor_qos)
        self.create_subscription(String, '/mission/start', self._start_cb, 10)
        # YKİ/MissionPlanner komut kanalı (mavlink_command_bridge → GCS niyeti):
        #   "go"/"start" | "resume" | "rtl" | "hold" | "reset"
        self.create_subscription(String, '/mission/command', self._command_cb, 10)
        # YKİ hedef rengi (mavlink_command_bridge → RGB→sınıf). Görev başlamadan
        # önce güncellenebilir; başladıktan sonra yok sayılır (şartname).
        self.create_subscription(String, '/mission/target_color', self._target_color_cb, 10)
        self.create_subscription(Bool, '/failsafe/triggered', self._failsafe_cb, 10)
        self.create_subscription(Bool, '/system/health', self._health_cb, 10)
        self.create_subscription(Bool, '/system/safe_mode', self._safe_mode_cb, 10)
        self.create_subscription(Bool, '/estop/active', self._estop_cb, 10)
        self.create_subscription(Imu, '/mavros/imu/data', self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(Bool, '/yki/heartbeat', self._yki_hb_cb, 10)
        self.create_subscription(NavSatFix, '/mavros/global_position/global',
                                 self._gps_cb, qos_profile_sensor_data)
        if self._en_upload:
            self.create_subscription(WaypointList, '/mavros/mission/waypoints',
                                     self._waypoints_cb, 10)

        # ── Yayıncılar ────────────────────────────────────────────────────
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self._state_pub   = self.create_publisher(MissionState, '/mission/state', 10)
        self._excl_pub    = self.create_publisher(Int32, '/perception/exclude_cluster_id', 10)
        self._estop_cmd_pub = self.create_publisher(Bool, '/estop/command', 10)
        # P2 sanal koridor sınırı (costmap engel kaynağı /virtual_boundary_cloud)
        self._boundary_pub = self.create_publisher(PointCloud2, '/virtual_boundary_cloud', 10)

        # ── Servis istemcileri ────────────────────────────────────────────
        self._mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self._arm_client  = self.create_client(CommandBool, '/mavros/cmd/arming')
        self._clear_client = None   # tembel: ClearCostmapAroundRobot

        self.create_timer(0.1, self._spin_once)   # 10 Hz
        self.get_logger().info(
            f'Görev düğümü hazır (INIT). Hedef="{self._target_color}", '
            f'sağlık kapısı={"açık" if self._require_health else "kapalı"}, '
            f'P1={self._p1_waypoints}, GN5={self._p2_goal}')

    # ═════════════════════════════════════════════════════════════════════
    # Abonelik geri çağrıları
    # ═════════════════════════════════════════════════════════════════════

    def _odom_cb(self, msg: Odometry):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        self._robot_vx = msg.twist.twist.linear.x
        self._robot_vy = msg.twist.twist.linear.y
        if self._launch_xy is None:
            self._launch_xy = (self._robot_x, self._robot_y)   # safe point

    def _buoy_cb(self, msg: BuoyDetectionArray):
        self._latest_buoys = list(msg.buoys)
        if self._state in (State.P3_SCAN, State.P3_KAMIKAZE):
            tgt = nearest_buoy_by_color(self._latest_buoys, self._target_color,
                                        (self._robot_x, self._robot_y))
            if tgt is not None and tgt.confidence >= self._conf_thr:
                self._target_last_seen_t   = time.time()
                self._target_last_odom     = (tgt.position.x, tgt.position.y)
                self._target_last_distance = tgt.distance if tgt.distance > 0 else \
                    math.hypot(tgt.position.x - self._robot_x, tgt.position.y - self._robot_y)
                self._target_last_bbox_cx  = tgt.bbox_center_x
                if tgt.cluster_id >= 0:
                    self._excl_pub.publish(Int32(data=tgt.cluster_id))

    def _health_cb(self, msg: Bool):
        # Sağlık kapısı KAPALIYSA (require_health=false), FDIR false yayınlasa
        # bile INIT'i tekrar kilitleme. Aksi halde latched /system/health=false,
        # mission_node abone olur olmaz _health_ok'u false'a çekip INIT→IDLE
        # geçişini kalıcı olarak engelliyordu (require_health=false etkisiz kalıyordu).
        self._health_ok = bool(msg.data) or (not self._require_health)

    def _safe_mode_cb(self, msg: Bool):
        self._safe_mode_ext = msg.data

    def _estop_cb(self, msg: Bool):
        self._estop_active = msg.data

    def _yki_hb_cb(self, msg: Bool):
        self._yki_hb_seen = True
        self._yki_last_hb = time.time()

    def _gps_cb(self, msg: NavSatFix):
        if self._gps_origin is None and msg.status.status >= 0:
            self._gps_origin = (msg.latitude, msg.longitude)
            self.get_logger().info(
                f'GPS orijini kilitlendi (safe point): '
                f'({self._gps_origin[0]:.7f}, {self._gps_origin[1]:.7f})')

    def _imu_cb(self, msg: Imu):
        # KTR §2.6/3.5: proper-acceleration büyüklüğü > 3G (ham |a|; MAVROS
        # /mavros/imu/data yerçekimi dahil özgül ivme verir → durağanda ~1G,
        # çarpma anında sivri artış). Yalnızca KAMIKAZE'de temas onayı sayılır.
        a = msg.linear_acceleration
        a_mag = math.sqrt(a.x*a.x + a.y*a.y + a.z*a.z)
        now = time.time()
        if a_mag > self._imu_contact_a:
            if self._imu_high_a_t0 is None:
                self._imu_high_a_t0 = now
            elif now - self._imu_high_a_t0 >= self._imu_contact_dt:
                if self._state == State.P3_KAMIKAZE:
                    self._imu_contact = True
        else:
            self._imu_high_a_t0 = None

    def _waypoints_cb(self, msg: WaypointList):
        """YKİ'den MAVLink MISSION_ITEM_INT → yerel P1 waypointleri (MISSION_LOAD)."""
        if not self._en_upload or self._state not in (State.INIT, State.IDLE):
            return
        if self._gps_origin is None or not msg.waypoints:
            return
        pts = []
        for wp in msg.waypoints:
            # frame 3 = GLOBAL_RELATIVE_ALT; x_lat/y_long alanları enlem/boylam
            if wp.x_lat == 0.0 and wp.y_long == 0.0:
                continue
            xy = self._geodetic_to_local(wp.x_lat, wp.y_long)
            pts.append(xy)
        if len(pts) >= 2:
            self._p1_waypoints = pts[:-1]
            self._p2_goal = pts[-1]
            self.get_logger().info(
                f'YKİ görevi yüklendi (MISSION_LOAD): {len(pts)} nokta → '
                f'P1={self._p1_waypoints}, GN5={self._p2_goal}')

    # ═════════════════════════════════════════════════════════════════════
    # Ana döngü
    # ═════════════════════════════════════════════════════════════════════

    def _spin_once(self):
        self._publish_state()

        # ── Global güvenlik geçişleri (her durumda) ────────────────────────
        if self._state not in (State.SAFE_MODE,):
            if self._safe_mode_ext or self._estop_active:
                self._enter_safe_mode('FDIR/estop sinyali')
                return
            if self._connection_lost() and self._state in (
                    State.P1_TRANSIT, State.P2_NAVIGATE, State.P3_SCAN,
                    State.P3_KAMIKAZE):
                self._enter_connection_fail()
                return

        # ── Durum davranışları ─────────────────────────────────────────────
        if self._state == State.INIT:
            if self._health_ok or not self._require_health:
                self.get_logger().info('Sağlık kontrolleri OK → IDLE (başlatmaya hazır).')
                self._transition(State.IDLE)
        elif self._state == State.P3_SCAN:
            self._run_p3_scan()
        elif self._state == State.P3_KAMIKAZE:
            self._run_p3_kamikaze()
        elif self._state == State.CONNECTION_FAIL:
            self._run_connection_fail()
        elif self._state == State.COMPLETE:
            self._run_complete()
        elif self._state in (State.SAFE_MODE, State.FAILSAFE):
            self._stop_robot()
        elif self._state == State.HOLD:
            # Operatör duraklattı: Jetson dursun, Pixhawk HOLD'da beklesin.
            self._stop_robot()
        elif self._state == State.RETURNING:
            # Operatör RTL: Pixhawk eve dönüyor; Jetson cmd_vel GÖNDERMEZ
            # (aksi halde RTL'i bozardı). cmd_vel_relay bu durumda devralmaz.
            pass
        # P1_TRANSIT / P2_NAVIGATE: Nav2 callback'leri yönetir; P1 retry watchdog:
        if self._state == State.P1_TRANSIT:
            self._p1_watchdog()

    # ═════════════════════════════════════════════════════════════════════
    # Başlatma
    # ═════════════════════════════════════════════════════════════════════

    def _start_cb(self, msg: String):
        self._begin_mission()

    def _begin_mission(self):
        """IDLE → P1: görevi baştan başlat."""
        if self._state != State.IDLE:
            self.get_logger().warn(
                f'Başlatma reddedildi: durum {self._state.name} (IDLE değil).')
            return
        self.get_logger().info('Görev başlatılıyor: P1 → P2 → P3')
        self._mission_start = time.time()
        self._p1_idx = 0
        self._p1_retry = 0
        if self._en_backup:
            self._push_pixhawk_backup()
        self._transition(State.P1_TRANSIT)
        self._send_p1_next_waypoint()

    # ── YKİ / MissionPlanner komut kanalı ──────────────────────────────────
    def _command_cb(self, msg: String):
        cmd = str(msg.data).strip().lower()
        if cmd in ('go', 'start', 'auto', 'basla', 'başla'):
            # IDLE → başlat; duraklamış durumdaysa devam et.
            if self._state == State.IDLE:
                self._begin_mission()
            elif self._state in (State.HOLD, State.RETURNING,
                                 State.CONNECTION_FAIL):
                self._resume_mission()
            else:
                self.get_logger().info(f'"{cmd}" yok sayıldı (durum {self._state.name}).')
        elif cmd in ('resume', 'devam', 'continue'):
            self._resume_mission()
        elif cmd in ('rtl', 'return', 'eve'):
            self._enter_returning()
        elif cmd in ('hold', 'pause', 'duraklat', 'bekle'):
            self._enter_hold_manual()
        elif cmd in ('reset', 'arm', 'clear'):
            self._reset_from_safe()
        else:
            self.get_logger().warn(f'Bilinmeyen /mission/command: "{cmd}"')

    def _resume_mission(self):
        """HOLD / RETURNING / CONNECTION_FAIL → kaldığı göreve dön."""
        if self._state not in (State.HOLD, State.RETURNING, State.CONNECTION_FAIL):
            self.get_logger().info(f'Devam yok sayıldı (durum {self._state.name}).')
            return
        prev = self._prev_state if self._prev_state in (
            State.P1_TRANSIT, State.P2_NAVIGATE) else State.P1_TRANSIT
        self.get_logger().info(f'Göreve devam ediliyor → {prev.name}')
        self._conn_rtl_sent = False
        self._transition(prev)
        if prev == State.P1_TRANSIT:
            self._send_p1_next_waypoint()
        elif prev == State.P2_NAVIGATE:
            self._enter_p2()

    def _enter_returning(self):
        """Operatör RTL komutu: Pixhawk RTL ile eve döner; otonom devralmayı bırakır."""
        if self._state in (State.RETURNING, State.CONNECTION_FAIL, State.SAFE_MODE):
            return   # zaten dönüyor / güvenlik durumunda — bozma
        if self._state in (State.P1_TRANSIT, State.P2_NAVIGATE,
                           State.P3_SCAN, State.P3_KAMIKAZE):
            self._prev_state = self._state
        self.get_logger().warn('Operatör RTL → RETURNING (Pixhawk eve dönüyor).')
        self._cancel_nav()
        self._stop_robot()
        self._transition(State.RETURNING)
        self._set_mode(self._rtl_mode)

    def _enter_hold_manual(self):
        """Operatör duraklat: görevi HOLD'da beklet (devam edilebilir)."""
        if self._state not in (State.P1_TRANSIT, State.P2_NAVIGATE,
                               State.P3_SCAN, State.P3_KAMIKAZE):
            return
        self._prev_state = self._state
        self.get_logger().warn('Operatör HOLD → görev duraklatıldı.')
        self._cancel_nav()
        self._stop_robot()
        self._transition(State.HOLD)
        self._set_mode(self._hold_mode)

    def _reset_from_safe(self):
        """Motor Başlat / arm: SAFE_MODE'dan kurtul (güç geri geldiyse INIT'e)."""
        if self._state == State.SAFE_MODE:
            self.get_logger().warn('RESET komutu → SAFE_MODE\'dan çıkılıyor (INIT).')
            self._safe_mode_ext = False
            self._estop_active  = False
            self._transition(State.INIT)
        else:
            self.get_logger().info(f'RESET yok sayıldı (durum {self._state.name}).')

    def _failsafe_cb(self, msg: Bool):
        if msg.data and self._state not in (State.SAFE_MODE,):
            self.get_logger().error('FAILSAFE! → SAFE_MODE.')
            self._enter_safe_mode('/failsafe/triggered')

    def _target_color_cb(self, msg: String):
        # Şartname: hedef bilgisi ancak görev BAŞLAMADAN önce aktarılabilir.
        if self._state in (State.INIT, State.IDLE):
            if msg.data and msg.data != self._target_color:
                self._target_color = msg.data
                self.get_logger().info(f'YKİ hedef rengi güncellendi: "{self._target_color}"')
        else:
            self.get_logger().warn(
                f'Hedef rengi görev sırasında değiştirilemez (durum={self._state.name}).')

    # ═════════════════════════════════════════════════════════════════════
    # P1 — Sıralı waypoint (retry + costmap temizleme)
    # ═════════════════════════════════════════════════════════════════════

    def _send_p1_next_waypoint(self):
        if self._p1_idx >= len(self._p1_waypoints):
            self.get_logger().info('Son GN tamamlandı. P2_NAVIGATE başlıyor.')
            self._enter_p2()
            return

        x, y = self._p1_waypoints[self._p1_idx]
        self.get_logger().info(f'P1: GN{self._p1_idx + 1} → ({x:.1f}, {y:.1f})')

        next_idx = self._p1_idx + 1
        if next_idx < len(self._p1_waypoints):
            nx, ny = self._p1_waypoints[next_idx]
            yaw = math.atan2(ny - y, nx - x)
        else:
            yaw = math.atan2(self._p2_goal[1] - y, self._p2_goal[0] - x)

        self._p1_goal_sent_t = time.time()
        self._send_nav_xy(x, y, yaw=yaw, done_cb=self._on_p1_waypoint_done)

    def _p1_watchdog(self):
        """NavigateToPose 60 s'de sonuçlanmazsa costmap temizle + retry, sonra atla."""
        if not self._nav_in_flight or self._p1_goal_sent_t == 0.0:
            return
        if time.time() - self._p1_goal_sent_t < self._p1_goal_to:
            return
        if self._p1_retry < self._p1_max_retry:
            self._p1_retry += 1
            self.get_logger().warn(
                f'GN{self._p1_idx + 1} timeout ({self._p1_goal_to:.0f}s) — '
                f'costmap temizleniyor, retry {self._p1_retry}/{self._p1_max_retry}.')
            self._clear_costmap_around_robot()
            self._cancel_nav()
            self._send_p1_next_waypoint()
        else:
            self.get_logger().warn(f'GN{self._p1_idx + 1} atlandı (retry bitti).')
            self._cancel_nav()
            self._p1_retry = 0
            self._p1_idx += 1
            self._send_p1_next_waypoint()

    def _on_p1_waypoint_done(self):
        if self._state != State.P1_TRANSIT:
            return
        self._p1_retry = 0
        self._p1_idx += 1
        self._send_p1_next_waypoint()

    # ═════════════════════════════════════════════════════════════════════
    # P2 — Kapı tespiti + NavigateThroughPoses (+ sanal koridor sınırı)
    # ═════════════════════════════════════════════════════════════════════

    def _enter_p2(self):
        self._transition(State.P2_NAVIGATE)
        poses: List[Pose] = []
        if self._en_gates:
            poses = self._compute_gate_poses()
            self._publish_virtual_boundary()
        # GN5 her zaman son hedef
        gn5 = Pose()
        gn5.position.x = self._p2_goal[0]
        gn5.position.y = self._p2_goal[1]
        gn5.orientation = _yaw_to_quaternion(0.0)
        poses.append(gn5)

        if len(poses) > 1:
            self.get_logger().info(
                f'P2: {len(poses)-1} kapı + GN5 → NavigateThroughPoses.')
            self._send_nav_through(poses, done_cb=self._on_p2_done)
        else:
            self.get_logger().info('P2: kapı bulunamadı — doğrudan GN5 (costmap kaçınma).')
            self._send_nav_pose(gn5, done_cb=self._on_p2_done)

    def _compute_gate_poses(self) -> List[Pose]:
        """Turuncu kenar duba çiftlerini kapı olarak ara; orta noktalarını
        ileri mesafeye göre sırala → geçiş pozları."""
        pairs = find_gate_pairs(self._latest_buoys, color_filter='orange',
                                min_width=self._min_gw, max_width=self._max_gw)
        robot = (self._robot_x, self._robot_y)
        poses = []
        for a, b, _w in pairs:
            poses.append(gate_midpoint_pose(a, b, robot_pos=robot))
        # İleri mesafeye göre sırala (robota yakından uzağa)
        poses.sort(key=lambda p: math.hypot(p.position.x - self._robot_x,
                                             p.position.y - self._robot_y))
        return poses

    def _publish_virtual_boundary(self):
        """Turuncu kenar dubalarını sol/sağ gruplara ayır; ardışık aynı-taraf
        dubalar arası noktaları yoğunlaştırıp costmap engel kaynağına yayınla
        (RPP parkur sınırını aşamaz)."""
        oranges = [b for b in self._latest_buoys if b.color == 'orange']
        if len(oranges) < 2:
            return
        # Aracın ileri yön vektörü (heading yoksa GN5 yönü)
        hx = self._p2_goal[0] - self._robot_x
        hy = self._p2_goal[1] - self._robot_y
        n = math.hypot(hx, hy) or 1.0
        hx, hy = hx / n, hy / n
        rx, ry = -hy, hx   # sağ eksen
        left, right = [], []
        for b in oranges:
            side = (b.position.x - self._robot_x) * rx + (b.position.y - self._robot_y) * ry
            (right if side >= 0 else left).append((b.position.x, b.position.y))
        pts = []
        for grp in (left, right):
            grp.sort(key=lambda p: (p[0]-self._robot_x)*hx + (p[1]-self._robot_y)*hy)
            for i in range(len(grp) - 1):
                (x0, y0), (x1, y1) = grp[i], grp[i + 1]
                seg = math.hypot(x1 - x0, y1 - y0)
                steps = max(int(seg / 0.3), 1)
                for s in range(steps + 1):
                    t = s / steps
                    pts.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0), 0.2))
        if not pts:
            return
        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        hdr.frame_id = self._odom_frame
        self._boundary_pub.publish(point_cloud2.create_cloud_xyz32(hdr, pts))

    def _on_p2_done(self):
        if self._state != State.P2_NAVIGATE:
            return
        self.get_logger().info('GN5 tamamlandı. P3_SCAN başlıyor.')
        self._transition(State.P3_SCAN)
        self._yaw_pid.reset()
        self._target_last_seen_t = 0.0
        self._target_last_odom = None
        self._target_last_distance = float('inf')
        self._target_locked_blind = False

    # ═════════════════════════════════════════════════════════════════════
    # P3 — SCAN / KAMIKAZE
    # ═════════════════════════════════════════════════════════════════════

    def _run_p3_scan(self):
        if (self._target_last_odom is not None and
                time.time() - self._target_last_seen_t < self._loss_timeout):
            self.get_logger().info(
                f'Hedef ({self._target_color}) bulundu. '
                f'd={self._target_last_distance:.2f}m. KAMIKAZE.')
            self._yaw_pid.reset()
            self._target_locked_blind = False
            self._imu_contact = False
            self._imu_high_a_t0 = None
            self._transition(State.P3_KAMIKAZE)
            return
        twist = Twist()
        twist.angular.z = self._scan_omega
        self._cmd_vel_pub.publish(twist)

    def _run_p3_kamikaze(self):
        now = time.time()
        if now - self._state_enter > self._kamikaze_to:
            self.get_logger().info('KAMIKAZE timeout. COMPLETE.')
            self._enter_complete()
            return
        # Kontak: LiDAR mesafesi VEYA IMU çarpma onayı (KTR |a|>3G)
        if self._target_last_distance <= self._contact_d or self._imu_contact:
            src = 'IMU çarpma' if self._imu_contact else f'LiDAR {self._target_last_distance:.2f}m'
            self.get_logger().info(f'Kontak onaylandı ({src}). COMPLETE.')
            self._enter_complete()
            return

        bbox_fresh = (now - self._target_last_seen_t < self._loss_timeout
                      and self._target_last_bbox_cx >= 0.0)
        twist = Twist()
        if bbox_fresh:
            self._target_locked_blind = False
            pixel_err = self._target_last_bbox_cx - self._img_cx
            twist.angular.z = self._yaw_pid.step(pixel_err, now)
            twist.linear.x  = self._compute_thrust(self._target_last_distance)
        elif self._target_last_odom is not None:
            if not self._target_locked_blind:
                self.get_logger().warn('Hedef bbox kayboldu — kör nokta kilidi.')
                self._target_locked_blind = True
                self._blind_t0 = now
                self._yaw_pid.reset()
            dx = self._target_last_odom[0] - self._robot_x
            dy = self._target_last_odom[1] - self._robot_y
            desired_yaw = math.atan2(dy, dx)
            cur_yaw = math.atan2(self._robot_vy, self._robot_vx) \
                if math.hypot(self._robot_vx, self._robot_vy) > 0.05 else desired_yaw
            yaw_err = _wrap_angle(desired_yaw - cur_yaw)
            # 2 s'den uzun kör kalırsa genişleyen arama spiraline geç (KTR §3.5)
            if now - getattr(self, '_blind_t0', now) > 2.0:
                twist.angular.z = self._scan_omega
                twist.linear.x  = self._base_lin * 0.5
            else:
                twist.angular.z = max(-1.0, min(1.0, 1.5 * yaw_err))
                twist.linear.x  = self._max_lin
            if math.hypot(dx, dy) <= self._contact_d:
                self.get_logger().info('Kör nokta konum kilidi kontağı. COMPLETE.')
                self._enter_complete()
                return
        else:
            twist.angular.z = self._scan_omega
        self._cmd_vel_pub.publish(twist)

    def _compute_thrust(self, distance: float) -> float:
        if distance >= self._approach_d:
            return self._base_lin
        if distance <= 0.0:
            return self._max_lin
        ratio = 1.0 - (distance / self._approach_d)
        return self._base_lin + ratio * (self._max_lin - self._base_lin)

    # ═════════════════════════════════════════════════════════════════════
    # COMPLETE → HOLD + DISARM (KTR §2.7)
    # ═════════════════════════════════════════════════════════════════════

    def _enter_complete(self):
        self._stop_robot()
        self._transition(State.COMPLETE)
        self._complete_t0 = time.time()
        self._disarmed = False
        self._set_mode(self._hold_mode)
        self.get_logger().info(
            f'Görev tamam — {self._hold_mode} + {self._settle_sec:.0f}s sonra DISARM.')

    def _run_complete(self):
        self._stop_robot()
        if not self._disarmed and self._complete_t0 is not None and \
                time.time() - self._complete_t0 >= self._settle_sec:
            self._disarm()
            self._disarmed = True

    # ═════════════════════════════════════════════════════════════════════
    # SAFE_MODE / CONNECTION_FAIL
    # ═════════════════════════════════════════════════════════════════════

    def _enter_safe_mode(self, reason: str):
        self.get_logger().error(f'*** SAFE_MODE *** ({reason})')
        self._cancel_nav()
        self._stop_robot()
        self._transition(State.SAFE_MODE)
        self._set_mode(self._hold_mode)
        # Fiziksel güç kesimi (estop heartbeat'i durdur → röle bırakır)
        self._estop_cmd_pub.publish(Bool(data=True))

    def _connection_lost(self) -> bool:
        if not self._yki_hb_seen:
            return False   # YKİ heartbeat hiç gelmediyse bu koruma pasif
        return (time.time() - self._yki_last_hb) > self._yki_hb_to

    def _enter_connection_fail(self):
        self.get_logger().error('YKİ bağlantısı koptu → CONNECTION_FAIL (HOLD).')
        self._prev_state = self._state
        self._cancel_nav()
        self._stop_robot()
        self._transition(State.CONNECTION_FAIL)
        self._set_mode(self._hold_mode)
        self._conn_rtl_sent = False

    def _run_connection_fail(self):
        self._stop_robot()
        elapsed = time.time() - self._state_enter
        if not self._connection_lost():
            self.get_logger().info('YKİ bağlantısı geri geldi → göreve devam.')
            self._transition(self._prev_state)
            if self._prev_state == State.P1_TRANSIT:
                self._send_p1_next_waypoint()
            elif self._prev_state == State.P2_NAVIGATE:
                self._enter_p2()
            return
        if elapsed > self._conn_safe_to:
            self._enter_safe_mode('CONNECTION_FAIL zaman aşımı')
        elif elapsed > self._conn_rtl_to and not getattr(self, '_conn_rtl_sent', False):
            self.get_logger().warn('CONNECTION_FAIL → RTL (safe point).')
            self._set_mode(self._rtl_mode)
            self._conn_rtl_sent = True

    # ═════════════════════════════════════════════════════════════════════
    # Nav2 yardımcıları
    # ═════════════════════════════════════════════════════════════════════

    def _send_nav_xy(self, x, y, yaw, done_cb):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.orientation = _yaw_to_quaternion(yaw)
        self._send_nav_pose(pose, done_cb)

    def _send_nav_pose(self, pose: Pose, done_cb):
        if not self._nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Nav2 (NavigateToPose) server yok!')
            return
        goal = NavigateToPose.Goal()
        goal.pose = self._stamped(pose)
        self._nav_in_flight = True
        self._nav_client.send_goal_async(goal).add_done_callback(
            lambda f: self._goal_accepted_cb(f, done_cb))

    def _send_nav_through(self, poses: List[Pose], done_cb):
        if not self._ntp_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Nav2 (NavigateThroughPoses) server yok! Tek hedefe düş.')
            self._send_nav_pose(poses[-1], done_cb)
            return
        goal = NavigateThroughPoses.Goal()
        goal.poses = [self._stamped(p) for p in poses]
        self._nav_in_flight = True
        self._ntp_client.send_goal_async(goal).add_done_callback(
            lambda f: self._goal_accepted_cb(f, done_cb))

    def _stamped(self, pose: Pose) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = self._odom_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = pose
        return ps

    def _goal_accepted_cb(self, future, done_cb):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Nav2 hedef reddedildi.')
            self._nav_in_flight = False
            done_cb()
            return
        self._nav_goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self._nav_done_cb(f, done_cb))

    def _nav_done_cb(self, future, done_cb):
        self._nav_goal_handle = None
        self._nav_in_flight = False
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'Nav2 başarısız (status={status}); devam.')
        done_cb()

    def _cancel_nav(self):
        if self._nav_goal_handle is not None:
            try:
                self._nav_goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._nav_goal_handle = None
        self._nav_in_flight = False

    def _clear_costmap_around_robot(self):
        from nav2_msgs.srv import ClearCostmapAroundRobot
        if self._clear_client is None:
            self._clear_client = self.create_client(
                ClearCostmapAroundRobot, '/local_costmap/clear_around_local_costmap')
        if self._clear_client.service_is_ready():
            req = ClearCostmapAroundRobot.Request()
            req.reset_distance = self._clear_radius
            self._clear_client.call_async(req)

    # ═════════════════════════════════════════════════════════════════════
    # MAVROS / Pixhawk yardımcıları
    # ═════════════════════════════════════════════════════════════════════

    def _set_mode(self, mode: str):
        if self._mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = mode
            self._mode_client.call_async(req)
            self.get_logger().info(f'{mode} modu istendi.')
        else:
            self.get_logger().warn(f'set_mode servisi hazır değil ({mode}).')

    def _disarm(self):
        if self._arm_client.service_is_ready():
            req = CommandBool.Request()
            req.value = False
            self._arm_client.call_async(req)
            self.get_logger().info('DISARM komutu gönderildi (aktüatör gücü kesiliyor).')
        else:
            self.get_logger().warn('arming servisi hazır değil; DISARM atlanıyor.')

    def _push_pixhawk_backup(self):
        """Waypointleri Pixhawk'a AUTO görev yedeği olarak yaz (Jetson çökerse
        Pixhawk devam eder — KTR §2.2)."""
        if self._gps_origin is None:
            self.get_logger().warn('GPS orijini yok; Pixhawk yedek görevi atlanıyor.')
            return
        try:
            from mavros_msgs.srv import WaypointPush
            from mavros_msgs.msg import Waypoint
        except Exception:
            return
        cli = self.create_client(WaypointPush, '/mavros/mission/push')
        if not cli.service_is_ready() and not cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('WaypointPush servisi yok; Pixhawk yedeği atlanıyor.')
            return
        wps = []
        allpts = list(self._p1_waypoints) + [self._p2_goal]
        for i, (x, y) in enumerate(allpts):
            lat, lon = self._local_to_geodetic(x, y)
            wp = Waypoint()
            wp.frame = 3          # GLOBAL_RELATIVE_ALT
            wp.command = 16       # NAV_WAYPOINT
            wp.is_current = (i == 0)
            wp.autocontinue = True
            wp.x_lat = lat
            wp.y_long = lon
            wp.z_alt = 0.0
            wps.append(wp)
        req = WaypointPush.Request()
        req.start_index = 0
        req.waypoints = wps
        cli.call_async(req)
        self.get_logger().info(f'Pixhawk AUTO yedek görevi yazıldı ({len(wps)} nokta).')

    # ═════════════════════════════════════════════════════════════════════
    # Geodezik dönüşüm (hafif ekvatoral projeksiyon — ikinci EKF'siz)
    # ═════════════════════════════════════════════════════════════════════

    _EARTH_R = 6378137.0

    def _geodetic_to_local(self, lat, lon) -> Tuple[float, float]:
        lat0, lon0 = self._gps_origin
        lat0r = math.radians(lat0)
        x = math.radians(lon - lon0) * self._EARTH_R * math.cos(lat0r)   # doğu = +x
        y = math.radians(lat - lat0) * self._EARTH_R                     # kuzey = +y
        return (x, y)

    def _local_to_geodetic(self, x, y) -> Tuple[float, float]:
        lat0, lon0 = self._gps_origin
        lat0r = math.radians(lat0)
        lat = lat0 + math.degrees(y / self._EARTH_R)
        lon = lon0 + math.degrees(x / (self._EARTH_R * math.cos(lat0r)))
        return (lat, lon)

    # ═════════════════════════════════════════════════════════════════════
    # Ortak yardımcılar
    # ═════════════════════════════════════════════════════════════════════

    def _stop_robot(self):
        self._cmd_vel_pub.publish(Twist())

    def _transition(self, new_state: State):
        self.get_logger().info(f'{self._state.name} → {new_state.name}')
        self._state = new_state
        self._state_enter = time.time()

    def _publish_state(self):
        msg = MissionState()
        msg.state = self._state.name
        msg.parkur_number = 0
        msg.gates_passed = self._p1_idx
        msg.total_gates = len(self._p1_waypoints)
        msg.target_color = self._target_color
        msg.mission_active = self._state in (
            State.P1_TRANSIT, State.P2_NAVIGATE, State.P3_SCAN, State.P3_KAMIKAZE)
        msg.elapsed_seconds = time.time() - self._mission_start \
            if self._mission_start else 0.0
        self._state_pub.publish(msg)


def _wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
