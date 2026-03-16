#!/usr/bin/env python3
"""
YAVUZ AUV — TEKNOFEST 2026 İleri Kategori Tema 2
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Otonom Navigasyon, İntikal ve Kontrollü Alan Geçişi

GÖREV PROTOKOLÜ:
  ┌─────────────────────────────────────────────────────────┐
  │  [INIT] ──► [DIVE] ──► [TRANSIT_TO_BUOY]               │
  │     └──────────────────────────────────┐                │
  │  [CIRCLE_BUOY] ◄── [ALIGN_TO_BUOY]    │                │
  │       │                                │                │
  │  [TRANSIT_TO_END] ──► [NAVIGATE_GATE]  │                │
  │       │                                │                │
  │  [SURFACE_IN_SQUARE] ──► [HOLD] ──► [DONE]             │
  │       └──► [EMERGENCY_STOP] (herhangi bir adımda)       │
  └─────────────────────────────────────────────────────────┘

PUANLAMA:
  Şamandıra etrafında tam tur  → 40 puan
  Bitiş karesinde yüzey çıkışı → 60 puan
  Süre bonusu                  → GP × (kalan_süre / 300)
  
KONUM: Dead reckoning EKF (IMU + derinlik + motor modeli)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import math
import time
import numpy as np
from enum import Enum, auto
from typing import Optional, Tuple
from dataclasses import dataclass

from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool, String, Int32
from sensor_msgs.msg import Imu
from builtin_interfaces.msg import Duration

# Yerel modüller
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from yavuz_control.pid import PIDController, AngularPIDController, PIDConfig
from yavuz_control.thruster_allocator import ThrusterAllocator


# ═══════════════════════════════════════════════════════════════════
# GÖREV DURUMLARI
# ═══════════════════════════════════════════════════════════════════

class MissionState(Enum):
    INIT              = auto()  # Sistem başlangıç kontrolü
    DIVE              = auto()  # İstenen derinliğe in
    TRANSIT_TO_BUOY   = auto()  # Şamandıraya git (su altı)
    ALIGN_TO_BUOY     = auto()  # Şamandıra etrafı başlangıç hizalanması
    CIRCLE_BUOY       = auto()  # Şamandıra etrafında 360° tur — 40 PUAN
    TRANSIT_TO_END    = auto()  # Bitiş koordinatına git (su altı)
    SURFACE_IN_SQUARE = auto()  # Kare içinde yüzey çıkışı — 60 PUAN
    HOLD_POSITION     = auto()  # Görev tamamlandı, bekle
    MISSION_COMPLETE  = auto()  # Başarı
    EMERGENCY_STOP    = auto()  # ACİL DURDURMA
    ERROR             = auto()  # Hata


@dataclass
class WayPoint:
    """Hedef nokta."""
    x: float       # m (yerel koordinat, İskele = 0,0)
    y: float       # m
    z: float       # m (negatif = su altı)
    tolerance: float = 1.0  # m, hedefe yaklaşım toleransı
    heading: Optional[float] = None  # rad, None = serbest


@dataclass
class MissionCoords:
    """Görev koordinatları (ip ucundan okunur veya yarışmacıya verilir)."""
    # Şamandıra (dönme koordinatı)
    buoy_x: float = 25.0   # m
    buoy_y: float = -10.0  # m
    
    # Bitiş kare merkezi
    end_x: float = 45.0   # m
    end_y: float = 5.0    # m
    
    # Kare boyutu
    square_half_size: float = 3.0  # m (±3m)

    # Operasyonel derinlik
    op_depth: float = -3.0  # m (negatif = su altı)
    
    # Şamandıra çevresi yarıçapı
    circle_radius: float = 3.5  # m (şamandıradan uzaklık)
    
    # Dönüş yönü (saat yönü = -1, ters = +1)
    circle_direction: int = 1


# ═══════════════════════════════════════════════════════════════════
# ANA GÖREV DÜĞÜMÜ
# ═══════════════════════════════════════════════════════════════════

class YavuzMissionNode(Node):
    """
    TEKNOFEST 2026 Tema 2 otonom görev yürütücüsü.
    
    Bu düğüm tüm görev mantığını, durum makinesini ve
    düşük seviye kontrol döngüsünü içerir.
    """

    # ——— Sabitler ———
    CONTROL_RATE_HZ = 50.0          # Kontrol döngüsü frekansı
    MISSION_TIMEOUT_S = 295.0       # Görev süresi (5dk - 5sn güvenlik)
    DIVE_DEPTH_M = -3.0             # Operasyonel derinlik
    SURFACE_DEPTH_M = -0.2          # Yüzey derinliği (kare içinde çıkış)
    
    # Fiziksel parametre sınırları
    MAX_SURGE = 30.0   # N
    MAX_SWAY  = 20.0   # N
    MAX_YAW   = 10.0   # N·m
    MAX_HEAVE = 15.0   # N

    def __init__(self):
        super().__init__('yavuz_mission_node')

        # ——— Parametreler ———
        self.declare_parameter('buoy_x', 25.0)
        self.declare_parameter('buoy_y', -10.0)
        self.declare_parameter('end_x', 45.0)
        self.declare_parameter('end_y', 5.0)
        self.declare_parameter('op_depth', -3.0)
        self.declare_parameter('circle_radius', 3.5)
        self.declare_parameter('circle_direction', 1)
        self.declare_parameter('auto_start', False)

        self.coords = MissionCoords(
            buoy_x=self.get_parameter('buoy_x').value,
            buoy_y=self.get_parameter('buoy_y').value,
            end_x=self.get_parameter('end_x').value,
            end_y=self.get_parameter('end_y').value,
            op_depth=self.get_parameter('op_depth').value,
            circle_radius=self.get_parameter('circle_radius').value,
            circle_direction=self.get_parameter('circle_direction').value,
        )

        # ——— Durum Makinesi ———
        self.state = MissionState.INIT
        self._prev_state = None
        self._state_entry_time: float = 0.0
        self._mission_start_time: Optional[float] = None
        self._mission_score: float = 0.0

        # ——— Araç durumu ———
        self.pose_x: float = 0.0
        self.pose_y: float = 0.0
        self.pose_z: float = 0.0
        self.roll:   float = 0.0
        self.pitch:  float = 0.0
        self.yaw:    float = 0.0
        self.vel_x:  float = 0.0
        self.vel_y:  float = 0.0
        self.vel_z:  float = 0.0
        self._pose_received = False
        self._emergency_stop = False
        self._last_odom_time: float = 0.0

        # ——— Şamandıra dönüşü sayacı ———
        self._circle_total_angle: float = 0.0
        self._circle_last_angle:  Optional[float] = None
        self._circle_start_pos:   Optional[Tuple[float, float]] = None
        self._circle_complete:    bool = False

        # ——— Bitiş kare durumu ———
        self._in_square: bool = False
        self._surfaced_in_square: bool = False

        # ——— Kontrol nesneleri ———
        self._init_controllers()
        self._allocator = ThrusterAllocator()

        # ——— Son kontrol komutu ———
        self._last_cmd = Twist()

        # ——— QoS ———
        reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        sensor   = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        # ——— Subscribers ———
        self.odom_sub = self.create_subscription(
            Odometry, '/yavuz/localization/odom',
            self._odom_callback, sensor)
        
        self.estop_sub = self.create_subscription(
            Bool, '/yavuz/emergency_stop',
            self._estop_callback, reliable)

        self.cmd_state_sub = self.create_subscription(
            String, '/yavuz/mission/command',
            self._command_callback, reliable)

        # ——— Publishers ———
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/yavuz/cmd_vel', reliable)
        
        self.state_pub = self.create_publisher(
            String, '/yavuz/mission/state', reliable)
        
        self.score_pub = self.create_publisher(
            Float64, '/yavuz/mission/score', reliable)
        
        self.depth_setpoint_pub = self.create_publisher(
            Float64, '/yavuz/control/depth_setpoint', reliable)

        # Thruster komutları (simülasyon için)
        self.thruster_pubs = [
            self.create_publisher(Float64, f'/yavuz/thruster{i}/cmd', reliable)
            for i in range(6)
        ]

        # ——— Kontrol döngüsü zamanlayıcısı ———
        dt = 1.0 / self.CONTROL_RATE_HZ
        self.control_timer = self.create_timer(dt, self._control_loop)

        # ——— Durum yayın zamanlayıcısı ———
        self.status_timer = self.create_timer(1.0, self._publish_status)

        # ——— Otomatik başlatma ———
        if self.get_parameter('auto_start').value:
            self.create_timer(2.0, lambda: self._transition(MissionState.DIVE))

        self.get_logger().info("=" * 55)
        self.get_logger().info(" YAVUZ AUV - TEKNOFEST 2026 - Tema 2")
        self.get_logger().info(f" Şamandıra: ({self.coords.buoy_x}, {self.coords.buoy_y})")
        self.get_logger().info(f" Bitiş: ({self.coords.end_x}, {self.coords.end_y})")
        self.get_logger().info(f" Derinlik: {self.coords.op_depth} m")
        self.get_logger().info("=" * 55)
        self.get_logger().info("Görev başlatmak için: ros2 topic pub /yavuz/mission/command std_msgs/String '{data: start}'")

    # ══════════════════════════════════════════════════════════════
    # KONTROL BAŞLATMA
    # ══════════════════════════════════════════════════════════════

    def _init_controllers(self):
        """PID kontrolcüleri başlat."""
        dt = 1.0 / self.CONTROL_RATE_HZ
        
        # Derinlik kontrolcüsü (Z ekseni)
        self.depth_pid = PIDController(
            PIDConfig(kp=15.0, ki=0.8, kd=3.0,
                      output_min=-self.MAX_HEAVE,
                      output_max=self.MAX_HEAVE,
                      integral_min=-5.0, integral_max=5.0,
                      derivative_filter_tau=0.05,
                      deadband=0.05),
            name="depth"
        )

        # Yön (heading) kontrolcüsü
        self.heading_pid = AngularPIDController(
            PIDConfig(kp=8.0, ki=0.3, kd=1.5,
                      output_min=-self.MAX_YAW,
                      output_max=self.MAX_YAW,
                      integral_min=-3.0, integral_max=3.0,
                      derivative_filter_tau=0.08),
            name="heading"
        )

        # İleri-geri (surge) kontrolcüsü
        self.surge_pid = PIDController(
            PIDConfig(kp=10.0, ki=0.2, kd=1.0,
                      output_min=-self.MAX_SURGE,
                      output_max=self.MAX_SURGE,
                      integral_min=-5.0, integral_max=5.0,
                      derivative_filter_tau=0.05),
            name="surge"
        )

        # Yan (sway) kontrolcüsü — yatay dönüş için
        self.sway_pid = PIDController(
            PIDConfig(kp=8.0, ki=0.1, kd=0.8,
                      output_min=-self.MAX_SWAY,
                      output_max=self.MAX_SWAY,
                      integral_min=-3.0, integral_max=3.0),
            name="sway"
        )

        # Roll ve pitch stabilizasyonu
        self.roll_pid = PIDController(
            PIDConfig(kp=5.0, ki=0.0, kd=1.0,
                      output_min=-8.0, output_max=8.0),
            name="roll"
        )
        self.pitch_pid = PIDController(
            PIDConfig(kp=5.0, ki=0.0, kd=1.0,
                      output_min=-8.0, output_max=8.0),
            name="pitch"
        )

        self._dt = 1.0 / self.CONTROL_RATE_HZ

    # ══════════════════════════════════════════════════════════════
    # CALLBACK'LER
    # ══════════════════════════════════════════════════════════════

    def _odom_callback(self, msg: Odometry):
        """EKF'ten gelen konum güncelleme."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear

        self.pose_x = p.x
        self.pose_y = p.y
        self.pose_z = p.z
        self.vel_x  = v.x
        self.vel_y  = v.y
        self.vel_z  = v.z

        # Quaternion → Euler
        self.roll, self.pitch, self.yaw = self._quat_to_euler(
            q.x, q.y, q.z, q.w)

        self._pose_received = True
        self._last_odom_time = self.get_clock().now().nanoseconds / 1e9

    def _estop_callback(self, msg: Bool):
        """Acil durdurma sinyali."""
        if msg.data:
            self.get_logger().error("ACİL DURDURMA TETIKLENDI!")
            self._emergency_stop = True
            self._transition(MissionState.EMERGENCY_STOP)

    def _command_callback(self, msg: String):
        """Operatör komutları."""
        cmd = msg.data.lower().strip()
        if cmd == 'start':
            if self.state == MissionState.INIT:
                self.get_logger().info("Görev başlatılıyor...")
                self._transition(MissionState.DIVE)
        elif cmd == 'abort':
            self._transition(MissionState.EMERGENCY_STOP)
        elif cmd == 'reset':
            self._reset_mission()
        self.get_logger().info(f"Komut alındı: {cmd}")

    # ══════════════════════════════════════════════════════════════
    # ANA KONTROL DÖNGÜSÜ
    # ══════════════════════════════════════════════════════════════

    def _control_loop(self):
        """50 Hz kontrol döngüsü — her tick çalışır."""

        # Acil durdurma her şeyden önce gelir
        if self._emergency_stop or self.state == MissionState.EMERGENCY_STOP:
            self._execute_emergency_stop()
            return

        # Görev timeout kontrolü
        if self._mission_start_time is not None:
            elapsed = self.get_clock().now().nanoseconds / 1e9 - self._mission_start_time
            if elapsed > self.MISSION_TIMEOUT_S:
                self.get_logger().warn("GÖREV TIMEOUT!")
                self._transition(MissionState.HOLD_POSITION)

        if not self._pose_received:
            return

        # ——— Durum makinesini çalıştır ———
        if   self.state == MissionState.INIT:
            self._execute_init()
        elif self.state == MissionState.DIVE:
            self._execute_dive()
        elif self.state == MissionState.TRANSIT_TO_BUOY:
            self._execute_transit_to_buoy()
        elif self.state == MissionState.ALIGN_TO_BUOY:
            self._execute_align_to_buoy()
        elif self.state == MissionState.CIRCLE_BUOY:
            self._execute_circle_buoy()
        elif self.state == MissionState.TRANSIT_TO_END:
            self._execute_transit_to_end()
        elif self.state == MissionState.SURFACE_IN_SQUARE:
            self._execute_surface_in_square()
        elif self.state == MissionState.HOLD_POSITION:
            self._execute_hold()
        elif self.state == MissionState.MISSION_COMPLETE:
            pass  # Bekle
        
    # ══════════════════════════════════════════════════════════════
    # DURUM YÜRÜTÜCÜLER
    # ══════════════════════════════════════════════════════════════

    def _execute_init(self):
        """INIT: Sistem hazırlık kontrolü."""
        # Hiçbir hareket yok, sadece sensör kontrolü
        self._send_zero_command()

        if self._pose_received:
            self.get_logger().info("Sistem hazır. 'start' komutu bekleniyor...")

    def _execute_dive(self):
        """DIVE: Operasyonel derinliğe in."""
        target_depth = self.coords.op_depth
        
        # Derinlik kontrolü
        heave = self.depth_pid.update(
            setpoint=target_depth,
            measurement=self.pose_z,
            dt=self._dt
        )
        
        # Roll/Pitch stabilizasyon
        roll_torque  = self.roll_pid.update(0.0, self.roll, self._dt)
        pitch_torque = self.pitch_pid.update(0.0, self.pitch, self._dt)
        
        # Heading: mevcut yönü koru
        yaw_torque = self.heading_pid.update(0.0, self.yaw, self._dt)
        
        self._send_wrench(surge=0.0, sway=0.0, heave=heave,
                          roll=roll_torque, pitch=pitch_torque, yaw=yaw_torque)

        # Derinliğe ulaştık mı?
        depth_error = abs(self.pose_z - target_depth)
        if depth_error < 0.3:
            self.get_logger().info(f"Operasyonel derinliğe ulaşıldı: {self.pose_z:.2f}m")
            self._transition(MissionState.TRANSIT_TO_BUOY)

    def _execute_transit_to_buoy(self):
        """TRANSIT_TO_BUOY: Şamandıra konumuna git."""
        # Şamandıranın yakınına bir waypoint (çevre noktası üzerinde)
        target_x, target_y = self._get_approach_point(
            self.coords.buoy_x, self.coords.buoy_y,
            self.coords.circle_radius + 1.0
        )
        
        done = self._navigate_to_point(
            target_x, target_y, self.coords.op_depth,
            tolerance=1.5
        )

        if done:
            self.get_logger().info("Şamandıraya yaklaşıldı")
            self._transition(MissionState.ALIGN_TO_BUOY)

    def _execute_align_to_buoy(self):
        """ALIGN_TO_BUOY: Dönüş yörüngesine hizalan."""
        # Şamandıradan circle_radius uzaklıkta bir nokta bul
        target_x, target_y = self._get_circle_start_point()
        
        done = self._navigate_to_point(
            target_x, target_y, self.coords.op_depth,
            tolerance=0.8
        )

        if done:
            self._circle_total_angle = 0.0
            self._circle_last_angle = self._angle_to_buoy()
            self._circle_start_pos = (self.pose_x, self.pose_y)
            self.get_logger().info("Dönüş başlangıç pozisyonu hazır. Şamandıra etrafında dönüş başlıyor...")
            self._transition(MissionState.CIRCLE_BUOY)

    def _execute_circle_buoy(self):
        """
        CIRCLE_BUOY: Şamandıra etrafında tam 360° dönüş — 40 PUAN
        
        Strateji: Şamandıra merkezinden sabit yarıçapta dairesel yörünge.
        - Şamandıraya dönük heading yaw kontrolü
        - Radyal mesafe hata → sway kontrolü  
        - Yörüngede ilerleme → surge kontrolü
        """
        buoy_x = self.coords.buoy_x
        buoy_y = self.coords.buoy_y
        R = self.coords.circle_radius
        direction = self.coords.circle_direction  # +1 veya -1

        # Mevcut açı (şamandıraya göre)
        current_angle = self._angle_to_buoy()
        
        # Toplam dönüş açısını güncelle
        if self._circle_last_angle is not None:
            d_angle = self._wrap_angle(current_angle - self._circle_last_angle)
            # Yön doğru mu kontrol et
            if direction * d_angle < -math.pi:
                d_angle += direction * 2 * math.pi
            self._circle_total_angle += d_angle
        self._circle_last_angle = current_angle

        # Hedef konum (bir adım ileride, yörünge üzerinde)
        lookahead = 0.3  # rad
        next_angle = current_angle + direction * lookahead
        target_x = buoy_x + R * math.cos(next_angle)
        target_y = buoy_y + R * math.sin(next_angle)

        # Hedef yön (şamandıraya dik, yörünge yönünde)
        tangent_angle = next_angle + direction * math.pi / 2
        
        # ——— Kontrol ———
        # Derinlik sabit tut
        heave = self.depth_pid.update(self.coords.op_depth, self.pose_z, self._dt)
        
        # Heading: yörünge teğetine bak
        yaw_torque = self.heading_pid.update(tangent_angle, self.yaw, self._dt)
        
        # Surge: sabit ilerle
        surge_force = 8.0  # N, hafif ilerleme

        # Radyal mesafe hatası: sway ile düzelt
        dx = self.pose_x - buoy_x
        dy = self.pose_y - buoy_y
        current_r = math.sqrt(dx**2 + dy**2)
        radial_error = R - current_r  # pozitif = içeri doğru çek
        
        # Radyal yön vektörü (şamandıradan dışa)
        if current_r > 0.01:
            radial_dir_x = dx / current_r
            radial_dir_y = dy / current_r
        else:
            radial_dir_x, radial_dir_y = 1.0, 0.0

        # Body frame'de sway hatası
        sway_error = radial_error * (
            radial_dir_x * math.sin(self.yaw) - radial_dir_y * math.cos(self.yaw))
        sway_force = self.sway_pid.update(0.0, -sway_error, self._dt)

        # Roll/Pitch stabilizasyon
        roll_torque  = self.roll_pid.update(0.0, self.roll, self._dt)
        pitch_torque = self.pitch_pid.update(0.0, self.pitch, self._dt)

        self._send_wrench(surge=surge_force, sway=sway_force, heave=heave,
                          roll=roll_torque, pitch=pitch_torque, yaw=yaw_torque)

        # 360° tamamlandı mı? (tolerans ±5°)
        if abs(self._circle_total_angle) >= math.radians(355):
            self._circle_complete = True
            self._mission_score += 40.0
            self.get_logger().info(
                f"✓ ŞAMANDIRA DÖNÜŞÜ TAMAMLANDI! +40 puan. Toplam: {self._mission_score}"
            )
            self._transition(MissionState.TRANSIT_TO_END)

        # Debug log
        self.get_logger().debug(
            f"Dönüş: {math.degrees(self._circle_total_angle):.1f}° / 360°, "
            f"R={current_r:.2f}m (hedef={R})"
        )

    def _execute_transit_to_end(self):
        """TRANSIT_TO_END: Bitiş koordinatına git (su altı kalarak)."""
        # Bitiş karesine yaklaş
        target_x = self.coords.end_x
        target_y = self.coords.end_y
        
        done = self._navigate_to_point(
            target_x, target_y, self.coords.op_depth,
            tolerance=self.coords.square_half_size * 0.8  # Kare içine gir
        )

        # Kare içinde miyiz?
        self._in_square = self._check_in_square()

        if done and self._in_square:
            self.get_logger().info("Bitiş karesine ulaşıldı. Yüzeye çıkılıyor...")
            self._transition(MissionState.SURFACE_IN_SQUARE)

    def _execute_surface_in_square(self):
        """
        SURFACE_IN_SQUARE: Kare içinde yüzeye çık — 60 PUAN
        
        Araç yalnızca kare içinde yüzeye çıkabilir.
        Kare dışına çıkarsa puan alamaz.
        """
        # Kare içinde miyiz?
        still_in_square = self._check_in_square()
        
        if not still_in_square:
            self.get_logger().warn("Kare dışına çıkıldı! Geri dönülüyor...")
            # Merkeze geri git
            self._navigate_to_point(
                self.coords.end_x, self.coords.end_y,
                self.coords.op_depth, tolerance=1.0
            )
            return

        # Yüzeye çık
        heave = self.depth_pid.update(
            setpoint=self.SURFACE_DEPTH_M,
            measurement=self.pose_z,
            dt=self._dt
        )
        
        # Kare merkezinde kal (X-Y)
        dx = self.coords.end_x - self.pose_x
        dy = self.coords.end_y - self.pose_y
        heading_to_center = math.atan2(dy, dx)
        dist_to_center = math.sqrt(dx**2 + dy**2)
        
        if dist_to_center > 1.0:
            surge = min(5.0, dist_to_center * 2.0)
            yaw_torque = self.heading_pid.update(heading_to_center, self.yaw, self._dt)
        else:
            surge = 0.0
            yaw_torque = 0.0

        roll_torque  = self.roll_pid.update(0.0, self.roll, self._dt)
        pitch_torque = self.pitch_pid.update(0.0, self.pitch, self._dt)

        self._send_wrench(surge=surge, sway=0.0, heave=heave,
                          roll=roll_torque, pitch=pitch_torque, yaw=yaw_torque)

        # Yüzeye ulaştık mı?
        if self.pose_z > self.SURFACE_DEPTH_M - 0.15:
            self._surfaced_in_square = True
            self._mission_score += 60.0
            
            # Süre bonusu
            if self._mission_start_time is not None:
                elapsed = self.get_clock().now().nanoseconds / 1e9 - self._mission_start_time
                remaining = max(0.0, 300.0 - elapsed)
                time_bonus = self._mission_score * (remaining / 300.0)
                self._mission_score += time_bonus
                
                self.get_logger().info(
                    f"★★★ GÖREV TAMAMLANDI! ★★★\n"
                    f"  Temel puan : {self._mission_score - time_bonus:.1f}\n"
                    f"  Süre bonusu: +{time_bonus:.1f}\n"
                    f"  Toplam     : {self._mission_score:.1f}\n"
                    f"  Süre       : {elapsed:.1f}s"
                )
            
            self._transition(MissionState.MISSION_COMPLETE)

    def _execute_hold(self):
        """Mevcut pozisyonu koru."""
        heave = self.depth_pid.update(self.pose_z, self.pose_z, self._dt)
        yaw_torque = self.heading_pid.update(self.yaw, self.yaw, self._dt)
        self._send_wrench(surge=0.0, sway=0.0, heave=heave,
                          roll=0.0, pitch=0.0, yaw=yaw_torque)

    def _execute_emergency_stop(self):
        """TÜM THRUSTERLARI DURDUR."""
        self._send_zero_command()
        self.get_logger().error("EMERGENCY STOP — Tüm thrusterlar durduruldu.", once=True)

    # ══════════════════════════════════════════════════════════════
    # NAVİGASYON YARDIMCILARI
    # ══════════════════════════════════════════════════════════════

    def _navigate_to_point(self, target_x: float, target_y: float,
                            target_z: float, tolerance: float = 1.0) -> bool:
        """
        Belirtilen noktaya ilerle. Hedefe ulaşıldığında True döner.
        
        Strateji: Hedef yöne bak (yaw PID) → ileri git (surge PID) + derinlik tut
        """
        dx = target_x - self.pose_x
        dy = target_y - self.pose_y
        dz = target_z - self.pose_z
        dist_horiz = math.sqrt(dx**2 + dy**2)
        dist_total = math.sqrt(dx**2 + dy**2 + dz**2)

        if dist_total < tolerance:
            self._send_zero_command()
            return True

        # Hedef yön
        target_heading = math.atan2(dy, dx)
        heading_error = self._wrap_angle(target_heading - self.yaw)

        # Derinlik kontrolü
        heave = self.depth_pid.update(target_z, self.pose_z, self._dt)
        
        # Yön kontrolü
        yaw_torque = self.heading_pid.update(target_heading, self.yaw, self._dt)
        
        # Surge: hedefe yöneldiysek ileri git
        if abs(heading_error) < math.radians(30):
            surge_sp = min(dist_horiz, 4.0)  # max 4m/s
            surge = self.surge_pid.update(surge_sp, self.vel_x, self._dt)
        else:
            surge = 0.0  # Önce dönsün

        roll_torque  = self.roll_pid.update(0.0, self.roll, self._dt)
        pitch_torque = self.pitch_pid.update(0.0, self.pitch, self._dt)

        self._send_wrench(surge=surge, sway=0.0, heave=heave,
                          roll=roll_torque, pitch=pitch_torque, yaw=yaw_torque)

        self.get_logger().debug(
            f"Navigasyon: dx={dx:.1f} dy={dy:.1f} dz={dz:.1f} "
            f"dist={dist_total:.1f}m heading_err={math.degrees(heading_error):.1f}°"
        )
        return False

    def _get_approach_point(self, cx: float, cy: float, approach_dist: float) -> Tuple[float, float]:
        """Mevcut konumdan hedefe yaklaşma noktasını hesapla."""
        dx = cx - self.pose_x
        dy = cy - self.pose_y
        dist = math.sqrt(dx**2 + dy**2)
        if dist < 0.01:
            return cx + approach_dist, cy
        # Hedefin arkasına (mevcut konumdan) approach_dist uzaklıkta nokta
        target_x = cx - (dx / dist) * approach_dist
        target_y = cy - (dy / dist) * approach_dist
        return target_x, target_y

    def _get_circle_start_point(self) -> Tuple[float, float]:
        """Şamandıra etrafında başlangıç noktasını bul."""
        angle = self._angle_to_buoy()
        R = self.coords.circle_radius
        target_x = self.coords.buoy_x + R * math.cos(angle)
        target_y = self.coords.buoy_y + R * math.sin(angle)
        return target_x, target_y

    def _angle_to_buoy(self) -> float:
        """Mevcut konumdan şamandıraya açı (radyan)."""
        dx = self.pose_x - self.coords.buoy_x
        dy = self.pose_y - self.coords.buoy_y
        return math.atan2(dy, dx)

    def _check_in_square(self) -> bool:
        """Araç bitiş karesi içinde mi?"""
        hs = self.coords.square_half_size
        in_x = abs(self.pose_x - self.coords.end_x) <= hs
        in_y = abs(self.pose_y - self.coords.end_y) <= hs
        return in_x and in_y

    # ══════════════════════════════════════════════════════════════
    # KONTROL ÇIKIŞ YARDIMCILARI
    # ══════════════════════════════════════════════════════════════

    def _send_wrench(self, surge, sway, heave, roll, pitch, yaw):
        """Wrench'i thruster komutlarına çevir ve yayınla."""
        thrusts = self._allocator.allocate(
            surge=surge, sway=sway, heave=heave,
            roll=roll, pitch=pitch, yaw=yaw
        )
        
        # Thruster komutlarını yayınla
        for i, f in enumerate(thrusts):
            msg = Float64()
            msg.data = float(f)
            self.thruster_pubs[i].publish(msg)

        # cmd_vel olarak da yayınla (görselleştirme için)
        twist = Twist()
        twist.linear.x = surge
        twist.linear.y = sway
        twist.linear.z = heave
        twist.angular.x = roll
        twist.angular.y = pitch
        twist.angular.z = yaw
        self.cmd_vel_pub.publish(twist)

    def _send_zero_command(self):
        """Tüm thrusterları durdur."""
        self._send_wrench(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    # ══════════════════════════════════════════════════════════════
    # DURUM MAKİNESİ YARDIMCILARI
    # ══════════════════════════════════════════════════════════════

    def _transition(self, new_state: MissionState):
        """Durum geçişini gerçekleştir ve loglama yap."""
        if new_state == self.state:
            return
        
        self.get_logger().info(
            f"Durum: {self.state.name} → {new_state.name}"
        )
        
        # Göreve başlama zamanını kaydet
        if new_state == MissionState.DIVE and self._mission_start_time is None:
            self._mission_start_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info("GÖREV SÜRESI BAŞLADI!")

        # PID resetleri (durum değişiminde)
        if new_state in (MissionState.TRANSIT_TO_BUOY,
                         MissionState.TRANSIT_TO_END,
                         MissionState.CIRCLE_BUOY):
            self.surge_pid.reset()
            self.heading_pid.reset()

        self._prev_state = self.state
        self.state = new_state
        self._state_entry_time = self.get_clock().now().nanoseconds / 1e9

    def _reset_mission(self):
        """Görev sıfırla."""
        self.state = MissionState.INIT
        self._mission_start_time = None
        self._mission_score = 0.0
        self._circle_total_angle = 0.0
        self._circle_complete = False
        self._surfaced_in_square = False
        self._emergency_stop = False
        
        for pid in [self.depth_pid, self.heading_pid,
                    self.surge_pid, self.sway_pid]:
            pid.reset()
        
        self._send_zero_command()
        self.get_logger().info("Görev sıfırlandı.")

    def _publish_status(self):
        """Durum bilgisini yayınla."""
        state_msg = String()
        state_msg.data = self.state.name
        self.state_pub.publish(state_msg)

        score_msg = Float64()
        score_msg.data = self._mission_score
        self.score_pub.publish(score_msg)

        if self._mission_start_time is not None:
            elapsed = self.get_clock().now().nanoseconds / 1e9 - self._mission_start_time
            remaining = max(0.0, 300.0 - elapsed)
            self.get_logger().info(
                f"[{self.state.name}] Poz:({self.pose_x:.1f},{self.pose_y:.1f},{self.pose_z:.1f})m "
                f"Puan:{self._mission_score:.0f} Kalan:{remaining:.0f}s"
            )

    # ══════════════════════════════════════════════════════════════
    # YARDIMCI FONKSİYONLAR
    # ══════════════════════════════════════════════════════════════

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        while angle >  math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    @staticmethod
    def _quat_to_euler(qx, qy, qz, qw):
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx**2 + qy**2)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
        pitch = math.asin(sinp)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy**2 + qz**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw


# ══════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = YavuzMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Görev durduruldu (Ctrl+C)")
    finally:
        node._send_zero_command()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
