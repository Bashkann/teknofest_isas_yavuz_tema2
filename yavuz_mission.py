#!/usr/bin/env python3
"""
YAVUZ AUV - TEKNOFEST 2026 Tema 2
Tek dosya, çalışan versiyon.

Çalıştırma:
  Terminal 1: ros2 launch yavuz_bringup simulation.launch.py
  Terminal 2: python3 yavuz_mission.py
  Terminal 3: ros2 topic pub /yavuz/mission/command std_msgs/msg/String '{data: start}' --once
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
import subprocess
import re
import threading

from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu


# ══════════════════════════════════════════════════════════
# PID KONTROLCÜ
# ══════════════════════════════════════════════════════════
class PID:
    def __init__(self, kp, ki, kd, out_min, out_max, deadband=0.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.deadband = deadband
        self._integral = 0.0
        self._prev_err = 0.0

    def update(self, setpoint, measurement, dt):
        if dt <= 0: return 0.0
        err = setpoint - measurement
        if abs(err) < self.deadband:
            err = 0.0
        self._integral = max(-10.0, min(10.0, self._integral + err * dt))
        d = (err - self._prev_err) / dt
        self._prev_err = err
        out = self.kp * err + self.ki * self._integral + self.kd * d
        return max(self.out_min, min(self.out_max, out))

    def reset(self):
        self._integral = 0.0
        self._prev_err = 0.0


class AngPID(PID):
    def update(self, setpoint, measurement, dt):
        err = setpoint - measurement
        while err >  math.pi: err -= 2*math.pi
        while err < -math.pi: err += 2*math.pi
        return super().update(err, 0.0, dt)


# ══════════════════════════════════════════════════════════
# THRUSTER ALLOCATOR
# ══════════════════════════════════════════════════════════
import numpy as np

class Allocator:
    TMAX = 30.0

    def __init__(self):
        c = math.cos
        s = math.sin
        d = math.pi
        cfg = [
            (0.18,  0.11, 0.0, 0.0, 0.0, 3*d/4),
            (0.18, -0.11, 0.0, 0.0, 0.0,-3*d/4),
            (-0.18, 0.11, 0.0, 0.0, 0.0,   d/4),
            (-0.18,-0.11, 0.0, 0.0, 0.0,  -d/4),
            (0.12,  0.22, 0.0, 0.0, d/2,   0.0),
            (0.12, -0.22, 0.0, 0.0, d/2,   0.0),
        ]
        B = np.zeros((6, 6))
        for i, (x, y, z, r, p, yaw) in enumerate(cfg):
            cy, sy = c(yaw), s(yaw)
            cp, sp = c(p), s(p)
            fx, fy, fz = cy*cp, sy*cp, -sp
            B[0,i]=fx; B[1,i]=fy; B[2,i]=fz
            B[3,i]=y*fz-z*fy; B[4,i]=z*fx-x*fz; B[5,i]=x*fy-y*fx
        self.Bpinv = np.linalg.pinv(B)

    def allocate(self, surge, sway, heave, roll, pitch, yaw):
        w = np.array([surge, sway, heave, roll, pitch, yaw])
        t = self.Bpinv @ w
        mx = max(abs(t.max()), abs(t.min()), self.TMAX)
        if mx > self.TMAX:
            t = t * self.TMAX / mx
        return np.clip(t, -self.TMAX, self.TMAX)


# ══════════════════════════════════════════════════════════
# ANA DÜĞÜM
# ══════════════════════════════════════════════════════════
class YavuzNode(Node):

    STATES = ['INIT','DIVE','TRANSIT_BUOY','ALIGN_BUOY',
              'CIRCLE_BUOY','TRANSIT_END','SURFACE','DONE','ESTOP']

    def __init__(self):
        super().__init__('yavuz_mission_node')

        # Parametreler
        self.buoy_x  = 25.0
        self.buoy_y  = -10.0
        self.end_x   = 45.0
        self.end_y   = 5.0
        self.op_depth = -3.0
        self.circle_r = 3.5
        self.sq_half  = 3.0

        # Durum
        self.state = 'INIT'
        self.x = self.y = self.z = 0.0
        self.roll = self.pitch = self.yaw = 0.0
        self.vx = self.vy = self.vz = 0.0
        self._pose_ok = False
        self._start_time = None
        self._score = 0.0
        self._circle_angle = 0.0
        self._circle_last  = None
        self._dt = 0.02

        # PID'ler
        self.pid_depth   = PID(12.0, 0.5, 2.0, -15, 15, deadband=0.05)
        self.pid_heading = AngPID(7.0, 0.2, 1.2, -10, 10)
        self.pid_surge   = PID(8.0, 0.1, 0.8, -25, 25)
        self.pid_sway    = PID(6.0, 0.1, 0.5, -15, 15)

        self.alloc = Allocator()

        # QoS
        be  = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)

        # Subscribers
        self.create_subscription(TFMessage,
            '/world/teknofest_theme2/dynamic_pose/info', self._pose_cb, be)
        self.create_subscription(String,
            '/yavuz/mission/command', self._cmd_cb, rel)
        self.create_subscription(Bool,
            '/yavuz/emergency_stop', self._estop_cb, rel)

        # Publishers
        self.thruster_pubs = [
            self.create_publisher(Float64, f'/yavuz/thruster{i}/cmd', rel)
            for i in range(6)
        ]
        self.cmd_vel_pub = self.create_publisher(Twist, '/yavuz/cmd_vel', rel)
        self.state_pub   = self.create_publisher(String, '/yavuz/mission/state', rel)
        self.score_pub   = self.create_publisher(Float64, '/yavuz/mission/score', rel)

        # Zamanlayıcılar
        self.create_timer(self._dt, self._control_loop)
        self.create_timer(1.0, self._status_log)

        self.get_logger().info("="*55)
        self.get_logger().info(" YAVUZ AUV - TEKNOFEST 2026 - Tema 2")
        self.get_logger().info(f" Buoy:({self.buoy_x},{self.buoy_y}) End:({self.end_x},{self.end_y})")
        self.get_logger().info(" 'start' komutu bekleniyor...")
        self.get_logger().info("="*55)

    # ── CALLBACKS ─────────────────────────────────────────

    def _pose_cb(self, msg: TFMessage):
        """Gazebo dynamic pose → araç konumu."""
        for tf in msg.transforms:
            t = tf.transform.translation
            r = tf.transform.rotation
            # Şamandıra değil, araç (0,0 civarında başlar)
            if abs(t.x) > 100 or abs(t.y) > 100:
                continue
            # Sadece yavuz_auv veya base_link
            cid = tf.child_frame_id
            if cid not in ('yavuz_auv','base_link','base_footprint',''):
                # thruster link'leri atla
                if any(s in cid for s in ['thruster','camera','imu']):
                    continue

            self.x, self.y, self.z = t.x, t.y, t.z
            roll, pitch, yaw = self._q2e(r.x, r.y, r.z, r.w)
            self.roll, self.pitch, self.yaw = roll, pitch, yaw
            self._pose_ok = True
            return

    def _cmd_cb(self, msg: String):
        cmd = msg.data.lower().strip()
        if cmd == 'start' and self.state == 'INIT':
            self._transition('DIVE')
        elif cmd == 'abort':
            self._transition('ESTOP')
        elif cmd == 'reset':
            self.state = 'INIT'
            self._start_time = None
            self._score = 0.0

    def _estop_cb(self, msg: Bool):
        if msg.data:
            self._transition('ESTOP')

    # ── KONTROL DÖNGÜSÜ ───────────────────────────────────

    def _control_loop(self):
        if not self._pose_ok:
            return

        if self.state == 'ESTOP':
            self._send(0,0,0,0,0,0); return

        # Timeout kontrolü
        if self._start_time:
            if (self.get_clock().now().nanoseconds/1e9 - self._start_time) > 295:
                self.get_logger().warn("TIMEOUT!")
                self._transition('DONE')

        if   self.state == 'INIT':           self._send(0,0,0,0,0,0)
        elif self.state == 'DIVE':           self._do_dive()
        elif self.state == 'TRANSIT_BUOY':   self._do_transit(self.buoy_x, self.buoy_y, 2.5, 'ALIGN_BUOY')
        elif self.state == 'ALIGN_BUOY':     self._do_align()
        elif self.state == 'CIRCLE_BUOY':    self._do_circle()
        elif self.state == 'TRANSIT_END':    self._do_transit(self.end_x, self.end_y, self.sq_half*0.8, 'SURFACE')
        elif self.state == 'SURFACE':        self._do_surface()
        elif self.state == 'DONE':           self._send(0,0,0,0,0,0)

    # ── DURUM YÜRÜTÜCÜLERİ ────────────────────────────────

    def _do_dive(self):
        heave = self.pid_depth.update(self.op_depth, self.z, self._dt)
        yaw_t = self.pid_heading.update(0.0, self.yaw, self._dt)
        self._send(0, 0, heave, 0, 0, yaw_t)
        if abs(self.z - self.op_depth) < 0.4:
            self.get_logger().info(f"Derinlige ulasıldı: {self.z:.2f}m")
            self._transition('TRANSIT_BUOY')

    def _do_transit(self, tx, ty, tol, next_state):
        dx = tx - self.x
        dy = ty - self.y
        dist = math.sqrt(dx**2 + dy**2)
        if dist < tol:
            self._transition(next_state)
            return
        th = math.atan2(dy, dx)
        yaw_t = self.pid_heading.update(th, self.yaw, self._dt)
        he = self.pid_depth.update(self.op_depth, self.z, self._dt)
        herr = self._wrap(th - self.yaw)
        surge = 8.0 if abs(herr) < math.radians(30) else 0.0
        self._send(surge, 0, he, 0, 0, yaw_t)

    def _do_align(self):
        """Şamandıradan circle_r uzaklıkta bir noktaya git."""
        ang = math.atan2(self.y - self.buoy_y, self.x - self.buoy_x)
        tx = self.buoy_x + self.circle_r * math.cos(ang)
        ty = self.buoy_y + self.circle_r * math.sin(ang)
        dx, dy = tx - self.x, ty - self.y
        dist = math.sqrt(dx**2 + dy**2)
        if dist < 0.8:
            self._circle_angle = 0.0
            self._circle_last  = ang
            self.get_logger().info("Dönus basliyor! +40 puan hedefleniyor...")
            self._transition('CIRCLE_BUOY')
            return
        th = math.atan2(dy, dx)
        yaw_t = self.pid_heading.update(th, self.yaw, self._dt)
        he = self.pid_depth.update(self.op_depth, self.z, self._dt)
        surge = 6.0 if abs(self._wrap(th-self.yaw)) < math.radians(30) else 0.0
        self._send(surge, 0, he, 0, 0, yaw_t)

    def _do_circle(self):
        """360° şamandıra dönüşü."""
        cur_ang = math.atan2(self.y - self.buoy_y, self.x - self.buoy_x)

        if self._circle_last is not None:
            d = self._wrap(cur_ang - self._circle_last)
            self._circle_angle += d
        self._circle_last = cur_ang

        # Hedef: bir adım ileride
        lookahead = 0.4
        next_ang = cur_ang + lookahead
        tx = self.buoy_x + self.circle_r * math.cos(next_ang)
        ty = self.buoy_y + self.circle_r * math.sin(next_ang)

        # Teğet yön
        tangent = next_ang + math.pi/2
        yaw_t = self.pid_heading.update(tangent, self.yaw, self._dt)
        he = self.pid_depth.update(self.op_depth, self.z, self._dt)

        # Radyal hata düzeltmesi
        dx = self.x - self.buoy_x
        dy = self.y - self.buoy_y
        cur_r = math.sqrt(dx**2 + dy**2)
        radial_err = self.circle_r - cur_r
        sway = self.pid_sway.update(0, -radial_err, self._dt)

        self._send(7.0, sway, he, 0, 0, yaw_t)

        deg = math.degrees(abs(self._circle_angle))
        if abs(self._circle_angle) >= math.radians(355):
            self._score += 40.0
            self.get_logger().info(f"✓ SAMANDIRA DONUSU TAMAMLANDI! +40 puan. Toplam:{self._score}")
            self._transition('TRANSIT_END')

        if int(deg) % 30 == 0:
            self.get_logger().info(f"Donus: {deg:.0f}° / 360°  R={cur_r:.2f}m")

    def _do_surface(self):
        """Kare içinde yüzeye çık."""
        in_sq = (abs(self.x-self.end_x) <= self.sq_half and
                 abs(self.y-self.end_y) <= self.sq_half)
        if not in_sq:
            self._do_transit(self.end_x, self.end_y, 1.0, 'SURFACE')
            return

        heave = self.pid_depth.update(-0.1, self.z, self._dt)
        self._send(0, 0, heave, 0, 0, 0)

        if self.z > -0.2:
            self._score += 60.0
            if self._start_time:
                elapsed = self.get_clock().now().nanoseconds/1e9 - self._start_time
                bonus = self._score * max(0, 300-elapsed) / 300.0
                self._score += bonus
            self.get_logger().info(f"★★★ GOREV TAMAMLANDI! Toplam puan: {self._score:.1f} ★★★")
            self._transition('DONE')

    # ── YARDIMCI FONKSİYONLAR ─────────────────────────────

    def _send(self, surge, sway, heave, roll, pitch, yaw):
        thrusts = self.alloc.allocate(surge, sway, heave, roll, pitch, yaw)
        for i, f in enumerate(thrusts):
            m = Float64(); m.data = float(f)
            self.thruster_pubs[i].publish(m)
        t = Twist()
        t.linear.x=surge; t.linear.y=sway; t.linear.z=heave
        t.angular.x=roll; t.angular.y=pitch; t.angular.z=yaw
        self.cmd_vel_pub.publish(t)

    def _transition(self, new_state):
        if new_state == self.state: return
        self.get_logger().info(f"Durum: {self.state} → {new_state}")
        if new_state == 'DIVE' and not self._start_time:
            self._start_time = self.get_clock().now().nanoseconds/1e9
            self.get_logger().info("GOREV SURESI BASLADI!")
        self.pid_depth.reset()
        self.pid_heading.reset()
        self.state = new_state

    def _status_log(self):
        m = String(); m.data = self.state
        self.state_pub.publish(m)
        s = Float64(); s.data = self._score
        self.score_pub.publish(s)
        if self._start_time:
            elapsed = self.get_clock().now().nanoseconds/1e9 - self._start_time
            rem = max(0, 300-elapsed)
            self.get_logger().info(
                f"[{self.state}] Poz:({self.x:.1f},{self.y:.1f},{self.z:.1f}) "
                f"Puan:{self._score:.0f} Kalan:{rem:.0f}s"
            )

    @staticmethod
    def _wrap(a):
        while a >  math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

    @staticmethod
    def _q2e(qx, qy, qz, qw):
        sinr = 2*(qw*qx+qy*qz)
        cosr = 1-2*(qx*qx+qy*qy)
        roll = math.atan2(sinr, cosr)
        sinp = max(-1, min(1, 2*(qw*qy-qz*qx)))
        pitch = math.asin(sinp)
        siny = 2*(qw*qz+qx*qy)
        cosy = 1-2*(qy*qy+qz*qz)
        yaw = math.atan2(siny, cosy)
        return roll, pitch, yaw


# ══════════════════════════════════════════════════════════
def main():
    rclpy.init()
    node = YavuzNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Durduruldu.")
    finally:
        node._send(0,0,0,0,0,0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
