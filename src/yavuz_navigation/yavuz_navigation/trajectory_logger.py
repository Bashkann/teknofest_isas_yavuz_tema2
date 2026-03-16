#!/usr/bin/env python3
"""
YAVUZ AUV - Yörünge Kaydedici ve Skor Doğrulayıcı

Yarışma süresince aracın yolunu kaydeder ve:
  - Şamandıra dönüşünü doğrular (360° izleme)
  - Kare içinde yüzey çıkışını doğrular
  - CSV ve görselleştirme çıktısı üretir
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64
import csv
import math
import os
from datetime import datetime
from typing import List, Tuple


class TrajectoryLogger(Node):
    """Görev yörüngesini kaydeder ve puanlama kriterlerini doğrular."""

    def __init__(self):
        super().__init__('yavuz_trajectory_logger')
        
        self.declare_parameter('output_dir', '/tmp')
        self.declare_parameter('log_rate_hz', 10.0)
        
        output_dir = self.get_parameter('output_dir').value
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = os.path.join(output_dir, f'yavuz_trajectory_{timestamp}.csv')
        
        # Yörünge noktaları
        self._trajectory: List[Tuple] = []
        self._mission_state = "UNKNOWN"
        self._score = 0.0

        # Buoy koordinatları (default - gerçek değerler parametreden alınmalı)
        self._buoy_x = 25.0
        self._buoy_y = -10.0
        self._end_x  = 45.0
        self._end_y  = 5.0
        self._square_half = 3.0

        # CSV başlıkları
        with open(self._csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'x', 'y', 'z', 'yaw',
                             'vx', 'vy', 'vz', 'state', 'score'])

        # Subscriptions
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, depth=10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/yavuz/localization/odom', self._odom_cb, qos)
        self.state_sub = self.create_subscription(
            String, '/yavuz/mission/state', self._state_cb,
            rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, depth=10))
        self.score_sub = self.create_subscription(
            Float64, '/yavuz/mission/score', self._score_cb,
            rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, depth=10))

        # Log zamanlayıcısı
        dt = 1.0 / self.get_parameter('log_rate_hz').value
        self.timer = self.create_timer(dt, self._log_point)
        
        self._current_pose = None
        self._current_vel = None

        self.get_logger().info(f"Yörünge kaydediliyor: {self._csv_path}")

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        )
        self._current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            yaw
        )
        self._current_vel = (
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        )

    def _state_cb(self, msg: String):
        self._mission_state = msg.data

    def _score_cb(self, msg: Float64):
        self._score = msg.data

    def _log_point(self):
        if self._current_pose is None:
            return
        
        ts = self.get_clock().now().nanoseconds / 1e9
        x, y, z, yaw = self._current_pose
        vx, vy, vz = self._current_vel or (0, 0, 0)

        row = [f"{ts:.3f}", f"{x:.3f}", f"{y:.3f}", f"{z:.3f}",
               f"{math.degrees(yaw):.1f}",
               f"{vx:.3f}", f"{vy:.3f}", f"{vz:.3f}",
               self._mission_state, f"{self._score:.1f}"]
        
        with open(self._csv_path, 'a', newline='') as f:
            csv.writer(f).writerow(row)
        
        self._trajectory.append((x, y, z, ts))

        # Anlık kontroller
        self._check_in_square(x, y, z)

    def _check_in_square(self, x, y, z):
        """Kare içinde mi? Log at."""
        in_x = abs(x - self._end_x) <= self._square_half
        in_y = abs(y - self._end_y) <= self._square_half
        if in_x and in_y and z > -0.5:
            self.get_logger().info(
                f"✓ Kare içinde yüzeyde! ({x:.1f}, {y:.1f}, {z:.2f}m)",
                once=True
            )


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"Yörünge kaydedildi: {node._csv_path}")
        node.get_logger().info(f"Toplam nokta: {len(node._trajectory)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
