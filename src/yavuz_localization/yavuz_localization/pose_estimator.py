#!/usr/bin/env python3
"""
YAVUZ AUV - Su Altı Konum Tahmini (EKF Dead Reckoning)

GPS su altında çalışmaz. Bu düğüm:
  - BMX160 IMU (ivme + jiroskop + manyetometre)
  - Basınç sensörü (derinlik = Z ekseni)
  - Motor thrust modeli (hız tahmini yardımcı)
  
verilerini Extended Kalman Filter (EKF) ile birleştirerek
6-DOF konum tahmini üretir.

Durum vektörü: [x, y, z, roll, pitch, yaw, vx, vy, vz, bax, bay, baz]
               pos(3) + rpy(3) + vel(3) + acc_bias(3) = 12 boyut

NOT: Simülasyonda Gazebo ground truth + gürültü kullanılır.
     Gerçek donanımda tam EKF devreye girer.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
import math
from typing import Optional, Tuple

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import rclpy.time


class EKFDeadReckoning(Node):
    """
    Extended Kalman Filter tabanlı su altı konum tahmini.
    
    Topic Girişleri:
        /yavuz/imu/data          (sensor_msgs/Imu)
        /yavuz/depth/raw         (std_msgs/Float64)     — basınç sensöründen
        /yavuz/ground_truth/pose (nav_msgs/Odometry)    — simülasyon only
        
    Topic Çıkışları:
        /yavuz/localization/pose    (nav_msgs/Odometry)
        /yavuz/localization/depth   (std_msgs/Float64)
    """

    def __init__(self):
        super().__init__('yavuz_ekf_node')
        
        # ——— Parametreler ———
        self.declare_parameter('use_ground_truth', True)   # Sim: True, Gerçek: False
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('imu_rate', 100.0)          # Hz
        self.declare_parameter('depth_noise_std', 0.005)   # m
        self.declare_parameter('accel_noise_std', 0.021)   # m/s² (BMX160 datasheet)
        self.declare_parameter('gyro_noise_std', 0.009)    # rad/s
        self.declare_parameter('accel_bias_std', 0.001)
        
        self.use_gt = self.get_parameter('use_ground_truth').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # ——— EKF Durum Vektörü ———
        # x = [px, py, pz, roll, pitch, yaw, vx, vy, vz, bax, bay, baz]
        self.n_states = 12
        self.x = np.zeros(self.n_states)
        self.P = np.eye(self.n_states) * 0.01  # Kovaryans matrisi

        # Süreç gürültüsü
        q_pos   = 0.001
        q_vel   = 0.01
        q_angle = 0.001
        q_bias  = 0.0001
        self.Q = np.diag([
            q_pos, q_pos, q_pos,        # pozisyon
            q_angle, q_angle, q_angle,  # açı
            q_vel, q_vel, q_vel,        # hız
            q_bias, q_bias, q_bias      # ivme bias
        ])

        # Ölçüm gürültüsü (derinlik)
        self.R_depth = np.array([[self.get_parameter('depth_noise_std').value ** 2]])

        # ——— IMU verisi ———
        self._last_imu: Optional[Imu] = None
        self._last_imu_time: Optional[float] = None
        self._imu_initialized = False

        # ——— Ground truth (simülasyon) ———
        self._gt_pose: Optional[Odometry] = None

        # ——— QoS ———
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )

        # ——— Subscribers ———
        self.imu_sub = self.create_subscription(
            Imu, '/yavuz/imu/data',
            self._imu_callback, sensor_qos)
        
        self.depth_sub = self.create_subscription(
            Float64, '/yavuz/depth/pressure',
            self._depth_callback, sensor_qos)
        
        if self.use_gt:
            self.gt_sub = self.create_subscription(
                Odometry, '/yavuz/ground_truth/odom',
                self._ground_truth_callback, reliable_qos)

        # ——— Publishers ———
        self.pose_pub = self.create_publisher(
            Odometry, '/yavuz/localization/odom', reliable_qos)
        
        self.depth_pub = self.create_publisher(
            Float64, '/yavuz/localization/depth', reliable_qos)

        # ——— TF Broadcaster ———
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # ——— Zamanlayıcı ———
        self.pub_timer = self.create_timer(0.02, self._publish_estimate)  # 50 Hz

        self.get_logger().info("EKF Dead Reckoning başlatıldı (use_gt={})".format(self.use_gt))

    # ================================================================
    # CALLBACK'LER
    # ================================================================

    def _imu_callback(self, msg: Imu):
        """IMU verisi gelince EKF predict adımı."""
        now = self.get_clock().now().nanoseconds / 1e9

        if self._last_imu_time is None:
            self._last_imu_time = now
            self._last_imu = msg
            return

        dt = now - self._last_imu_time
        if dt <= 0.0 or dt > 1.0:
            self._last_imu_time = now
            return

        # Gyroskop verisinden dönme hızları (rad/s)
        omega_x = msg.angular_velocity.x
        omega_y = msg.angular_velocity.y
        omega_z = msg.angular_velocity.z

        # İvme ölçümü (m/s²)
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        if not self.use_gt:
            self._ekf_predict(ax, ay, az, omega_x, omega_y, omega_z, dt)

        self._last_imu = msg
        self._last_imu_time = now
        self._imu_initialized = True

    def _depth_callback(self, msg: Float64):
        """Basınç sensöründen derinlik güncellemesi — EKF update."""
        depth = msg.data  # metre (pozitif = aşağı)
        if not self.use_gt:
            self._ekf_update_depth(depth)
        # Derinliği her zaman doğrudan yayınla
        out = Float64()
        out.data = depth
        self.depth_pub.publish(out)

    def _ground_truth_callback(self, msg: Odometry):
        """Simülasyon: Gazebo ground truth + gerçekçi gürültü ekle."""
        self._gt_pose = msg
        
        if self.use_gt:
            # Ground truth'a gerçekçi sensor gürültüsü ekle
            pos_noise = np.random.normal(0, 0.05, 3)  # ±5cm
            yaw_noise = np.random.normal(0, 0.01)     # ±0.01 rad

            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            
            self.x[0] = p.x + pos_noise[0]
            self.x[1] = p.y + pos_noise[1]
            self.x[2] = p.z + pos_noise[2]

            # Quaternion → Euler
            roll, pitch, yaw = self._quat_to_euler(q.x, q.y, q.z, q.w)
            self.x[3] = roll
            self.x[4] = pitch
            self.x[5] = yaw + yaw_noise

            # Hız
            v = msg.twist.twist.linear
            vel_noise = np.random.normal(0, 0.02, 3)
            self.x[6] = v.x + vel_noise[0]
            self.x[7] = v.y + vel_noise[1]
            self.x[8] = v.z + vel_noise[2]

    # ================================================================
    # EKF - PREDICT (IMU ile hareket modeli)
    # ================================================================

    def _ekf_predict(self, ax, ay, az, wx, wy, wz, dt):
        """EKF tahmin adımı (hareket modeli)."""
        roll  = self.x[3]
        pitch = self.x[4]
        yaw   = self.x[5]
        vx    = self.x[6]
        vy    = self.x[7]
        vz    = self.x[8]
        bax   = self.x[9]
        bay   = self.x[10]
        baz   = self.x[11]

        # Bias düzeltmeli ivme (body frame)
        ax_corr = ax - bax
        ay_corr = ay - bay
        az_corr = az - baz

        # Body → World dönüşümü
        cr, sr = math.cos(roll),  math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw),   math.sin(yaw)

        # Dünya çerçevesinde ivme (yerçekimi çıkarılmış)
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr           ]
        ])
        a_world = R @ np.array([ax_corr, ay_corr, az_corr])
        a_world[2] += 9.81  # Yerçekimi kompansasyonu

        # Durum geçiş
        x_new = np.copy(self.x)
        x_new[0] += vx * dt + 0.5 * a_world[0] * dt**2
        x_new[1] += vy * dt + 0.5 * a_world[1] * dt**2
        x_new[2] += vz * dt + 0.5 * a_world[2] * dt**2
        x_new[6] += a_world[0] * dt
        x_new[7] += a_world[1] * dt
        x_new[8] += a_world[2] * dt

        # Açı entegrasyonu (Euler metodu)
        x_new[3] += wx * dt
        x_new[4] += wy * dt
        x_new[5] += wz * dt
        x_new[5] = self._wrap_angle(x_new[5])

        # Jacobian (F matrisi) - basitleştirilmiş
        F = np.eye(self.n_states)
        F[0, 6] = dt
        F[1, 7] = dt
        F[2, 8] = dt
        F[6, 9] = -dt
        F[7, 10] = -dt
        F[8, 11] = -dt

        # Kovaryans güncelleme
        self.P = F @ self.P @ F.T + self.Q * dt

        self.x = x_new

    # ================================================================
    # EKF - UPDATE (Derinlik ölçümü)
    # ================================================================

    def _ekf_update_depth(self, depth_measurement: float):
        """Derinlik sensöründen EKF güncelleme."""
        # Ölçüm matrisi: sadece Z pozisyonunu gözlemle
        H = np.zeros((1, self.n_states))
        H[0, 2] = 1.0

        # Yenilik (innovation)
        z = np.array([depth_measurement])
        z_pred = H @ self.x
        innovation = z - z_pred

        # Kalman kazancı
        S = H @ self.P @ H.T + self.R_depth
        K = self.P @ H.T @ np.linalg.inv(S)

        # Durum ve kovaryans güncelleme
        self.x = self.x + K @ innovation
        self.P = (np.eye(self.n_states) - K @ H) @ self.P

    # ================================================================
    # YAYINLAMA
    # ================================================================

    def _publish_estimate(self):
        """Tahmin edilen pozu yayınla."""
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'base_link'

        # Pozisyon
        odom.pose.pose.position.x = self.x[0]
        odom.pose.pose.position.y = self.x[1]
        odom.pose.pose.position.z = self.x[2]

        # Quaternion
        qx, qy, qz, qw = self._euler_to_quat(self.x[3], self.x[4], self.x[5])
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Hız
        odom.twist.twist.linear.x = self.x[6]
        odom.twist.twist.linear.y = self.x[7]
        odom.twist.twist.linear.z = self.x[8]

        # Kovaryans (6x6, sadece köşegen)
        cov = [0.0] * 36
        cov[0]  = self.P[0, 0]
        cov[7]  = self.P[1, 1]
        cov[14] = self.P[2, 2]
        cov[21] = self.P[3, 3]
        cov[28] = self.P[4, 4]
        cov[35] = self.P[5, 5]
        odom.pose.covariance = cov

        self.pose_pub.publish(odom)

        # TF yayını
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'world'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x[0]
            t.transform.translation.y = self.x[1]
            t.transform.translation.z = self.x[2]
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

    # ================================================================
    # YARDIMCI FONKSİYONLAR
    # ================================================================

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        while angle >  math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    @staticmethod
    def _quat_to_euler(qx, qy, qz, qw) -> Tuple[float, float, float]:
        """Quaternion → (roll, pitch, yaw) Euler açıları."""
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (qw * qy - qz * qx)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    @staticmethod
    def _euler_to_quat(roll, pitch, yaw) -> Tuple[float, float, float, float]:
        """Euler (roll, pitch, yaw) → Quaternion (qx, qy, qz, qw)."""
        cr, sr = math.cos(roll * 0.5),  math.sin(roll * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cy, sy = math.cos(yaw * 0.5),   math.sin(yaw * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)
    node = EKFDeadReckoning()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
