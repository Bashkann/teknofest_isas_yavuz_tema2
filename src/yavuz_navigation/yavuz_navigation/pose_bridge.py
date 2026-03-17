#!/usr/bin/env python3
"""
YAVUZ AUV - Gazebo Pose Bridge
Gazebo'dan gelen model pose'unu lokalizasyon topic'ine çevirir.
use_sim_time=false ile çalışır (VM'de zaman senkron sorunu olmaması için).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math

from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class GazeboPoseBridge(Node):

    def __init__(self):
        super().__init__('gazebo_pose_bridge')

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)

        self.sub = self.create_subscription(
            TFMessage,
            '/world/teknofest_theme2/dynamic_pose/info',
            self._cb, qos)

        self.odom_pub = self.create_publisher(Odometry, '/yavuz/localization/odom', rel)
        self.depth_pub = self.create_publisher(Float64, '/yavuz/localization/depth', rel)

        self._model_names = ['yavuz_auv', 'base_link', 'yavuz']
        self._pub_count = 0

        self.get_logger().info("Gazebo Pose Bridge baslatildi!")
        self.get_logger().info("  /world/teknofest_theme2/dynamic_pose/info -> /yavuz/localization/odom")

    def _cb(self, msg: TFMessage):
        for tf in msg.transforms:
            cid = tf.child_frame_id.lower()
            fid = tf.header.frame_id.lower()

            # yavuz_auv modelinin world'e göre pozisyonu
            is_model = any(n in cid for n in self._model_names) or \
                       any(n in fid for n in self._model_names)

            if not is_model:
                continue

            t = tf.transform.translation
            r = tf.transform.rotation

            # Sadece ana gövdeyi al (thruster değil)
            if abs(t.x) > 200 or abs(t.y) > 200:
                continue

            # Quaternion'dan yaw hesapla (debug için)
            yaw = math.atan2(2*(r.w*r.z + r.x*r.y),
                             1 - 2*(r.y*r.y + r.z*r.z))

            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'world'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = t.x
            odom.pose.pose.position.y = t.y
            odom.pose.pose.position.z = t.z
            odom.pose.pose.orientation.x = r.x
            odom.pose.pose.orientation.y = r.y
            odom.pose.pose.orientation.z = r.z
            odom.pose.pose.orientation.w = r.w

            self.odom_pub.publish(odom)

            depth_msg = Float64()
            depth_msg.data = t.z
            self.depth_pub.publish(depth_msg)

            self._pub_count += 1
            if self._pub_count % 100 == 0:
                self.get_logger().info(
                    f"Pos: ({t.x:.2f}, {t.y:.2f}, {t.z:.2f}) yaw={math.degrees(yaw):.1f}deg"
                )
            return  # İlk eşleşen frame yeterli


def main(args=None):
    rclpy.init(args=args)
    node = GazeboPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
