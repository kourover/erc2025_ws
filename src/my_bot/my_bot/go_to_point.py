#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

class GoToPoint(Node):
    def __init__(self):
        super().__init__('go_to_point')

        # --- 1) Parametreleri tanımla ve al ---
        self.declare_parameter('target_frame',   'odom')    # önce denenecek TF çerçevesi
        self.declare_parameter('fallback_frame', 'world')   # odom yoksa buraya bak
        self.declare_parameter('target_x',       1.0)
        self.declare_parameter('target_y',       0.0)
        self.declare_parameter('tolerance',      0.05)      # m
        self.declare_parameter('max_linear',     0.5)       # m/s
        self.declare_parameter('max_angular',    1.0)       # rad/s
        self.declare_parameter('ang_threshold',  0.1)       # rad

        self.target_frame  = self.get_parameter('target_frame').value
        self.fallback_frame= self.get_parameter('fallback_frame').value
        self.target_x      = self.get_parameter('target_x').value
        self.target_y      = self.get_parameter('target_y').value
        self.tolerance     = self.get_parameter('tolerance').value
        self.max_linear    = self.get_parameter('max_linear').value
        self.max_angular   = self.get_parameter('max_angular').value
        self.ang_threshold = self.get_parameter('ang_threshold').value

        # --- 2) Publisher ve TF listener ---
        self.cmd_pub    = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.tf_buffer  = Buffer()
        self.tf_listener= TransformListener(self.tf_buffer, self)

        # --- 3) Kontrol döngüsü (10 Hz) ---
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f"[GoToPoint] Başladı → hedef=({self.target_x:.2f}, {self.target_y:.2f}), "
            f"{self.target_frame}|fallback={self.fallback_frame}, tol={self.tolerance:.2f}"
        )

    def control_loop(self):
        # --- a) TF’den güncel poz al (önce target_frame, yoksa fallback) ---
        trans = None
        for frame in (self.target_frame, self.fallback_frame):
            try:
                trans = self.tf_buffer.lookup_transform(
                    frame, 'base_link',
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1)
                )
                used_frame = frame
                break
            except Exception:
                continue

        if trans is None:
            self.get_logger().warning(
                f"TF bulunamadı: ne '{self.target_frame}' ne '{self.fallback_frame}'"
            )
            return

        # --- b) Poz ve yön açısını hesapla ---
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        q = trans.transform.rotation
        yaw = math.atan2(
            2.0*(q.w*q.z + q.x*q.y),
            1.0 - 2.0*(q.y*q.y + q.z*q.z)
        )

        # --- c) Hedefe mesafe ve açı farkı ---
        dx = self.target_x - x
        dy = self.target_y - y
        distance    = math.hypot(dx, dy)
        angle_goal  = math.atan2(dy, dx)
        angle_error = self.normalize_angle(angle_goal - yaw)

        # --- d) Twist komutunu hazırla ---
        cmd = Twist()
        if distance > self.tolerance:
            if abs(angle_error) > self.ang_threshold:
                # dön
                cmd.angular.z = max(
                    -self.max_angular,
                    min(self.max_angular, 2.0 * angle_error)
                )
                cmd.linear.x = 0.0
            else:
                # ileri
                cmd.linear.x  = max(
                    0.0,
                    min(self.max_linear, 0.5 * distance)
                )
                cmd.angular.z = 0.0
        else:
            # hedefte dur, ama timer çalışmaya devam edecek
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0

        # --- e) Yayınla ---
        self.cmd_pub.publish(cmd)

    @staticmethod
    def normalize_angle(angle):
        """Açıyı –π…+π aralığına getir."""
        while angle > math.pi:
            angle -= 2.0*math.pi
        while angle < -math.pi:
            angle += 2.0*math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = GoToPoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
