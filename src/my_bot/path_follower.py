#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        # Parametreler
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('max_speed', 0.3)
        self.lookahead = self.get_parameter('lookahead_distance').value
        self.max_speed = self.get_parameter('max_speed').value

        # Abonelikler ve yayıncı
        self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)

        # TF dinleyici (odom->base_link)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path = None
        self.get_logger().info('Path Follower başlatıldı')
        self.timer = self.create_timer(0.1, self.control_loop)

    def path_callback(self, msg: Path):
        self.path = msg
        self.get_logger().info('Yeni yol alındı, takip başlıyor')

    def control_loop(self):
        if not self.path or len(self.path.poses) == 0:
            return
        # Mevcut robot pozu
        try:
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', Time())
            rx = trans.transform.translation.x
            ry = trans.transform.translation.y
            q = trans.transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                             1 - 2*(q.y*q.y + q.z*q.z))
        except Exception:
            return

        # Lookahead noktasını seç
        goal_pt = None
        for pose_stamped in self.path.poses:
            dx = pose_stamped.pose.position.x - rx
            dy = pose_stamped.pose.position.y - ry
            if math.hypot(dx, dy) >= self.lookahead:
                goal_pt = (pose_stamped.pose.position.x, pose_stamped.pose.position.y)
                break
        if not goal_pt:
            goal_pt = (self.path.poses[-1].pose.position.x,
                       self.path.poses[-1].pose.position.y)

        # Yönelim ve hız
        dx = goal_pt[0] - rx
        dy = goal_pt[1] - ry
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = self.normalize(angle_to_goal - yaw)

        twist = Twist()
        twist.linear.x = min(self.max_speed, math.hypot(dx, dy))
        twist.angular.z = 1.0 * angle_diff
        self.cmd_pub.publish(twist)

    @staticmethod
    def normalize(a):
        while a > math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
