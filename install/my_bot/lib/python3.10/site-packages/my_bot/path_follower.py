import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
import math

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.sub = self.create_subscription(Path, '/planned_path', self.cb_path, 10)
        self.pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.timer = None
        self.waypoints = []
        self.current = 0

    def cb_path(self, msg):
        self.waypoints = msg.poses
        self.current = 0
        if self.timer is None:
            self.timer = self.create_timer(0.1, self.follow)

    def follow(self):
        if self.current >= len(self.waypoints):
            self.get_logger().info("Hedefe ulaşıldı.")
            self.timer.cancel()
            return
        goal = self.waypoints[self.current].pose.position
        # basit yönelim ve mesafe kontrolü
        dx = goal.x; dy = goal.y
        dist = math.hypot(dx,dy)
        yaw  = math.atan2(dy,dx)
        cmd = Twist()
        if dist > 0.1:
            cmd.linear.x  = 0.2
            cmd.angular.z = 4*(yaw)  # P kontrol
        self.pub.publish(cmd)
        if dist<0.1: self.current +=1

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()
