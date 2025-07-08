import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)  # 10 Hz yayın
        self.get_logger().info("Velocity Publisher Node Started")

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 0.5   # İleri hareket (m/s)
        msg.angular.z = 0.3  # Hafif dönüş (rad/s)
        
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: linear.x={msg.linear.x}, angular.z={msg.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
