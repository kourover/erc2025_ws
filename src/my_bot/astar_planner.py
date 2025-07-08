# Dosya: src/my_bot/astar_planner.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        self.map = None
        # Harita ve hedef abonelikleri
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        # Rota yayıncısı
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        # TF dinleyici (map->base_link)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info('A* Planner başlatıldı')

    def map_callback(self, msg: OccupancyGrid):
        self.map = msg

    def goal_callback(self, goal: PoseStamped):
        if not self.map:
            self.get_logger().warning('Harita gelmedi, bekleniyor...')
            return
        # Başlangıç pozunu TF ile al
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_link', Time())
            sx = trans.transform.translation.x
            sy = trans.transform.translation.y
        except Exception:
            self.get_logger().warning('TF yok, bekleniyor...')
            return
        gx = goal.pose.position.x
        gy = goal.pose.position.y
        self.get_logger().info(f'Planlama: start=({sx:.2f},{sy:.2f}), goal=({gx:.2f},{gy:.2f})')

        # A* algoritmasını çağırın (kendi implementasyonunuz)
        path = self.run_astar((sx, sy), (gx, gy), self.map)
        path.header = Header(frame_id='map')
        self.path_pub.publish(path)
        self.get_logger().info('Rota yayınlandı')

    def run_astar(self, start, goal, grid: OccupancyGrid) -> Path:
        # TODO: Gerçek A* uygulamanızı buraya ekleyin
        # Şu an basit doğrusal tahmin ile ara adımlar oluşturuyoruz
        path = Path()
        x0, y0 = start
        x1, y1 = goal
        steps = 20
        for i in range(steps + 1):
            pose = PoseStamped()
            t = i / steps
            pose.header.frame_id = 'map'
            pose.pose.position.x = x0 + (x1 - x0) * t
            pose.pose.position.y = y0 + (y1 - y0) * t
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
