import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import heapq

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        self.map = None
        self.sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.pub = self.create_publisher(Path, '/planned_path', 10)
        self.declare_parameter('start', [0,0])
        self.declare_parameter('goal', [5,5])

    def map_callback(self, msg):
        self.map = msg
        self.plan_path()

    def plan_path(self):
        if not self.map: return
        start = tuple(self.get_parameter('start').value)
        goal  = tuple(self.get_parameter('goal').value)
        grid = self.occupancy_to_grid(self.map)
        path  = self.a_star(grid, start, goal)
        ros_path = Path()
        ros_path.header = self.map.header
        for x,y in path:
            pose = PoseStamped()
            pose.header = ros_path.header
            pose.pose.position.x = x * self.map.info.resolution
            pose.pose.position.y = y * self.map.info.resolution
            ros_path.poses.append(pose)
        self.pub.publish(ros_path)

    def occupancy_to_grid(self, msg):
        w,h = msg.info.width, msg.info.height
        data = msg.data
        # 0: free, 1: occupied
        return [[1 if data[i+j*w] > 50 else 0 for i in range(w)] for j in range(h)]

    def a_star(self, grid, start, goal):
        h = lambda a,b: abs(a[0]-b[0]) + abs(a[1]-b[1])
        open_, came = [(h(start,goal), 0, start, None)], {}
        closed = set()
        while open_:
            f, g, cur, parent = heapq.heappop(open_)
            if cur in closed: continue
            came[cur] = parent
            if cur == goal: break
            closed.add(cur)
            for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                nb = (cur[0]+dx, cur[1]+dy)
                if 0<=nb[0]<len(grid[0]) and 0<=nb[1]<len(grid) and grid[nb[1]][nb[0]]==0:
                    heapq.heappush(open_, (g+1+h(nb,goal), g+1, nb, cur))
        # reconstruct
        node = goal; path=[]
        while node:
            path.append(node); node = came.get(node)
        return path[::-1]

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
