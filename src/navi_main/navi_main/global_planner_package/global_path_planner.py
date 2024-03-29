import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

from .global_map import GlobalMap
from .global_planner_node import GlobalPlannerNode
from .astar_path_finder import find_astar_path
from .global_mover import GlobalMover

class GlobalPathPlanner(Node):
    def __init__(self):
        super().__init__('global_path_planner')

        self.map = None
        self.start = None
        self.goal = None

        self.mover = GlobalMover(self)

        self.is_goal_cancelled = False
        self.is_goal_reached = False
        self.is_map_loaded = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_subscriber = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.goal_subscriber = self.create_subscription(PoseStamped, "/goal", self.goal_callback, 10)

        self.path_publisher = self.create_publisher(Path, "/path", 10)
        self.mover_publisher = self.create_publisher(Path, "/move", 1)

    def map_callback(self, msg: OccupancyGrid):
        self.map = GlobalMap(msg)
        self.is_map_loaded = True
    
    def goal_callback(self, msg: PoseStamped):
        return None
    
    def calculate_path(self):
        self.get_logger.info("Calculating path")
        path_list = find_astar_path(self.map, self.start, self.goal)

        if path_list:
            self.get_logger.info("Path Received")
            path_msg = self.display_path(path_list)
            self.mover_publisher.publish(path_msg)
        else:
            self.get_logger.info("No path found")
            self.display_path([])
    
    def display_path(self, path_list: list) -> Path:
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for node in path_list:
            current_time = self.get_clock().now().to_msg()
            path_msg.poses.append(node.to_pose_stamped(current_time))
        
        self.path_publisher.publish(path_msg)
        return path_msg