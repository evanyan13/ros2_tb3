import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from scipy.interpolate import BSpline, make_interp_spline

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
        if self.mover.is_moving:
            self.cancel_goal()

        self.get_logger().info("Goal Callback: Received new goal")
        self.goal = GlobalPlannerNode.from_pose(msg.pose)
        self.is_goal_cancelled = False
        self.is_goal_reached = False

        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.start = GlobalPlannerNode.from_tf(trans.transform.translation, trans.transform.rotation)
            self.mover.robot_pos = self.start
        except TransformException as e:
            self.get_logger().info(f"Could not transform base_link to map: {str(e)}")
            return
        
        if not self.map.is_node_free(self.goal) or not self.map or not self.start:
            self.get_logger().info("Goal cannot be reached")
            self.display_path([]) # Empty path
            return
        
        self.calculate_path()
    
    def calculate_path(self):
        self.get_logger().info("Calculating path")
        path_list = find_astar_path(self.map, self.start, self.goal)

        if path_list:
            self.get_logger().info("Path Received")
            path_list = self.smooth_path_bspline(path_list)
            path_msg = self.display_path(path_list)
            self.mover_publisher.publish(path_msg)
        else:
            self.get_logger().info("No path found")
            self.display_path([])

    def smooth_path_bspline(self, path_list: list) -> list:
        """
        Use B-spine algorithm to generate a smoother path between the given path
        """
        if len(path_list) < 3:
            self.get_logger().warn("Not enough ponts to complete B-spline")
            return path_list
        
        x = [node.x for node in path_list]
        y = [node.y for node in path_list]

        # Parameterisation variable
        t = np.linspace(0, 1, len(path_list))
        spline_degree = min(3, len(path_list) - 1)
        data = np.array([x, y]).T

        spl = make_interp_spline(t, data, k=spline_degree)

        # Expand the number of points into more fine ones
        t_fine = np.linspace(0, 1, num=len(path_list) * 10)
        smooth_data = spl(t_fine)
        x_fine, y_fine = smooth_data[:, 0], smooth_data[:, 1]

        smoothed_path_list = [GlobalPlannerNode(x, y) for x, y in zip(x_fine, y_fine)]

        return smoothed_path_list
    
    def display_path(self, path_list: list) -> Path:
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for node in path_list:
            current_time = self.get_clock().now().to_msg()
            path_msg.poses.append(node.to_pose_stamped(current_time))
        
        self.path_publisher.publish(path_msg)
        return path_msg
    
    def wait_for_map(self):
        while not self.is_map_loaded and not rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
    
    def wait_for_result(self, duration: float) -> bool:
        sending_time = self.get_clock().now().seconds_nanoseconds()[0]
        rclpy.spin_once(self, timeout_sec=1.0)

        while self.mover.is_moving and not self.is_goal_cancelled and rclpy.ok():
            if self.get_clock().now().seconds_nanoseconds()[0] - sending_time > duration:
                self.cancel_goal()
                return False

        return self.is_goal_reached
    
    def send_goal(self, pose: Pose):
        self.is_goal_cancelled = False
        self.goal = GlobalPlannerNode.from_pose(pose)
        return self.calculate_path()
    
    def cancel_goal(self):
        self.mover.stop_moving()
        self.is_goal_cancelled = True