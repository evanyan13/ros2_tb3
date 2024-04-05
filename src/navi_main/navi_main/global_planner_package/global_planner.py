import rclpy
import numpy as np
import threading
import tf2_ros
import queue
import rclpy.logging as log
from transitions import Machine
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from scipy.interpolate import make_interp_spline
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from .astar_path_finder import find_astar_path
from .global_map import GlobalMap
from .global_node import GlobalPlannerNode
from .global_mover import GlobalMover
from .utils import euler_from_quaternion, plot_map_helper


class GlobalPlanner(Node):
    states = ['IDLE', 'PLANNING', 'NAVIGATING']
    
    def __init__(self):
        super().__init__('global_planner')

        self.map = None
        self.start = None
        self.goal = None
        self.curr_path = None
        self.plot_queue = queue.Queue()
        self.planner_ready = threading.Event()

        self.mover = GlobalMover(self)

        self.initialise_state()
        self.get_logger().info(f'init: States initialised {self.state}')

        self.map_subscriber = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.goal_subscriber = self.create_subscription(PoseStamped, 'goal', self.goal_callback, 10)

        self.path_publisher = self.create_publisher(Path, "path", 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(f'init: GlobalPlanner initialised {self.state}')

    def initialise_state(self):
        self.machine = Machine(model=self, states=GlobalPlanner.states, initial='IDLE')
        self.machine.add_transition(trigger='start_planning', source='IDLE', dest='PLANNING')
        self.machine.add_transition(trigger='path_found', source='PLANNING', dest='NAVIGATING')
        self.machine.add_transition(trigger='goal_reached', source='NAVIGATING', dest='IDLE')
        self.machine.add_transition(trigger='fail', source='*', dest='IDLE')

    def map_callback(self, map_msg: OccupancyGrid):
        self.map = GlobalMap(map_msg)
        self.update_ros_pos_from_tf()
        if self.check_ready():
            map_data = plot_map_helper(self.map, map_msg, self.mover.robot_pos, self.goal, self.curr_path)
            self.plot_queue.put(map_data)
    
    def update_ros_pos_from_tf(self):
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform('map', 'base_link', now)

            translation = transform.transform.translation
            rotation = transform.transform.rotation
            _, _, yaw = euler_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)

            # Update the robot's current position
            self.mover.robot_pos = GlobalPlannerNode(translation.x, translation.y, yaw)
            self.start = self.mover.robot_pos

            # Initialise current robot_pos to 0
            start_x, start_y = self.map.coordinates_to_indices(self.start.x, self.start.y)
            self.map.data[start_y][start_x] = 0

        except(LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f'Could not transform base_link to map: {e}')
    
    def check_ready(self):
        if self.map and self.mover and self.mover.robot_pos:
            self.planner_ready.set() # Signal that GlobalPlanner is ready
            return True
    
    def goal_callback(self, pose_msg: PoseStamped):
        self.goal = GlobalPlannerNode.from_pose(pose_msg.pose)
        self.get_logger().info(f'goal_callback: Received new goal: ({self.goal.x}, {self.goal.y})')
        if self.map and self.mover.robot_pos:
            self.plan_path()
        else:
            self.wait_for_map()
    
    def plan_path(self):
        """"
        Given map information, plan and publish the path from start node to end node
        """
        if self.state != 'IDLE':
            self.get_logger().warn("plan_path: Already planning or navigating")
            return
        
        if not self.map or not self.start or not self.goal:
            self.fail()
            self.get_logger().warn("plan_path: Map and nodes are not initialised properly")
            return
        
        self.start_planning()
        start_pt = (self.start.x, self.start.y)
        goal_pt = (self.goal.x, self.goal.y)
        self.get_logger().info(f"plan_path: Start path planning {start_pt, goal_pt}")

        path_list = find_astar_path(self.map, self.start, self.goal)
        if path_list:
            self.curr_path = path_list
            self.get_logger().info(f"plan_path: Path found from astar, {len(path_list)} points")
            path_list = self.smooth_path_bspline(path_list)
            
            # Publish path information for mover
            path_msg = self.convert_to_path(path_list)
            self.path_publisher.publish(path_msg)
            self.get_logger().info("plan_path: Path published")
            self.path_found()
        else:
            self.get_logger().warn("plan_path: No path found")
            self.fail()

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
        t_fine = np.linspace(0, 1, num=max(30, len(path_list) * 3))
        smooth_data = spl(t_fine)
        x_fine, y_fine = smooth_data[:, 0], smooth_data[:, 1]

        smoothed_path_list = [GlobalPlannerNode(x, y) for x, y in zip(x_fine, y_fine)]

        return smoothed_path_list
    
    def convert_to_path(self, path_list: list) -> Path:
        """
        Take in a list of path points and convert it to path with PoseStamped
        """
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for node in path_list:
            current_time = self.get_clock().now().to_msg()
            path_msg.poses.append(node.to_pose_stamped(current_time))
        
        return path_msg
    
    def wait_for_map(self):
        while not self.map and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
