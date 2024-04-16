from statistics import mean
import rclpy
import numpy as np
import threading
import tf2_ros
import rclpy.logging as log
from transitions import Machine
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String
from scipy.interpolate import make_interp_spline
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from .astar_path_finder import find_astar_path
from .global_map import GlobalMap
from .global_node import GlobalPlannerNode
from .mover import Mover
from .utils import euler_from_quaternion, PATH_REFRESH, MOVER_PATH_REFRESH

logger = log.get_logger("global_planner")

class GlobalPlanner(Node):
    states = ['IDLE', 'PLANNING', 'NAVIGATING_LOCAL', 'NAVIGATING_GLOBAL']
    
    def __init__(self):
        super().__init__('global_planner')

        self.map = None
        self.start = None
        self.goal = None
        self.planner_ready = threading.Event()
        self.first_path = True
        self.last_path_time = self.get_clock().now().nanoseconds * 1e-9
        self.shutdown_requested = False

        self.mover = Mover(self)

        self.initialise_states()
        logger.info(f'init: States initialised {self.state}')

        self.map_subscriber = self.create_subscription(OccupancyGrid, 'map', self.map_callback, qos_profile_sensor_data)
        self.goal_subscriber = self.create_subscription(PoseStamped, 'goal', self.goal_callback, qos_profile_sensor_data)
        self.stop_subscriber = self.create_subscription(String, 'enter_state', self.handle_stop_command, qos_profile_sensor_data)

        self.path_publisher = self.create_publisher(Path, "path", 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.path_refresh_timer = self.create_timer(PATH_REFRESH, self.plan_path)

        logger.info(f'init: GlobalPlanner initialised {self.state}')

    def initialise_states(self):
        self.machine = Machine(model=self, states=GlobalPlanner.states, initial='IDLE')
        self.machine.add_transition(trigger='start_planning', source='IDLE', dest='PLANNING')
        self.machine.add_transition(trigger='switch_to_local', source='*', dest='NAVIGATING_LOCAL')
        self.machine.add_transition(trigger='switch_to_global', source='*', dest='NAVIGATING_GLOBAL')
        self.machine.add_transition(trigger='goal_reached', source='*', dest='IDLE')
        self.machine.add_transition(trigger='fail', source='*', dest='IDLE')

    def map_callback(self, map_msg: OccupancyGrid):
        self.map = GlobalMap(map_msg)
        if self.map and self.map.data is not None:
            self.update_ros_pos_from_tf()
            self.check_ready()
        else:
            logger.warn("Map data is not fully initialized yet.")
    
    def update_ros_pos_from_tf(self):
        """
        Transformation function to update bot current position using base_link
        """
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
            logger.error(f'Could not transform base_link to map: {e}')

    def goal_callback(self, pose_msg: PoseStamped):
        self.goal = GlobalPlannerNode.from_pose(pose_msg.pose)
        # logger.info(f'goal_callback: Received new goal: ({self.goal.x}, {self.goal.y})')

    def handle_stop_command(self, msg: String):
        if msg.data == 'Stop Nav':
            logger.info("Stopping navigation as instructed.")
            self.shutdown_requested = True

    def check_ready(self):
        if self.map and self.mover and self.mover.robot_pos:
            self.planner_ready.set() # Signal that GlobalPlanner is ready
            return True
    
    def check_shutdown(self):
        return self.shutdown_requested
    
    def plan_path(self, reset_path=False):
        """"
        Given map information, plan and publish the path from start node to end node
        """
        if not self.map or not self.start or not self.goal:
            self.fail()
            logger.warn("plan_path: Map and nodes are not initialised properly")
            return
        
        current_time = self.get_clock().now().nanoseconds * 1e-9
        current_path_time = current_time - self.last_path_time

        if current_path_time > MOVER_PATH_REFRESH or self.first_path or reset_path:
            self.fail()
            self.start_planning()
            
            if self.state != 'PLANNING':
                logger.warn("plan_path: State is not PLANNING after start_planning call")
                return

            path_list = find_astar_path(self.map, self.start, self.goal)
            if path_list:
                # Publish path information for mover
                waypoints = self.get_waypoints(path_list)
                path_msg = self.convert_to_path(waypoints)
                self.path_publisher.publish(path_msg)
                logger.info(f"plan_path: Published new path")
                self.last_path_time = current_time
                # self.get_logger().info("plan_path: Path published")
                if self.state == "PLANNING":
                    self.switch_to_global()
                    self.first_path = False
            else:
                logger.warn("plan_path: No path found")
                self.switch_to_local()
    
    def get_waypoints(self, path_list):
        waypoints = [path_list[0]] # Add the first node
        prev_dirt = (0, 0)

        for i in range(1, len(path_list)):
            curr_dirt = (path_list[i].x - path_list[i - 1].x,
                         path_list[i].y - path_list[i - 1].y)
        
            if curr_dirt != prev_dirt:
                waypoints.append(path_list[i - 1])
                prev_dirt = curr_dirt
        
        waypoints.append(path_list[-1]) # Add the last node
        return waypoints


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
            rclpy.spin_once(self, timeout_sec=0.5)