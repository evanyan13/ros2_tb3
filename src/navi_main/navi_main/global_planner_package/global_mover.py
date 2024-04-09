import rclpy
import math
import threading
import numpy as np
from math import atan2, pi
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
import rclpy.logging as log
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan

from .global_node import GlobalPlannerNode

MOVE_TOL = 0.1
LINEAR_VEL = 0.10
ANGULAR_VEL = 0.05
OBSTACLE_THRESHOLD = 0.10 # etres
PATH_REFRESH = 5  # seconds
logger = log.get_logger("global_mover")

class GlobalMover(Node):
    def __init__(self, planner):
        super().__init__('global_mover')
        self.planner = planner
        self.robot_pos = None
        self.current_path = []
        self.current_goal_index = 0
        self.obstacle_detected = False

        self.last_update_time = self.get_clock().now()
        self.path_refresh_interval = Duration(seconds=5) 

        self.path_subscriber = self.create_subscription(Path, 'path', self.path_callback, qos_profile_sensor_data)
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)

        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer for following the path
        self.follow_path_timer = self.create_timer(0.1, self.follow_path)
        # Timer for periodic path refresh
        self.path_refresh_timer = self.create_timer(PATH_REFRESH, self.refresh_path)

    def path_callback(self, path_msg: Path):
        """
        Processing the incoming path and initiating navigation.
        """
        self.current_path = path_msg.poses
        self.current_goal_index = 0
        # logger.info(f"path_callback: Received path with {len(self.current_path)} points.")

    def scan_callback(self, scan_msg: LaserScan):
        """ 
        Processes laser scan data to detect obstacles
        """
        if min(scan_msg.ranges) < OBSTACLE_THRESHOLD:
            self.obstacle_detected = True
            self.refresh_path()
            # logger.info(f"scan_callback: Obstacle encountered: {min(scan_msg.ranges)} < {OBSTACLE_THRESHOLD}")
        else:
            self.obstacle_detected = False

    def follow_path(self):
        """
        Follows the given path by moving to each point sequentially.
        """
        if self.current_goal_index >= len(self.current_path):
            return
        
        current_goal = self.current_path[self.current_goal_index].pose.position
        distance_to_goal = self.calculate_distance(self.robot_pos, current_goal)
        logger.info(f"follow_path: Following path.{current_goal.x, current_goal.y}")

        twist = Twist()
        if self.obstacle_detected:
            twist.linear.x = 0.0
            twist.angular.z = ANGULAR_VEL
            logger.info(f"follow_path: Obstacle encountered.")
        elif distance_to_goal > MOVE_TOL:
            twist.linear.x = LINEAR_VEL
            twist.angular.z = 0.0
            logger.info(f"follow_path: Moving forward.")
        else:
            # Current goal reached
            self.current_goal_index += 1
        
        # Publish twist
        self.velocity_publisher.publish(twist)

    def refresh_path(self):
        # Instruct planner to generate new path
        self.planner.fail()
        self.planner.plan_path()
    
    def calculate_distance(self, pos1, pos2):
        return np.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2)
    
