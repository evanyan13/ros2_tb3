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
        self.current_path = [(pose.pose.position.x, pose.pose.position.y) for pose in path_msg.poses]
        self.current_goal_index = 0
        # logger.info(f"path_callback: Received path with {len(self.current_path)} points.")

    def scan_callback(self, scan_msg: LaserScan):
        """ 
        Processes laser scan data to detect obstacles
        """
        if np.nanmin(scan_msg.ranges) < OBSTACLE_THRESHOLD:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def follow_path(self):
        """
        Follows the given path by moving to each point sequentially.
        """
        if self.obstacle_detected or self.current_goal_index >= len(self.current_path):
            self.handle_obstacle()
            return

        current_goal = self.current_path[self.current_goal_index]
        self.move_to_point(current_goal)

    def handle_obstacle(self):
        if self.obstacle_detected:
            self.send_velocity(0.0, 0.0)
            self.refresh_path()
            
    def move_to_point(self, goal):
        # Control the robot to move to the next point in the path
        distance_to_goal = math.hypot(self.robot_pos.x - goal[0], self.robot_pos.y - goal[1])
        if distance_to_goal < MOVE_TOL:
            self.current_goal_index += 1
            if self.current_goal_index >= len(self.current_path):
                self.send_velocity(0, 0)  # Stop if path is complete
                return

        target_heading = math.atan2(goal[1] - self.robot_pos.y, goal[0] - self.robot_pos.x)
        heading_error = self.normalise_angle(target_heading - self.robot_pos.theta)

        linear = LINEAR_VEL
        angular = ANGULAR_VEL * heading_error

        self.send_velocity(linear, angular)

    def refresh_path(self):
        # Instruct planner to generate new path
        self.planner.fail()
        self.planner.plan_path()
    
    def send_velocity(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.velocity_publisher.publish(twist)
    
    def normalise_angle(self, angle):
        # Normalize the angle to be between -pi and pi
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
