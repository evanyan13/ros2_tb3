import rclpy
import math
import cmath
import time
import threading
import numpy as np
from math import atan2, pi, hypot
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import rclpy.logging as log
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan

from .utils import MOVE_TOL, LINEAR_VEL, ANGULAR_VEL, STOP_DISTANCE, FRONT_ANGLE

logger = log.get_logger("global_mover")

class GlobalMover(Node):
    def __init__(self, robot_pos, planner):
        super().__init__('global_mover')
        self.is_active = False
        self.planner = planner
        self.robot_pos = robot_pos
        self.current_path = []
        self.current_goal_index = 0
        self.laser_range = np.array([])
        self.obstacle_detected = False
        self.following_path = True

        self.path_subscriber = self.create_subscription(Path, 'path', self.path_callback, qos_profile_sensor_data)
        self.global_scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)

        self.global_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.follow_path_timer = None
        # self.follow_path_timer = self.create_timer(1, self.follow_path)

    def path_callback(self, path_msg: Path):
        """
        Processing the incoming path and initiating navigation.
        """
        new_path = [(pose.pose.position.x, pose.pose.position.y) for pose in path_msg.poses]
        logger.info(f"path_callback: Received new path: {len(new_path)} vs {len(self.current_path)}.")
        self.current_path = new_path
        self.current_goal_index = 0
        # self.follow_path()

    def scan_callback(self, scan_msg: LaserScan):
        """ 
        Processes laser scan data to detect obstacles
        """
        self.laser_range = np.array(scan_msg.ranges)
        self.laser_range[self.laser_range == 0] = np.nan
        
        num_ranges = len(self.laser_range)
        degrees_per_index = 270 / num_ranges
        indices_per_side = int((FRONT_ANGLE / 2) / degrees_per_index)

        # Get indices for the front sector
        rear_left_sector = self.laser_range[:indices_per_side]
        rear_right_sector = self.laser_range[-indices_per_side:]

        min_distance_left = np.nanmin(rear_left_sector)
        min_distance_right = np.nanmin(rear_right_sector)

        # Get the minimum distance in the front sector
        min_distance = min(min_distance_left, min_distance_right)

        # Log information about obstacles in the front sector
        if min_distance < STOP_DISTANCE:
            self.obstacle_detected = True
            self.adjust_obstacle()
        else:
            self.obstacle_detected = False
        # logger.info(f"scan_callback: CALLBACK {self.obstacle_detected}")

    def follow_path(self):
        """
        Follows the given path by moving to each point sequentially.
        """
        if self.follow_path:
            if not self.current_path:
                return
            
            if self.obstacle_detected:
                logger.info(f"follow_path: Obstacle detected, adjusting position")
                self.follow_path = False
                self.adjust_obstacle()
                return

            if self.current_goal_index >= len(self.current_path):
                logger.info("follow_path: End of path reached")
                self.follow_path = False
                self.stop_moving()
                self.reset_path()
            else:
                current_goal = self.current_path[self.current_goal_index]
                self.move_to_point(current_goal)
                # logger.info("follow_path: Following path")

        self.follow_path_timer = threading.Timer(1.0, self.follow_path).start()
        
    def move_to_point(self, goal):
        dy = goal[1] - self.robot_pos.y
        dx = goal[0] - self.robot_pos.x
        distance_to_goal = hypot(dx, dy)
        target_heading = atan2(dy, dx)
        heading_error = self.normalise_angle(target_heading - self.robot_pos.theta)

        if distance_to_goal < MOVE_TOL:
            self.current_goal_index += 1
            return

        linear = LINEAR_VEL
        angular = ANGULAR_VEL * heading_error
        # Moderate linear velocity based on how aligned we are to the goal direction
        # This ensures the robot slows down if it needs to turn sharply
        linear = LINEAR_VEL * max(0, 1 - 2 * abs(heading_error) / pi)

        logger.info(f"Moving to point ({goal[0]:.2f}, {goal[1]:.2f})")
        self.send_velocity(linear, angular)

    def adjust_obstacle(self):
        self.get_logger().info('Adjusting to obstacle')
        if self.laser_range.size != 0:
            # use nanargmax as there are nan's in laser_range added to replace 0's
            lr2i = np.nanargmax(self.laser_range)
            self.get_logger().info('Picked direction: %d %f m' % (lr2i, self.laser_range[lr2i]))
        else:
            lr2i = 0
            self.get_logger().info('No data!')

        # rotate to that direction
        self.rotatebot(float(lr2i))
        logger.info("Complete rotation")
        self.send_velocity(LINEAR_VEL, 0.0)
        self.check_obstacle_clear()

    def rotatebot(self, rot_angle):
        self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.robot_pos.theta
        # log the info
        logger.info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        logger.info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * ANGULAR_VEL
        # start rotation
        self.global_vel_publisher.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            # rclpy.spin_once(self)
            current_yaw = self.robot_pos.theta
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        logger.info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.global_vel_publisher.publish(twist)
        self.stop_moving()
    
    def check_obstacle_clear(self):
        # Check if the obstacle is still detected
        if not self.obstacle_detected:
            logger.info("Obstacle cleared, resuming path")
            self.reset_path()
            self.follow_path = True
            threading.Timer(1.0, self.follow_path).start()
        else:
            logger.info("Obstacle still detected, checking further")
            threading.Timer(1.0, self.adjust_obstacle).start()
    
    def send_velocity(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        time.sleep(1)
        self.global_vel_publisher.publish(twist)
    
    def reset_path(self):
        self.planner.plan_path()
        # rclpy.spin_once(self)

    def stop_moving(self):
        self.send_velocity(0.0, 0.0)
        self.planner.fail()
    
    def normalise_angle(self, angle):
        # Normalize the angle to be between -pi and pi
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def on_shutdown(self):
        self.stop_moving()
        self.is_active = False
        if self.follow_path_timer:
            self.follow_path_timer.cancel()    
    
