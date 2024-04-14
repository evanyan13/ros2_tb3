import time
import rclpy
import math
import cmath
import threading
import numpy as np
from math import atan2, pi
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import rclpy.logging as log
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan

from .utils import MOVE_TOL, LINEAR_VEL, ANGULAR_VEL, STOP_DISTANCE, FRONT_ANGLE

logger = log.get_logger("global_mover")

class GlobalMover(Node):
    def __init__(self, planner):
        super().__init__('global_mover')
        self.planner = planner
        self.robot_pos = None
        self.current_path = []
        self.current_goal_index = 0
        self.obstacle_detected = False
        self.laser_range = np.array([])

        self.path_subscriber = self.create_subscription(Path, 'path', self.path_callback, qos_profile_sensor_data)
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)

        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def path_callback(self, path_msg: Path):
        """
        Processing the incoming path and initiating navigation.
        """
        self.current_path = [(pose.pose.position.x, pose.pose.position.y) for pose in path_msg.poses]
        self.current_goal_index = 0

    def scan_callback(self, scan_msg: LaserScan):
        # create numpy array
        self.laser_range = np.array(scan_msg.ranges)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan
        if self.laser_range.size != 0:
            # check distances in front of TurtleBot and find values less
            # than stop_distance
            lri = (self.laser_range[FRONT_ANGLE]<float(STOP_DISTANCE)).nonzero()

            # if the list is not empty
            if(len(lri[0])>0):
                self.obstacle_detected = True
            else:
                self.obstacle_detected = False

    def follow_path(self):
        """
        Follows the given path by moving to each point sequentially.
        """
        if not self.current_path:
            return
        
        if self.obstacle_detected:
            logger.info(f"Obstacle detected, adjusting position")
            self.avoid_obstacle()
            return
        elif self.current_goal_index >= len(self.current_path):
            logger.info("End of path reached")
            self.stop_moving()
            self.reset_path()
            return
        else:
            current_goal = self.current_path[self.current_goal_index]
            self.move_to_point(current_goal)
        
    def move_to_point(self, goal):
        # Calculate angle to the goal
        dx = goal[0] - self.robot_pos.x
        dy = goal[1] - self.robot_pos.y
        goal_angle = atan2(dy, dx)
        heading_error = self.normalise_angle(goal_angle - self.robot_pos.theta)

        if abs(heading_error) > 0.1: # Threshold to decide if rotation is needed
            self.rotate_to_heading(heading_error, goal)
        else:
            distance_to_goal = math.hypot(dy, dy)
            if distance_to_goal >= MOVE_TOL:
                self.send_velocity(LINEAR_VEL, 0.0)
            else:
                self.current_goal_index += 1
                if self.current_goal_index < len(self.current_path):
                    next_goal = self.current_path(self.current_goal_index)
                    self.move_to_point(next_goal)
                else:
                    self.stop_moving()

        # # Control the robot to move to the next point in the path
        # distance_to_goal = math.hypot(goal[0] - self.robot_pos.x, goal[1] - self.robot_pos.y)
        # target_heading = math.atan2(goal[1] - self.robot_pos.y, goal[0] - self.robot_pos.x)
        # heading_error = self.normalise_angle(target_heading - self.robot_pos.theta)

        # if distance_to_goal < MOVE_TOL:
        #     self.current_goal_index += 1
        #     if self.current_goal_index >= len(self.current_path):
        #         self.stop_moving()  # Stop if path is complete
        #         return

        # linear = LINEAR_VEL
        # angular = ANGULAR_VEL * heading_error

        # # Moderate linear velocity based on how aligned we are to the goal direction
        # # This ensures the robot slows down if it needs to turn sharply
        # linear = LINEAR_VEL * max(0, 1 - 2 * abs(heading_error) / pi)

        # # logger.info(f"Moving to point {goal}")
        # self.send_velocity(linear, angular)

    def rotate_to_heading(self, heading_error, goal):
        dx = goal[0] - self.robot_pos.x
        dy = goal[1] - self.robot_pos.y
        angular_speed = ANGULAR_VEL * np.sign(heading_error)
        self.send_velocity(0.0, angular_speed)
        
        while abs(heading_error) > 0.05: # Small threshold to stop rotation
            self.robot_pos.theta += angular_speed * 0.1
            heading_error = self.normalise_angle(atan2(dy, dx) - self.robot_pos.theta)
            time.sleep(0.1)
        self.stop_moving()

    def avoid_obstacle(self):
        if self.laser_range.size != 0:
            # use nanargmax as there are nan's in laser_range added to replace 0's
            lr2i = np.nanargmax(self.laser_range)
        else:
            lr2i = 0

        logger.info(f"Avoiding obstacle, turning to {float(lr2i)}")
        # rotate to that direction and move
        self.rotatebot(float(lr2i))
        self.send_velocity(LINEAR_VEL, 0.0)

    # function to rotate the TurtleBot
    def rotate_bot(self, rot_angle):
        # create Twist object
        twist = Twist()

        # get current yaw angle
        current_yaw = self.robot_pos.theta
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * ANGULAR_VEL
        # start rotation
        self.velocity_publisher.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.robot_pos.theta
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)

        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.velocity_publisher.publish(twist)
    
    def send_velocity(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        time.sleep(1)
        self.velocity_publisher.publish(twist)
    
    def reset_path(self):
        self.planner.plan_path()
        # rclpy.spin_once(self.planner, timeout_sec=3.0)

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

    
