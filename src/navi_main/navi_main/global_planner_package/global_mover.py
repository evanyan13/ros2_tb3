import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from math import atan2, pi

from .global_planner_node import GlobalPlannerNode

move_tolerance = 0.1
scan_tolerance_front = 0.4
scan_tolerance_side = 0.3
rotate_tolerance = 0.004
linear_velocity = 0.25
angular_velocity = 1.5

class GlobalMover(Node):
    def __init__(self, planner):
        super().__init__('global_mover')

        self.robot_pos = None
        self.path_deviation = 0.0
        self.planner = planner

        self.is_shutdown_initiated = False
        self.is_moving = False
        self.is_obstacle_ahead = False

        self.path_subscriber = self.create_subscription(Path, 'path', self.path_callback, 10)
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def path_callback(self, path: Path):
        node_path = []

        for pose in path.poses:
            node_path.append(GlobalPlannerNode.from_pose(pose.pose))

        self.follow_path(node_path)

    # Localised obstacle prevention
    def scan_callback(self, scan_data: LaserScan):
        if np.nanmin(scan_data.ranges[0:10] + scan_data.ranges[350:360]) < scan_tolerance_front:
            self.is_obstacle_ahead = True
        elif np.nanmin(scan_data.ranges[11:165]) < scan_tolerance_side:
            self.path_deviation = -0.5
        elif np.nanmin(scan_data.ranges[195:349]) < scan_tolerance_side:
            self.path_deviation = 0.5
        else:
            self.path_deviation = 0.0
    
    def stop_moving(self):
        self.is_moving = False
        self.velocity_publisher.publish(Twist()) # A zero twist to stop the bot

    def follow_path(self, path: list):
        self.is_moving = True

        for node in path:
            if self.is_shutdown_initiated:
                self.stop_moving()
                return
            if self.is_obstacle_ahead:
                self.stop_moving()
                self.move_back()
                self.planner.calcualte_path()
                return

            self.move_to_point(node)

        # Reached goal
        self.stop_moving()
        self.rotate_to_goal(self.planner.goal)
        self.stop_moving()

        self.planner.is_goal_reached = True
        self.is_moving = False

    def move_back(self):
        distance_moved = 0.0
        twist_msg = Twist()
        twist_msg.linear.x = -linear_velocity
        t0 = self.get_clock().now()

        while distance_moved < 0.4:
            if self.is_shutdown_initiated:
                self.is_obstacle_ahead = False
                return
            
            self.velocity_publisher.publish(twist_msg)
            t1 = self.get_clock().now()
            distance_moved = linear_velocity * (t1 - t0)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        self.is_obstacle_ahead = False

    def move_to_point(self, point: GlobalPlannerNode):
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity # Moves the bot forward
        loop_rate = self.create_rate(1000) # Create a rate to sleep at 1000 Hz

        print("START MOVING")
        while self.robot_pos.calculate_distance(point) > move_tolerance:
            if self.is_shutdown_initiated or self.is_obstacle_ahead:
                print("MOVE STOPPED - Shutdown Initiated / Obstacle")
                return
            
            speed = angular_velocity * self.angular_difference(point)
            twist_msg.angular.z = min(angular_velocity, speed) + self.path_deviation
            self.velocity_publisher.publish(twist_msg)
            print(f"Published twist_msg: {twist_msg}")
            loop_rate.sleep()
        print("POINT REACHED")

    def rotate_to_goal(self, goal: GlobalPlannerNode):
        twist_msg = Twist()
        loop_rate = self.create_rate(1000)

        print("START ROTATING")
        while abs(self.angular_difference(goal)) > rotate_tolerance:
            if self.is_shutdown_initiated:
                self.stop_moving()
                print("ROTATE STOPPED - Shutdown Initiated")
                return
            
            speed = angular_velocity * self.angular_difference(goal)
            twist_msg.angular.z = min(angular_velocity, max(-angular_velocity, speed))
            self.velocity_publisher.publish(twist_msg)
            print(f"Published twist_msg: {twist_msg}")
            loop_rate.sleep()

        self.velocity_publisher.publish(Twist())  # Stop the robot
        print("ROTATE COMPLETE")

    def angular_difference(self, point: GlobalPlannerNode) -> float:
        """
        Calcualtes the angle btw the current orientation of the bot and the direction to the target point
        """
        angle = atan2(point.y - self.robot_pos.y, point.x - self.robot_pos.x)\
                    - self.robot_pos.theta
        
        # Normalising the angle to be within [-pi, pi]
        if angle <= -pi:
            angle += 2 * pi
        elif angle > pi:
            angle -= 2 * pi
        
        return angle
    