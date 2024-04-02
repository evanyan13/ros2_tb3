import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from math import atan2, pi

from .global_node import GlobalPlannerNode

move_tolerance = 0.1
scan_tolerance_front = 0.4
scan_tolerance_side = 0.3
rotate_tolerance = 0.004
linear_velocity = 0.25
angular_velocity = 1.5

class GlobalMover(Node):
    def __init__(self, planner):
        super().__init__('global_mover')

        self.robot_pos = GlobalPlannerNode()
        self.path_deviation = 0.0
        self.planner = planner

        self.is_moving = False
        self.is_obstacle_ahead = False

        self.path_subscriber = self.create_subscription(Path, 'path', self.path_callback, 10)
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def path_callback(self, path_msg: Path):
        """
        Processing the incoming path and initiating navigation.
        """
        node_path = [GlobalPlannerNode.from_pose(pose.pose) for pose in path_msg.poses]
        self.get_logger().info(f"path_callback: Received path with {len(node_path)} points.")

        if node_path:
            self.follow_path(node_path)
        else:
            self.handle_navi_fail()

    def scan_callback(self, scan_data: LaserScan):
        """ 
        Processes laser scan data to detect obstacles and adjust path deviation.
        """
        if np.nanmin(scan_data.ranges[0:10] + scan_data.ranges[350:360]) < scan_tolerance_front:
            self.is_obstacle_ahead = True
        elif np.nanmin(scan_data.ranges[11:165]) < scan_tolerance_side:
            self.path_deviation = -0.5
        elif np.nanmin(scan_data.ranges[195:349]) < scan_tolerance_side:
            self.path_deviation = 0.5
        else:
            self.path_deviation = 0.0
    
    def stop_moving(self):
        """
        Stops the bot movement by publishing a zero velocity.
        """
        self.is_moving = False
        self.velocity_publisher.publish(Twist()) # A zero twist to stop the bot

    def handle_navi_fail(self):
        self.stop_moving()
        self.planner.fail()

    def complete_navi(self):
        self.stop_moving()
        self.planner.goal_reached()

    def follow_path(self, path: list):
        """
        Follows the given path by moving to each point sequentially.
        """
        self.is_moving = True
        self.get_logger().info("follow_path: Starting to follow the path.")

        for node in path:
            if not rclpy.ok():
                self.handle_navi_fail()
                return
            # If an obstacle is encountered,
            # Move the bot back and recompute path
            if self.is_obstacle_ahead:
                self.handle_navi_fail()
                self.move_back()
                self.planner.plan_path()
                return

            self.move_to_point(node)

        # Reached goal
        self.stop_moving()
        self.rotate_to_goal(self.planner.goal)
        self.complete_navi()

    def move_back(self):
        """
        Moves the bot backward for a short distance when an obstacle is detected.
        """
        self.get_logger().info("move_back: Moving back due to an obstacle.")
        distance_moved = 0.0
        twist_msg = Twist()
        twist_msg.linear.x = -linear_velocity
        t0 = self.get_clock().now()

        while distance_moved < 0.4 and rclpy.ok():
            self.velocity_publisher.publish(twist_msg)
            t1 = self.get_clock().now()
            distance_moved = linear_velocity * (t1 - t0)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        self.is_obstacle_ahead = False

    def move_to_point(self, point: GlobalPlannerNode):
        """
        Moves the bot to the specified point
        """
        self.get_logger().info(f"move_to_point: Moving to point ({point.x}, {point.y})")        
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity # Moves the bot forward
        loop_rate = self.create_rate(1000) # Create a rate to sleep at 1000 Hz

        while self.robot_pos.calculate_distance(point) > move_tolerance:
            if self.is_obstacle_ahead:
                self.get_logger().warn("move_to_point: Move stopped due to obstacle ahead")
                return
            
            speed = angular_velocity * self.angular_difference(point)
            twist_msg.angular.z = min(angular_velocity, speed) + self.path_deviation
            self.velocity_publisher.publish(twist_msg)
            loop_rate.sleep()

        self.get_logger().info("move_to_point: Point reached")

    def rotate_to_goal(self, goal: GlobalPlannerNode):
        """
        Rotates the bot to align with the goal orientation.
        """
        self.get_logger().info(f"rotate_to_goal: Rotate to align with goal ({goal.x}, {goal.y})") 
        twist_msg = Twist()
        loop_rate = self.create_rate(1000)

        while abs(self.angular_difference(goal)) > rotate_tolerance:
            if not rclpy.ok():
                self.stop_moving()
                return
            
            speed = angular_velocity * self.angular_difference(goal)
            twist_msg.angular.z = min(angular_velocity, max(-angular_velocity, speed))
            self.velocity_publisher.publish(twist_msg)
            loop_rate.sleep()

        self.velocity_publisher.publish(Twist())  # Stop the robot
        self.get_logger().info(f"rotate_to_goal: Rotate complete") 

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


def main(args=None):
    rclpy.init(args=args)
    node = GlobalMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    