import time
import rclpy
import numpy as np
from transitions import Machine
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from scipy.interpolate import make_interp_spline

from .astar_path_finder import find_astar_path
from .global_map import GlobalMap
from .global_node import GlobalPlannerNode
from .global_mover import GlobalMover
from .utils import euler_from_quaternion


class GlobalPlanner(Node):
    states = ['IDLE', 'PLANNING', 'NAVIGATING']
    
    def __init__(self):
        super().__init__('global_planner')

        self.map = None
        self.start = None
        self.goal = None
        self.curr_path = None

        self.initialise_state()
        self.get_logger().info('init: States initialised')

        self.mover = GlobalMover(self)
        self.get_logger().info('init: GlobalMover initialised')

        self.map_subscriber = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.goal_subscriber = self.create_subscription(PoseStamped, 'goal', self.goal_callback, 10)

        self.path_publisher = self.create_publisher(Path, "path", 10)

    def initialise_state(self):
        self.machine = Machine(model=self, states=GlobalPlanner.states, initial='IDLE')
        self.machine.add_transition(trigger='start_planning', source='IDLE', dest='PLANNING')
        self.machine.add_transition(trigger='path_found', source='PLANNING', dest='NAVIGATING')
        self.machine.add_transition(trigger='goal_reached', source='NAVIGATING', dest='IDLE')
        self.machine.add_transition(trigger='fail', source='*', dest='IDLE')

    def map_callback(self, map_msg: OccupancyGrid):
        self.map = GlobalMap(map_msg)
        self.get_logger().info(f"map_callback: Map loaded")

    def odom_callback(self, odom_msg: Odometry):
        quaternion = (odom_msg.pose.pose.position.x,
                      odom_msg.pose.pose.position.y,
                      odom_msg.pose.pose.orientation.z,
                      odom_msg.pose.pose.orientation.w)
        _, _, yaw = euler_from_quaternion(quaternion)
        self.mover.robot_pos = GlobalPlannerNode(odom_msg.pose.pose.position.x,
                                           odom_msg.pose.pose.position.y,
                                           yaw)
        self.get_logger().info("odom_callback: Odometry (robot_pos) updated")
    
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
        self.get_logger().info("plan_path: Start path planning")

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


# def main(args=None):
#     rclpy.init(args=args)
#     node = GlobalPlannerMain()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
