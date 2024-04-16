from statistics import mean
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped

from .global_planner import GlobalPlanner
from .global_map import GlobalMap
from .utils import EXPLORER_STEPS, OBSTACLE

class FrontierExplorer(Node):
    def __init__(self, global_planner: GlobalPlanner):
        super().__init__('frontier_explorer')
        self.global_planner = global_planner
        
        self.goal_publisher = self.create_publisher(PoseStamped, 'goal', qos_profile_sensor_data)

        time_period = 3
        self.timer = self.create_timer(time_period, self.publish_goal)

    def find_frontiers(self, map: GlobalMap):
        """
        Find available frontiers in current data
        """
        frontiers = []
        step = EXPLORER_STEPS
        moves = [(step, 0), (0, -step), (-step, 0), (0, step),
                      (step, step), (step, -step), (-step, step), (-step, -step)]

        for y in range(1, map.height - 1):
            for x in range(1, map.width - 1):
                if map.data[y, x] == OBSTACLE:
                    neighbours = [(y + dy, x + dx) for dy, dx in moves]

                    for ny, nx in neighbours:
                        if map.is_indice_avail(nx, ny):
                            frontiers.append((nx, ny))
                            break
        return frontiers

    def publish_goal(self):
        # self.get_logger().info("Attempting to publish goal...")
        current_map = self.global_planner.map
        current_robot_pos = self.global_planner.mover.robot_pos

        if not current_map or not current_robot_pos:
            self.get_logger().warn("publish_goal: STOPPED Map / robot_pos not initialised")
            return
        
        frontiers = self.find_frontiers(current_map)
        if not frontiers:
            self.get_logger().warn("publish_goal: No frontiers found")
            return

        bot_x, bot_y = current_map.coordinates_to_indices(current_robot_pos.x, current_robot_pos.y)
        furtherest_frontier = max(frontiers, key=lambda point: (point[0] - bot_x)**2 + (point[1] - bot_y)**2)
        goal_pose = current_map.indices_to_coordinates(furtherest_frontier[0], furtherest_frontier[1])

        # closet_frontier = min(frontiers, key=lambda point: (point[0] - bot_x)**2 + (point[1] - bot_y)**2)
        # goal_pose = current_map.indices_to_coordinates(closet_frontier[0], closet_frontier[1])

        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal_pose[0]
        goal_msg.pose.position.y = goal_pose[1]
        goal_msg.pose.orientation.w = 1.0

        self.goal_publisher.publish(goal_msg)
        # self.get_logger().info(f"Published new goal: ({goal_msg.pose.position.x}, {goal_msg.pose.position.y})")