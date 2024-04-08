from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped

from .global_map import GlobalMap
from .global_node import GlobalPlannerNode

class FrontierExplorer(Node):
    def __init__(self, global_map: GlobalMap, robot_pos: GlobalPlannerNode):
        super().__init__('frontier_explorer')
        self.map = global_map
        self.robot_pos = robot_pos
        
        self.goal_publisher = self.create_publisher(PoseStamped, 'goal', qos_profile_sensor_data)

        time_period = 3
        self.timer = self.create_timer(time_period, self.publish_goal)

    def publish_goal(self):
        self.get_logger().info("Attempting to publish goal...")
        if not self.map or not self.robot_pos:
            self.get_logger().warn("publish_goal: STOPPED Map / robot_pos not initialised")
            return
        
        frontiers = self.map.find_frontiers()
        if not frontiers:
            self.get_logger().warn("publish_goal: No frontiers found")
            return

        bot_x, bot_y = self.map.coordinates_to_indices(self.robot_pos.x, self.robot_pos.y)
        furtherest_frontier = max(frontiers, key=lambda point: (point[0] - bot_x)**2 + (point[1] - bot_y)**2)
        goal_pose = self.map.indices_to_coordinates(furtherest_frontier[0], furtherest_frontier[1])

        # closet_frontier = min(frontiers, key=lambda point: (point[0] - bot_x)**2 + (point[1] - bot_y)**2)
        # goal_pose = self.map.indices_to_coordinates(closet_frontier[0], closet_frontier[1])

        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal_pose[0]
        goal_msg.pose.position.y = goal_pose[1]
        goal_msg.pose.orientation.w = 1.0

        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(f"Published new goal: ({goal_msg.pose.position.x}, {goal_msg.pose.position.y})")