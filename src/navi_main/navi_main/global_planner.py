import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np

class GlobalPlanner(Node):
    def __init__(self):
        super.__init__('global_planner_node')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )
        self.publishers = self.create_publisher(
            Path,
            'global_plan',
            10
        )

    def map_callback(self, msg):
        start, goal = self.get_start_goal_pos()
        
