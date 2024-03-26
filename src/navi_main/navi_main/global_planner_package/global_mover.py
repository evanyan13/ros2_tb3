import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from math import atan2, pi
from numpy import np

move_tolerance = 0.1
scan_tolerance_front = 0.4
scan_tolerance_side = 0.3
rotate_tolerance = 0.004
linear_velocity = 0.25
angular_velocity = 1.5

class GlobalMover(Node):
    def __init__(self, planner):
        super().__init__('global_mover')

        self.robat_pos = None
        self.path_deviation = 0.0
        self.planner = planner

        self.is_shutdown = False
        self.is_moving = False
        self.is_obstacle_ahead = False

        self.mover_subscriber = self.create_subscription(Path, '/move', self.move_callback, 10)
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def 
    
