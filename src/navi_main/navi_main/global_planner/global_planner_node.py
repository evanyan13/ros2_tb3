import rclpy
import global_planner
import math
from tf import transformations
from geometry_msgs.msg import Pose, PoseStamped

class GlobalPlannerNode:
    def __init__(
            self,
            x: float = 0.0,
            y: float = 0.0,
            theta: float = 0.0,
            parent = None
    ):
        self.parent = parent
        self.x = x
        self.y = y
        self.theta = theta

        self.g = 0 # Current cost
        self.h = 0 # Estimated cost to goal node
        self.f = 0 # Total cost

def calculate_distance(self, end) -> float:
    """
    Returns euclidean distance btw two nodes
    """
    return math.sqrt((self.x - end.x) ** 2 + (self.y - end.y) ** 2)

def generate_neighbours(self, map_resolution: float) -> list:
    neighbours = []
    step = (global_planner.pixel_tolerance + 1) * map_resolution
    moves = [(0, step),
             (step, step),
             (step, 0),
             (step, -step),
             (0, -step),
             (-step, -step),
             (-step, 0),
             (-step, step)]

    for move in moves:
        new_x = self.x + move[0]
        new_y = self.y + move[1]
        neighbours.append(GlobalPlannerNode(new_x, new_y))
    
    return neighbours