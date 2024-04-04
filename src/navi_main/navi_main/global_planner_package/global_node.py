import rclpy
import math
from geometry_msgs.msg import Pose, PoseStamped

from .utils import euler_from_quaternion, pixel_tolerance

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

    def generate_neighbours(self, map_resolution) -> list:
        neighbours = []
        step = pixel_tolerance * map_resolution
        moves = [(0, step), (step, step), (step, 0), (step, -step),
                 (0, -step), (-step, -step), (-step, 0), (-step, step)]

        for move in moves:
            new_x = self.x + move[0]
            new_y = self.y + move[1]
            neighbours.append(GlobalPlannerNode(new_x, new_y))
        
        return neighbours
    
    def generate_neighbours_indices(self):
        neighbours = []
        moves = [(0, 1), (1, 1), (1, 0), (1, -1),
                 (0, -1), (-1, -1), (-1, 0), (-1, 1)]
        
        for move in moves:
            new_x = self.x + move[0]
            new_y = self.y + move[1]
            neighbours.append((new_x, new_y))

        return neighbours
    
    def backtrack_path(self) -> list:
        """
        Backtrack to find path from start to current node
        """
        path = []
        current_node = self
        while current_node is not None:
            path.append(current_node)
            current_node = current_node.parent
        path.reverse()
        return path
    
    def __lt__(self, other):
        """
        Less than comparison for priority queue sorting bacsed on f value
        """
        return self.f < other.f

    def equals(self, other):
        return self.x == other.x and self.y == other.y
    
    @staticmethod
    def from_pose(pose: Pose):
        new_state = GlobalPlannerNode()
        new_state.x = pose.position.x
        new_state.y = pose.position.y

        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        (roll, pitch, yaw) = euler_from_quaternion(*quaternion) # Unpack tuples into separate arguments
        new_state.theta = yaw

        return new_state
    
    @staticmethod
    def from_tf(position: list, quaternion: list):
        new_state = GlobalPlannerNode()
        new_state.x = position[0]
        new_state.y = position[1]

        (roll, pitch, yaw) = euler_from_quaternion(*quaternion)
        new_state.theta = yaw

        return new_state
    
    def to_pose_stamped(self, current_time):
        pose = PoseStamped()
        pose.header.stamp = current_time
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation.w = 1.0

        return pose

    def coord_node_to_indice_node(self, map):
        node_i, node_j = map.coordinates_to_indices(self.x, self.y)
        indice_node = GlobalPlannerNode(node_i, node_j, self.theta, self.parent)
        return indice_node

    def indice_node_to_coord_node(self, map):
        coord_x, coord_y = map.indices_to_coordinates(self.x, self.y)
        coord_node = GlobalPlannerNode(coord_x, coord_y, self.theta, self.parent)
        return coord_node