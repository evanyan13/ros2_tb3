import numpy as np
from .global_planner_node import GlobalPlannerNode
from nav_msgs.msg import OccupancyGrid
from . import pixel_tolerance

class GlobalMap:
    def __init__(self, grid_map: OccupancyGrid):
        self.width = grid_map.info.width
        self.height = grid_map.info.height
        self.resolution = grid_map.info.resolution
        self.origin = grid_map.info.origin.position

        # Convert OccupancyGrid data to 2D numpy array
        self.data = np.array(grid_map.data).reshape(self.height, self.width)

    def get_occupancy_value_by_indices(self, i: int, j: int) -> int:
        """
        Access occupany value by indices of OccupancyGrid
        """
        if 0 <= i < self.height and 0 <= j < self.width:
            return self.data[i, j]
        return -1 # Out of bound value


    def get_occupancy_value_by_coordinates(self, x: float, y: float) -> int:
        """
        Access occupany value by real world coordinates
        """
        i, j = self.coordinates_to_indices(x, y)
        value = self.get_occupancy_value_by_indices(i, j)
        return value


    def coordinates_to_indices(self, x: float, y: float) -> tuple:
        """
        Convert the node's real world coordinates into gride indices on the OccupancyGrid
        """
        i = int((y - self.origin.y) / self.resolution)
        j = int((x - self.origin.x) / self.resolution)
        
        return i, j


    def is_node_free(self, node: GlobalPlannerNode) -> bool:
        """
        Checks whether a given node is in a free space on the map
        """
        i, j = self.coordinates_to_indices(node.x, node.y)

        # Checks if the node is out of bound
        if not (0 <= i <= self.width and 0 <= j <= self.height):
            return False

        tolerance = pixel_tolerance
        min_i, max_i = max(i - tolerance, 0), min(i + tolerance, self.height - 1)
        min_j, max_j = max(j - tolerance, 0), min(j + tolerance, self.width - 1)

        # Check if any of the cells in the tolerance area are occupied
        return np.all(self.data[min_i:max_i + 1, min_j:max_j + 1] < 100)


    def print_map(self, index):
        print('+' + '---+' * self.width)
        for i in range(self.height):
            row = '|'
            for j in range(self.width):
                curr_index = self.data[i, j]
                if curr_index == -1: # Unknown
                    char = '?'
                elif curr_index == 100: # Obstacle
                    char = '█'
                else:
                    char = ' ' # Free
                
                if (i, j) == index:
                    char = 'A'
                row += f'{char:>2} |'
            print(row)
            print('+' + '---+' * self.width)