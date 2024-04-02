import csv
import os
import numpy as np
from nav_msgs.msg import OccupancyGrid

from .utils import pixel_tolerance
from .global_node import GlobalPlannerNode

class GlobalMap:
    def __init__(self, grid_map: OccupancyGrid):
        self.width = grid_map.info.width
        self.height = grid_map.info.height
        self.resolution = grid_map.info.resolution
        self.origin = grid_map.info.origin.position

        # Convert OccupancyGrid data to 2D numpy array
        self.data = np.array(grid_map.data).reshape(self.height, self.width)

    def find_frontiers(self):
        """
        Find available frontiers in current data
        """
        unknown_value = -1
        frontiers = []

        for y in range(1, self.height - 1):
            for x in range(1, self.width - 1):
                if self.data[y, x] == 0:
                    neighbours = [(x + dx, y + dy) for dx, dy
                                  in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (1, -1), (-1, 1), (1, 1)]]
                    if any(self.data[ny][nx] == unknown_value for nx, ny in neighbours):
                        frontiers.append((x, y))
        return frontiers
    
    def generate_occupancy(self):
        filename = 'map.csv'
        
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            for row in self.data:
                writer.writerow(row)
        
        return os.path.abspath(filename)

    def get_occupancy_value_by_indices(self, i: int, j: int) -> int:
        """
        Access occupany value by indices of OccupancyGrid
        """
        if 0 <= i < self.height and 0 <= j < self.width:
            return self.data[i, j]
        return -1 # Unkown value

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
        i = int((x - self.origin.x) / self.resolution)
        j = int((y - self.origin.y) / self.resolution)
        
        return i, j
    
    def indices_to_coordinates(self, i: int, j: int) -> tuple:
        """
        Convert the indices on the OccupancyGrid to real world coordinates
        """
        x = float(i * self.resolution + self.origin.x)
        y = float(j * self.resolution + self.origin.y)

        return x, y
    
    def is_node_valid(self, node: GlobalPlannerNode) -> bool:
        """
        Checks if node is valid/within boundary of the map
        """
        i, j = self.coordinates_to_indices(node.x, node.y)
        return (i < self.height) and (j < self.width)

    def is_node_valid_by_indices(self, i: int, j: int) -> bool:
        """
        Checks if node is valid/within boundary of the map by indices
        """
        return (i < self.height) and (j < self.width)
    
    def is_node_avail(self, node: GlobalPlannerNode) -> bool:
        """
        Checks whether a given node is in a free space on the map
        """
        i, j = self.coordinates_to_indices(node.x, node.y)

        # Checks if the node is out of bound
        if self.is_node_valid_by_indices(i, j):
            return (self.get_occupancy_value_by_indices(i, j) < 100)

    def is_node_free(self, node: GlobalPlannerNode) -> bool:
        """
        Checks whether a given node is in a free space on the map
        """
        i, j = self.coordinates_to_indices(node.x, node.y)

        # Checks if the node is out of bound
        if not self.is_node_valid_by_indices(i, j):
            return False

        tolerance = pixel_tolerance
        min_i, max_i = max(i - tolerance, 0), min(i + tolerance, self.height - 1)
        min_j, max_j = max(j - tolerance, 0), min(j + tolerance, self.width - 1)

        # Check if any of the cells in the tolerance area are occupied
        return np.all(self.data[min_i:max_i + 1, min_j:max_j + 1] < 100)

    def print_map(self, start, end):
        if start == end:
            print("We have reached the goal")

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
                
                if (i, j) == start:
                    char = 'A'
                if (i, j) == end:
                    char = 'Z'
                row += f'{char:>2} |'
            print(row)
            print('+' + '---+' * self.width)