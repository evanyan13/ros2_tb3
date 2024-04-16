import csv
import os
import numpy as np
import threading
from scipy.ndimage import binary_dilation
from nav_msgs.msg import OccupancyGrid

from .utils import MAP_PATH, UNEXPLORED, FREE, OCC_BIN, OBSTACLE, ROBOT_RADIUS
from .global_node import GlobalPlannerNode

class GlobalMap:
    def __init__(self, grid_map: OccupancyGrid):
        self.width = grid_map.info.width
        self.height = grid_map.info.height
        self.resolution = grid_map.info.resolution
        self.origin = grid_map.info.origin.position

        # Convert OccupancyGrid data to 2D numpy array
        self.orginial_data = np.array(grid_map.data).reshape(self.height, self.width)
        self.data = self.categorise_data(self.orginial_data)
        self.data = self.add_obstacle_buffer()

    def categorise_data(self, data):
        occ_bins = [UNEXPLORED, FREE, OCC_BIN, OBSTACLE]
        bin_indices = np.digitize(data, bins=occ_bins, right=False)

        cat_data = np.select(
            [bin_indices == 1, bin_indices == 2, bin_indices == 3],
            [UNEXPLORED, FREE, OBSTACLE],
            default=UNEXPLORED  # Use -1 for values outside the defined bins
        )
        return cat_data.astype(np.int8)
    
    def add_obstacle_buffer(self):
        buffer_size = int(ROBOT_RADIUS / self.resolution / 2)
        new_data = np.copy(self.data)

        for y in range(self.height):
            for x in range(self.width):
                if self.data[y, x] == OBSTACLE:
                    # Calculate the bounding box for the buffer zone
                    min_x = max(0, x - buffer_size)
                    max_x = min(self.width, x + buffer_size + 1)
                    min_y = max(0, y - buffer_size)
                    max_y = min(self.height, y + buffer_size + 1)

                    # Set the cells within the buffer zone to a specific value
                    for by in range(min_y, max_y):
                        for bx in range(min_x, max_x):
                                new_data[by][bx] = OBSTACLE

        return new_data

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
    
    def indices_to_node(self, i: int, j: int) -> GlobalPlannerNode:
        x, y = self.indices_to_coordinates(i, j)
        return GlobalPlannerNode(x, y)
    
    def get_occupancy_value_by_indices(self, i: int, j: int) -> int:
        """
        Access occupany value by indices of OccupancyGrid
        """
        i = int(i)
        j = int(j)
        if self.is_indice_valid(i, j):
            return self.data[j, i]
        return -1 # Unkown value

    def get_occupancy_value_by_coordinates(self, x: float, y: float) -> int:
        """
        Access occupany value by real world coordinates
        """
        i, j = self.coordinates_to_indices(x, y)
        value = self.get_occupancy_value_by_indices(i, j)
        return value

    def is_indice_valid(self, i: int, j: int) -> bool:
        return (0 <= i < self.height) and (0 <= j < self.width)

    def is_indice_avail(self, i: int, j: int):
        if self.is_indice_valid(i, j):
            value = self.get_occupancy_value_by_indices(i, j)
            return value == FREE
        return False
        
    def is_node_valid(self, node: GlobalPlannerNode) -> bool:
        i, j = self.coordinates_to_indices(node.x, node.y)
        return self.is_indice_valid(i, j)
    
    def is_node_avail(self, node: GlobalPlannerNode) -> bool:
        i, j = self.coordinates_to_indices(node.x, node.y)
        return self.is_indice_avail(i, j)

    def generate_occupancy(self):
        """
        Export occupancy grid data as csv to MAP_PATHs
        """
        categorised_filename = os.path.join(MAP_PATH, 'categorised_map.csv')
        original_filename = os.path.join(MAP_PATH, 'orginial_map.csv')
        
        # Save categorised data
        with open(categorised_filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            for row in self.data:
                writer.writerow(row)

        # Save original data
        with open(original_filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            for row in self.orginial_data:
                writer.writerow(row)

    def print_map(self, start, end):
        if start == end:
            print("We have reached the goal")

        print('+' + '---+' * self.width)
        for i in range(self.height):
            row = '|'
            for j in range(self.width):
                curr_index = self.data[j, i]
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