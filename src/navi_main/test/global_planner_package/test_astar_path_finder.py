import unittest
import csv
import random
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point

from navi_main.global_planner_package.astar_path_finder import find_astar_path
from navi_main.global_planner_package.global_map import GlobalMap
from navi_main.global_planner_package.global_node import GlobalPlannerNode
from navi_main.global_planner_package.utils import print_path

START = GlobalPlannerNode(x=0.0, y=0.0)
END = GlobalPlannerNode(x=3.0, y=0.0)

class TestAstarPathFinder(unittest.TestCase):
    
    def setUp(self) -> None:
        filename = '/home/evanyan13/colcon_ws/map.csv'  # Path to your CSV file
        self.grid = read_occupancy_grid_from_csv(filename)
        self.map = GlobalMap(self.grid)

        self.start = find_random_point(self.map)
        self.goal = find_random_point(self.map)

    def test_path_fixed_points(self):
        path = find_astar_path(self.map, START, END)
        if path:
            print_path(path)
        self.assertTrue(len(path) > 0, "A path should be found")

    def test_path_random_points(self):
        path = find_astar_path(self.map, self.start, self.goal)
        if path:
            print_path(path)
            plot_path(self, self.map, path)
        self.assertTrue(len(path) > 0, "A path should be found")


def read_occupancy_grid_from_csv(filename):
    with open(filename, 'r') as csvfile:
        reader = csv.reader(csvfile)
        data = np.array([list(map(int, row)) for row in reader], dtype=np.int8)

    resolution=1.0
    origin=(0.0, 0.0, 0.0)
    height, width = data.shape

    grid = OccupancyGrid()
    grid.info = MapMetaData()
    grid.info.resolution = resolution
    grid.info.width = width
    grid.info.height = height
    grid.info.origin = Pose()
    grid.info.origin.position.x = origin[0]
    grid.info.origin.position.y = origin[1]
    grid.info.origin.position.z = 0.0
    grid.data = [int(value) for value in data.flatten()]

    return grid


def find_random_point(map: GlobalMap):
    while True:
        x = random.randint(0, map.width - 1)
        y = random.randint(0, map.height - 1)
        node = GlobalPlannerNode(x=map.indices_to_coordinates(x, y)[0], y=map.indices_to_coordinates(x, y)[1])
        if map.is_node_avail(node):
            return node
    

def plot_path(self, map: GlobalMap, path):
    # Convert path nodes to map coordinates
    x_coords, y_coords = zip(*[(node.x, node.y) for node in path])

    # Plot the occupancy grid
    plt.imshow(map.data.reshape((map.height, map.width)), cmap='gray', origin='lower')
    plt.scatter(x_coords, y_coords, c='red')  # Path in red
    plt.scatter(self.start.x, self.start.y, c='blue')  # Start in blue
    plt.scatter(self.goal.x, self.goal.y, c='green')  # End in green
    plt.title("A* Path Finding")
    plt.show()


if __name__ == '__main__':
    unittest.main()
