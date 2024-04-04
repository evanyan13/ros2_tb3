import unittest
import random
import csv
import numpy as np
import matplotlib.pyplot as plt

from navi_main.global_planner_package.astar_path_finder import find_astar_path
from navi_main.global_planner_package.global_map import GlobalMap
from navi_main.global_planner_package.global_node import GlobalPlannerNode
from navi_main.global_planner_package.utils import print_path, read_occupancy_grid_from_csv, MAP_PATH

START = GlobalPlannerNode(x=24.0, y=47.0)
END = GlobalPlannerNode(x=34.0, y=47.0)

class TestAstarPathFinder(unittest.TestCase):
    
    def setUp(self) -> None:
        filename = MAP_PATH  # Path to your CSV file
        self.grid = read_occupancy_grid_from_csv(filename)
        self.map = GlobalMap(self.grid)

        self.start = find_random_point(self.map)
        self.goal = find_random_point(self.map, self.start)

    def test_path_random_points(self):
        path = find_astar_path(self.map, self.start, self.goal)
        if path:
            print_path(path)
            plot_path(self, self.map, path)
        self.assertTrue(len(path) > 0, "A path should be found")

    # def test_path_fixed_points(self):
    #     path = find_astar_path(self.map, START, END)
    #     if path:
    #         print_path(path)
    #         plot_path(self, self.map, path)
    #     self.assertTrue(len(path) > 0, "A path should be found")


def find_random_point(map: GlobalMap, exclude_node = None):
    exclude = []
    if exclude_node is not None:
        exclude.append((exclude_node.x, exclude_node.y))

    while True:
        x = random.randint(0, map.width - 1)
        y = random.randint(0, map.height - 1)
        node = GlobalPlannerNode(x=x, y=y)
        if map.is_indice_avail(x, y) and (node.x, node.y) not in exclude:
            return node
        # occ_value = map.get_occupancy_value_by_indices(x, y)
        # print(f"Checking node: ({node.x, node.y}) | in_indice_avail {map.is_indice_avail(x, y)} | occ_value {occ_value} | not in exclude {(node.x, node.y) not in exclude}")


def plot_path(self, map: GlobalMap, path):
    # Convert path nodes to map coordinates
    x_coords, y_coords = zip(*[(node.x, node.y) for node in path])

    # Plot the occupancy grid
    cmap = plt.cm.gray
    norm = plt.Normalize(vmin=-1, vmax=100)
    plt.imshow(map.data.reshape((map.height, map.width)), cmap=cmap, norm=norm, origin='lower')
    plt.scatter(x_coords, y_coords, c='red')  # Path in red
    plt.scatter(self.start.x, self.start.y, c='blue')  # Start in blue
    plt.scatter(self.goal.x, self.goal.y, c='green')  # End in green
    plt.title("A* Path Finding")
    plt.show()


if __name__ == '__main__':
    unittest.main()
