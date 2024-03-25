import unittest
from navi_main.global_planner_package.global_map import GlobalMap
from navi_main.global_planner_package.astar_path_finder import find_astar_path
from navi_main.global_planner_package import pixel_tolerance
from nav_msgs.msg import OccupancyGrid

class TestAStarPathFinder(unittest.TestCase):
    def setUp(self):
        # Setup an example OccupancyGrid
        self.grid = OccupancyGrid()
        self.grid.info.width = 10
        self.grid.info.height = 10
        self.grid.info.resolution = 1.0  # 1 meter per cell
        self.grid.data = [0] * 100  # Initialize grid to free space

        # You will have to define the GlobalMap and Node classes accordingly
        self.map = GlobalMap(self.grid)

        # Set pixel_tolerance if it's used in your implementation
        pixel_tolerance = 5

    def test_path_exists(self):
        # Define a start and goal that you know should work on your map
        start = (1, 1)
        goal = (8, 8)
        
        # Call the find_astar_path function
        path = find_astar_path(self.map, start, goal)
        
        # Test that a path was found
        self.assertTrue(len(path) > 0, "No path found when one exists")

    def test_no_path(self):
        # Set obstacles in the grid to ensure no path exists
        for i in range(self.grid.info.width * self.grid.info.height):
            self.grid.data[i] = 100  # Set all cells to occupied

        # Update the map with the new grid
        self.map = GlobalMap(self.grid)

        # Try to find a path
        start = (1, 1)
        goal = (8, 8)
        path = find_astar_path(self.map, start, goal)
        
        # Test that no path was found
        self.assertTrue(len(path) == 0, "Path found when none should exist")

if __name__ == '__main__':
    unittest.main()
