import unittest
from nav_msgs.msg import OccupancyGrid
from navi_main.global_planner_package.global_map import GlobalMap
from navi_main.global_planner_package.astar_path_finder import find_astar_path
from navi_main.global_planner_package.global_planner_node import GlobalPlannerNode

class TestAstarPathFinder(unittest.TestCase):
    
    def setUp(self) -> None:
        self.grid = OccupancyGrid()
        self.grid.info.width = 5
        self.grid.info.height = 5
        self.grid.info.resolution = 1.0
        self.grid.data = [-1] * (self.grid.info.width * self.grid.info.height) # All unexplored
    
        # Make a barrier in the middle except for one pass-through
        for i in range(self.grid.info.width):
            self.grid.data[2 * self.grid.info.width + i] = 100 
        self.grid.data[2 * self.grid.info.width + 2] = -1  # Leave a gap

        self.map = GlobalMap(self.grid)

    def test_path_success(self):
        start = GlobalPlannerNode(0.0, 0.0)
        goal = GlobalPlannerNode(1.0, 1.0)

        path = find_astar_path(self.map, start, goal)
        self.map.print_map((start.x, start.y))
        print(path)
        self.assertTrue(len(path) > 0, "A path should be found")


if __name__ == '__main__':
    unittest.main()
