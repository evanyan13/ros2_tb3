import unittest
from nav_msgs.msg import OccupancyGrid
from navi_main.global_planner_package.global_map import GlobalMap
from navi_main.global_planner_package.astar_path_finder import find_astar_path, print_path
from navi_main.global_planner_package.global_planner_node import GlobalPlannerNode

START = GlobalPlannerNode(x=0.0, y=0.0)
END = GlobalPlannerNode(x=3.0, y=0.0)

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

        self.map_open = GlobalMap(self.grid)

        # Make a barrier in the middle
        for i in range(self.grid.info.width):
            self.grid.data[2 * self.grid.info.width + i] = 100 
        
        self.map_close = GlobalMap(self.grid)

    def test_path_success(self):
        path = find_astar_path(self.map_open, START, END)
        self.map_open.print_map((START.x, START.y), (END.x, END.y))
        if path:
            print_path(path)
        self.assertTrue(len(path) > 0, "A path should be found")

    def test_path_failure(self):
        path = find_astar_path(self.map_close, START, END)
        self.map_close.print_map((START.x, START.y), (END.x, END.y))
        print_path(path)
        self.assertFalse(len(path) > 0, "A path does not exist")

if __name__ == '__main__':
    unittest.main()
