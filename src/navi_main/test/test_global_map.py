import unittest
from nav_msgs.msg import OccupancyGrid
from navi_main.global_planner_package import GlobalMap

class TestMapPrinting(unittest.TestCase):
    def test_print_map(self):
        # Setup an example OccupancyGrid
        grid = OccupancyGrid()
        grid.info.width = 5
        grid.info.height = 5
        grid.info.resolution = 1.0  # 1 meter per cell
        grid.data = [-1, 0, 50, 100, -1,
                     0, 0, 50, 100, -1,
                     50, 50, 50, 100, -1,
                     100, 100, 100, 100, -1,
                     -1, -1, -1, -1, -1]  # Sample grid data

        map = GlobalMap(grid)
        starting_index = (2, 2)
        map.print_map(starting_index)

if __name__ == '__main__':
    unittest.main()
