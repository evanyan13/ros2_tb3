import unittest
from nav_msgs.msg import OccupancyGrid
 
from navi_main.global_planner_package.utils import read_occupancy_grid_from_csv, display_with_frontier
from navi_main.global_planner_package.global_map import GlobalMap
from navi_main.global_planner_package.global_node import GlobalPlannerNode

VALID_NODE = GlobalPlannerNode(1, 1)
INVALID_NODE = GlobalPlannerNode(-1, -1)
FREE_NODE = GlobalPlannerNode(1, 1)
OCCUPIED_NODE = GlobalPlannerNode(2, 3)

class TestMapPrinting(unittest.TestCase):
    # Setup an example OccupancyGrid
    def setUp(self):
        filename = '/home/evanyan13/colcon_ws/src/navi_main/test/map.csv'
        self.grid = read_occupancy_grid_from_csv(filename)
        self.map_real = GlobalMap(self.grid)

        self.grid = OccupancyGrid()
        self.grid.info.width = 5
        self.grid.info.height = 5
        self.grid.info.resolution = 1.0  # 1 meter per cell
        self.grid.data = [-1, 0, 25, 100, -1,
                            0, 0, 50, 100, -1,
                            50, 50, 50, 100, -1,
                            100, 100, 100, 100, -1,
                            -1, -1, -1, -1, -1]  # Sample grid data
        self.map_mock = GlobalMap(self.grid)

    def test_find_frontiers(self):
        frontiers = self.map_real.find_frontiers()
        display_with_frontier(self.map_real.data, frontiers)        
    
    def test_get_occupancy_value_by_coordinate(self):
        self.assertEqual(self.map_mock.get_occupancy_value_by_coordinates(2, 1), 50)
        self.assertEqual(self.map_mock.get_occupancy_value_by_coordinates(3, 2), 100)
        
    def test_is_node_valid_true(self):
        test_node = GlobalPlannerNode()

    # is_node_valid
    def test_is_node_valid_valid(self):
        self.assertTrue(self.map_mock.is_node_valid(VALID_NODE), "Node -> Valid, Returned -> False")

    def test_is_node_valid_invalid(self):
        self.assertFalse(self.map_mock.is_node_valid(INVALID_NODE), "Node -> Invalid, Returned -> True")

    # is_node_avail
    def test_is_node_avail_avail(self):
        self.assertTrue(self.map_mock.is_node_valid(FREE_NODE), "Node -> Available, Returned -> Occupied")

    def test_is_node_avail_unavail(self):
        self.assertFalse(self.map_mock.is_node_avail(OCCUPIED_NODE), "Node -> Occupied, Returned -> Available")


if __name__ == '__main__':
    unittest.main()
