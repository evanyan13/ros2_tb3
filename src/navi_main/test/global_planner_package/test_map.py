import unittest
from nav_msgs.msg import OccupancyGrid
 
from navi_main.global_planner_package.utils import read_occupancy_grid_from_csv, display_with_frontier, MAP_PATH
from navi_main.global_planner_package.global_map import GlobalMap
from navi_main.global_planner_package.global_node import GlobalPlannerNode

VALID_NODE = GlobalPlannerNode(2, 2)
VALID_INDICES = 1, 1
INVALID_NODE = GlobalPlannerNode(-2, -2)
INVALID_INDICES = -1, -1
AVAIL_INDICES = 0, 1
AVAIL_NODE = GlobalPlannerNode(0, 2)
OCCUPIED_INDICES = 1, 2
OCCUPIED_NODE = GlobalPlannerNode(4, 6)

class TestMap(unittest.TestCase):
    # Setup an example OccupancyGrid
    def setUp(self):
        filename = MAP_PATH
        self.grid = read_occupancy_grid_from_csv(filename)
        self.map_real = GlobalMap(self.grid)

        self.grid = OccupancyGrid()
        self.grid.info.width = 5
        self.grid.info.height = 5
        self.grid.info.resolution = 2.0  # 1 meter per cell
        self.grid.data = [-1, 0, 25, 100, -1,
                            0, 0, 50, 100, -1,
                            50, 66, 50, 100, -1,
                            100, 100, 100, 49, -1,
                            -1, -1, -1, -1, -1]  # Sample grid data
        self.map_mock = GlobalMap(self.grid)

    def test_coordinate_to_indices(self):
        ix, iy = self.map_mock.coordinates_to_indices(4, 2)
        self.assertEqual((ix, iy), (2, 1))

    def test_indices_to_coordinate(self):
        cx, cy = self.map_mock.indices_to_coordinates(2, 1)
        self.assertEqual((cx, cy), (4, 2))

    def test_get_occupancy_value_by_indices(self):
        self.assertEqual(self.map_mock.get_occupancy_value_by_indices(2, 1), 100)
        self.assertEqual(self.map_mock.get_occupancy_value_by_indices(0, 2), 0)

    def test_get_occupancy_value_by_coordinate(self):
        self.assertEqual(self.map_mock.get_occupancy_value_by_coordinates(4, 2), 100)
        self.assertEqual(self.map_mock.get_occupancy_value_by_coordinates(6, 6), 0)

    def test_check_indice_valid(self):
        self.assertTrue(self.map_mock.is_indice_valid(*VALID_INDICES))
        self.assertFalse(self.map_mock.is_indice_valid(*INVALID_INDICES))
    
    def test_check_indice_avail(self):
        self.assertTrue(self.map_mock.is_indice_avail(*AVAIL_INDICES))
        self.assertFalse(self.map_mock.is_indice_avail(*OCCUPIED_INDICES))

    def test_check_node_valid(self):
        self.assertTrue(self.map_mock.is_node_valid(VALID_NODE))
        self.assertFalse(self.map_mock.is_node_valid(INVALID_NODE))

    def test_check_node_avail(self):
        self.assertTrue(self.map_mock.is_node_avail(AVAIL_NODE))
        self.assertFalse(self.map_mock.is_node_avail(OCCUPIED_NODE))

    # def test_find_frontiers(self):
    #     frontiers = self.map_real.find_frontiers()
    #     display_with_frontier(self.map_real.data, frontiers)        
    

if __name__ == '__main__':
    unittest.main()
