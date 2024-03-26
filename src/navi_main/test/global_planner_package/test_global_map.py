import unittest
from nav_msgs.msg import OccupancyGrid
from navi_main.global_planner_package.global_map import GlobalMap
from navi_main.global_planner_package.global_planner_node import GlobalPlannerNode

VALID_NODE = GlobalPlannerNode(1, 1)
INVALID_NODE = GlobalPlannerNode(-1, -1)
FREE_NODE = GlobalPlannerNode(1, 1)
OCCUPIED_NODE = GlobalPlannerNode(2, 3)

class TestMapPrinting(unittest.TestCase):
    # Setup an example OccupancyGrid
    def setUp(self):
        self.grid = OccupancyGrid()
        self.grid.info.width = 5
        self.grid.info.height = 5
        self.grid.info.resolution = 1.0  # 1 meter per cell
        self.grid.data = [-1, 0, 50, 100, -1,
                            0, 0, 50, 100, -1,
                            50, 50, 50, 100, -1,
                            100, 100, 100, 100, -1,
                            -1, -1, -1, -1, -1]  # Sample grid data
        self.map = GlobalMap(self.grid)

    def test_print_map(self):
        start_index = (2, 2)
        end_index = (4, 4)
        self.map.print_map(start_index, end_index)
    
    def test_get_occupancy_value_by_indices(self):
        self.assertEqual(self.map.get_occupancy_value_by_indices(0, 2), 50)
        self.assertEqual(self.map.get_occupancy_value_by_indices(1, 1), 0)
        self.assertEqual(self.map.get_occupancy_value_by_indices(3, 4), -1)

    # is_node_valid
    def test_is_node_valid_valid(self):
        self.assertTrue(self.map.is_node_valid(VALID_NODE), "Node -> Valid, Returned -> False")

    def test_is_node_valid_invalid(self):
        self.assertFalse(self.map.is_node_valid(INVALID_NODE), "Node -> Invalid, Returned -> True")

    # is_node_avail
    def test_is_node_avail_avail(self):
        self.assertTrue(self.map.is_node_valid(FREE_NODE), "Node -> Available, Returned -> Occupied")

    def test_is_node_avail_unavail(self):
        self.assertFalse(self.map.is_node_avail(OCCUPIED_NODE), "Node -> Occupied, Returned -> Available")

    # is_node_free
    def test_is_node_free_free_space(self):
        self.assertTrue(self.map.is_node_free(FREE_NODE), "Node -> Free, Returned -> False")

    def test_is_node_free_occupied_space(self):
        self.assertFalse(self.map.is_node_free(OCCUPIED_NODE), "Node -> Occupied, Returned -> True")

    def test_is_node_free_out_of_bound(self):
        self.assertFalse(self.map.is_node_free(INVALID_NODE), "Node -> Out of Bound, Returned -> True")


if __name__ == '__main__':
    unittest.main()
