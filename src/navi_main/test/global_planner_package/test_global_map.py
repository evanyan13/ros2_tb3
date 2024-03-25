import unittest
from nav_msgs.msg import OccupancyGrid
from navi_main.global_planner_package.global_map import GlobalMap
from navi_main.global_planner_package.global_planner_node import GlobalPlannerNode

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
        starting_index = (2, 2)
        self.map.print_map(starting_index)
    
    def test_get_occupancy_value_by_indices(self):
        self.assertEqual(self.map.get_occupancy_value_by_indices(0, 2), 50)
        self.assertEqual(self.map.get_occupancy_value_by_indices(1, 1), 0)
        self.assertEqual(self.map.get_occupancy_value_by_indices(3, 4), -1)

    def test_is_node_free_free_space(self):
        node = GlobalPlannerNode(1, 1)
        self.assertTrue(self.map.is_node_free(node), "Node -> Free, Returned -> False")

    def test_is_node_free_occupied_space(self):
        node = GlobalPlannerNode(0, 3)
        self.assertFalse(self.map.is_node_free(node), "Node -> Occupied, Returned -> True")

    def test_is_node_free_edge_space(self):
        node = GlobalPlannerNode(3, 0)
        self.assertFalse(self.map.is_node_free(node), "Node -> Occupied, Returned -> True")
    
    def test_is_node_free_out_of_bound(self):
        node = GlobalPlannerNode(-1, -1)
        self.assertFalse(self.map.is_node_free(node), "Node -> Out of Bound, Returned -> True")


if __name__ == '__main__':
    unittest.main()
