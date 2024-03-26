import unittest
from navi_main.global_planner_package.global_planner_node import GlobalPlannerNode

class TestGlobalPlannerNode(unittest.TestCase):

    def test_calculate_distance(self):
        node_a = GlobalPlannerNode(0, 0)
        node_b = GlobalPlannerNode(3, 4)
        self.assertEqual(node_a.calculate_distance(node_b), 5)


if __name__ == "__main__":
    unittest.main()