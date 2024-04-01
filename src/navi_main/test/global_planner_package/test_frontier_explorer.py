import unittest
from unittest.mock import MagicMock, patch
import rclpy

from navi_main.global_planner_package.frontier_explorer import FrontierExplorer
from navi_main.global_planner_package.global_map import GlobalMap
from navi_main.global_planner_package.global_node import GlobalPlannerNode

class TestFrontierExplorer(unittest.TestCase):
    def setUp(self) -> None:
        rclpy.init()
        self.mock_map = MagicMock(spec=GlobalMap)
        self.mock_robot_pos = MagicMock(spec=GlobalPlannerNode, x=1.0, y=2.0)
        self.explorer = FrontierExplorer(global_map=self.mock_map, robot_pos=self.mock_robot_pos)

    def test_publish_goal_no_frontier(self):
        self.mock_map.find_frontiers.return_value = []

        with patch.object(self.explorer.goal_publisher, 'publish') as mock_publish:
            self.explorer.publish_goal()

            mock_publish.assert_not_called()
    
    def test_publish_goal_with_frontier(self):
        self.mock_map.find_frontiers.return_value = [(2.0, 2.0), (3.0, 3.0)]
        self.mock_map.coordinates_to_indices.side_effect = lambda x, y: (x, y)
        self.mock_map.indices_to_coordinates.side_effect = lambda x, y: (x, y)
        
        with patch.object(self.explorer.goal_publisher, 'publish') as mock_publish:
            self.explorer.publish_goal()

            mock_publish.assert_called_once()
            published_msg = mock_publish.call_args[0][0]
            self.assertTrue(published_msg.pose.position.x in [2, 3])
            self.assertTrue(published_msg.pose.position.y in [2, 3])
            self.assertEqual(published_msg.pose.orientation.w, 1.0)

    def tearDown(self) -> None:
        self.explorer.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    unittest.main()

