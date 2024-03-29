import rclpy
import unittest
from unittest.mock import Mock
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from std_msgs.msg import Header

from navi_main.global_planner_package.global_path_planner import GlobalPathPlanner
from navi_main.global_planner_package.global_map import GlobalMap
from navi_main.global_planner_package.global_planner_node import GlobalPlannerNode


class TestGlobalPathPlanner(unittest.TestCase):
    def setUp(self) -> None:
        rclpy.init()
        self.path_planner = GlobalPathPlanner()

    def mock_occupancy_grid(self):
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header = Header()
        occupancy_grid.header.stamp = rclpy.time.Time().to_msg()
        occupancy_grid.header.frame_id = "map"
        occupancy_grid.info = MapMetaData()
        occupancy_grid.info.resolution = 1.0  # 1 meter per cell
        occupancy_grid.info.width = 10  # 10 cells wide
        occupancy_grid.info.height = 10  # 10 cells tall
        occupancy_grid.info.origin = Pose() # Empty origin
        occupancy_grid.data = [0] * 100  # Flat open space for simplicity

        return occupancy_grid

    def test_map_callback(self):
        test_grid = self.mock_occupancy_grid()
        self.path_planner.map_callback(test_grid)

        self.assertTrue(self.path_planner.is_map_loaded)
        self.assertIsNotNone(self.path_planner.map)
        self.assertIsInstance(self.path_planner.map, GlobalMap)

    def test_display_path(self):
        node = GlobalPlannerNode(0.0, 0.0)
        patch_list = [node]
        # Prevent ROS interactions
        self.path_planner.path_publisher.publish = Mock()

        path_msg = self.path_planner.display_path(patch_list)

        self.assertIsInstance(path_msg, Path)
        self.assertEqual(len(path_msg.poses), len(patch_list))

    def tearDown(self) -> None:
        self.path_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    unittest.main()
