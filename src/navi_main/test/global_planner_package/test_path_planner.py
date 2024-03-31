import random
import rclpy
import unittest
import math
import threading
import time
import numpy as np
from unittest.mock import Mock, patch
from geometry_msgs.msg import Pose, PoseStamped
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

    @patch('navi_main.global_planner_package.global_planner_node.GlobalPlannerNode.from_tf')
    def test_goal_callback(self, mock_from_tf):
        # Mock transformation lookup
        mock_transform = Mock()
        mock_transform.translation.x = 0.0
        mock_transform.translation.y = 0.0
        mock_transform.translation.z = 0.0
        mock_transform.rotation.x = 0.0
        mock_transform.rotation.y = 0.0
        mock_transform.rotation.z = 0.0
        mock_transform.rotation.w = 1.0

        self.path_planner.tf_buffer.lookup_transform = Mock(return_value=mock_transform)

        self.path_planner.map = Mock(spec=GlobalMap)
        self.path_planner.map.is_node_free = Mock(return_value=True)
        self.path_planner.calculate_path = Mock()
        
        # Set up the goal pose
        test_goal = PoseStamped()
        test_goal.pose.position.x = 1.0
        test_goal.pose.position.y = 1.0
        test_goal.pose.orientation.w = 1.0

        self.path_planner.goal_callback(test_goal)

        self.path_planner.tf_buffer.lookup_transform.assert_called_once()
        self.path_planner.calculate_path.assert_called_once()
        self.assertFalse(self.path_planner.is_goal_cancelled)
        self.assertIsNotNone(self.path_planner.goal)

    @patch('navi_main.global_planner_package.global_path_planner.find_astar_path')
    def test_calculate_path(self, mock_find_astar_path):
        self.path_planner.map = Mock()
        self.path_planner.start = GlobalPlannerNode(0.0, 0.0)
        self.path_planner.goal = GlobalPlannerNode(1.0, 1.0)
        mock_find_astar_path.return_value = [self.path_planner.start, self.path_planner.goal]
        
        self.path_planner.mover_publisher.publish = Mock()
        self.path_planner.get_logger().info = Mock()
        
        self.path_planner.calculate_path()

        mock_find_astar_path.called_once_with(self.path_planner.map,
                                              self.path_planner.start,
                                              self.path_planner.goal)
        self.path_planner.get_logger().info.assert_any_call("Calculating path")
        self.path_planner.get_logger().info.assert_any_call("Path Received")
    
    def test_smooth_path_bspline(self):
        test_path_list = [GlobalPlannerNode(x=random.uniform(0, 10), y=random.uniform(0, 10)) for _ in range(10)]

        smoothed_path = self.path_planner.smooth_path_bspline(test_path_list)

        # Basic Check
        self.assertGreater(len(smoothed_path), len(test_path_list), "Smoothed path should have more points than original.")
        self.assertIsInstance(smoothed_path, list)
        self.assertTrue(all(isinstance(node, GlobalPlannerNode) for node in smoothed_path), "All elements should be GlobalPlannerNode instances.")

        # Smoothness Check
        original_angles = get_path_angles(test_path_list)
        smoothed_angles = get_path_angles(smoothed_path)
        original_std = np.std(original_angles)
        smoothed_std = np.std(smoothed_angles)
        self.assertLess(smoothed_std, original_std, "Smoothed path should have less angle variation.")

    def test_display_path(self):
        node = GlobalPlannerNode(0.0, 0.0)
        patch_list = [node]
        # Prevent ROS interactions
        self.path_planner.path_publisher.publish = Mock()

        path_msg = self.path_planner.display_path(patch_list)

        self.assertIsInstance(path_msg, Path)
        self.assertEqual(len(path_msg.poses), len(patch_list))

    def test_wait_for_map(self):
        self.path_planner.is_map_loaded = False

        wait_thread = threading.Thread(target=self.path_planner.wait_for_map)
        wait_thread.start()

        # Simulate the map being loaded
        time.sleep(0.1)
        self.path_planner.is_map_loaded = True

        # Wait for the thread to finish
        wait_thread.join(timeout=1)

        # Check if the method exited correctly
        self.assertTrue(self.path_planner.is_map_loaded)

    # @patch('navi_main.global_planner_package.global_planner_node.GlobalPlannerNode.from_pose')
    def test_send_goal(self):
        test_pose = PoseStamped()
        self.path_planner.calculate_path = Mock(return_value=True)

        result = self.path_planner.send_goal(test_pose)

        # mock_from_pose.assert_called_once_with(test_pose)
        self.path_planner.calculate_path.assert_called_once()
        self.assertFalse(self.path_planner.is_goal_cancelled)
        self.assertIsInstance(self.path_planner.goal, GlobalPlannerNode)
        self.assertTrue(result) 

    @patch('navi_main.global_planner_package.global_mover.GlobalMover.stop_moving')
    def test_cancel_goal(self, mock_stop_moving):
        self.path_planner.cancel_goal()

        mock_stop_moving.assert_called_once()
        self.assertTrue(self.path_planner.is_goal_cancelled)

    def tearDown(self) -> None:
        self.path_planner.destroy_node()
        rclpy.shutdown()


def get_path_angles(path):
    """
    Calculate the angles for each set of three consecutive points in the path.
    """
    angles = []
    for i in range(1, len(path) - 1):
        angle = calculate_angle(path[i-1], path[i], path[i+1])
        angles.append(angle)
    return angles


def calculate_angle(p1, p2, p3):
    """
    Calculate angle between three points where p2 is the vertex.
    """
    a = math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
    b = math.sqrt((p2.x - p3.x)**2 + (p2.y - p3.y)**2)
    c = math.sqrt((p3.x - p1.x)**2 + (p3.y - p1.y)**2)
    if a == 0 or b == 0:
        return 0
    
    cos_value = ((a ** 2 + b ** 2 - c ** 2) / (2 * a * b))
    cos_value = max(min(cos_value, 1), -1) # To restrict the value of acos to [-1, 1]
    angle = math.acos(cos_value)
    return math.degrees(angle) 


if __name__ == '__main__':
    unittest.main()
