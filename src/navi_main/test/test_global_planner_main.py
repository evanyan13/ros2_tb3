import rclpy
import unittest
import numpy as np
import matplotlib.pyplot as plt
from unittest.mock import MagicMock, patch
from geometry_msgs.msg import Point, Quaternion, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path

from navi_main.global_planner_main import GlobalPlannerMain
from navi_main.global_planner_package.global_map import GlobalMap
from navi_main.global_planner_package.global_planner_node import GlobalPlannerNode

class TestGlobalPlannerMain(unittest.TestCase):
    def setUp(self) -> None:
        rclpy.init()
        self.node = GlobalPlannerMain()
    
    def test_map_callback(self):
        map_msg = OccupancyGrid()

        self.node.map_callback(map_msg)

        self.assertIsNotNone(self.node.map)
        self.assertIsInstance(self.node.map, GlobalMap)

    def test_odom_callback(self):
        odom_msg = Odometry()
        odom_msg.pose.pose.position = Point(x=1.0, y=2.0, z=0.0)
        odom_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=1.0)

        with patch('navi_main.global_planner_main.euler_from_quaternion') as mock_euler:
            mock_euler.return_value = (0.0, 0.0, 0.0)
            self.node.odom_callback(odom_msg)

        expected_robot_pos = (1.0, 2.0, 0.0)
        actual_robot_pos = (self.node.mover.robot_pos.x,
                            self.node.mover.robot_pos.y,
                            self.node.mover.robot_pos.theta)
        self.assertEqual(expected_robot_pos, actual_robot_pos)

    def test_goal_callback(self):
        goal_msg = PoseStamped()
        goal_msg.pose.position = Point(x=3.0, y=4.0, z=0.0)
        goal_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=1.0)
        self.node.map = MagicMock()
        self.node.mover.robot_pos = MagicMock()

        with patch('navi_main.global_planner_main.GlobalPlannerMain.plan_path') as mock_plan_path:
            mock_plan_path.return_value = True
            self.node.goal_callback(goal_msg)

        self.assertIsNotNone(self.node.goal)
        self.assertIsInstance(self.node.goal, GlobalPlannerNode)
        mock_plan_path.assert_called()

    def test_smooth_path_bspline_not_enough_points(self):
        patch_list = [(1.0, 2.0), (3.0, 4.0)]
        
        smoothed = self.node.smooth_path_bspline(patch_list)

        self.assertEqual(patch_list, smoothed)

    def test_smooth_path_bspline_success(self):
        path_tuple = [(1.0, 1.0), (1.0, 2.0), (2.0, 3.0), (3.0, 3.0)]
        path_list = [GlobalPlannerNode(x, y) for x, y in path_tuple]
        expected_points_count = 30

        result = self.node.smooth_path_bspline(path_list)

        self.assertIsInstance(result, list)
        self.assertTrue(all(isinstance(node, GlobalPlannerNode) for node in result))
        self.assertEqual(expected_points_count, len(result))
        self.plot_path_comparison(path_list, result)
    
    def plot_path_comparison(self, original_path, smoothed_path):
        # Extract x and y coordinates from the original and smoothed paths
        original_x = [node.x for node in original_path]
        original_y = [node.y for node in original_path]
        smoothed_x = [node.x for node in smoothed_path]
        smoothed_y = [node.y for node in smoothed_path]

        # Plot the original path
        plt.plot(original_x, original_y, label='Original Path', marker='o', linestyle='dashed')

        # Plot the smoothed path
        plt.plot(smoothed_x, smoothed_y, label='Smoothed Path', marker='x', linestyle='solid')

        # Add labels and legend
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.title('Comparison between Original and Smoothed Paths')
        plt.legend()

        # Show the plot
        plt.grid(True)
        plt.show()

    def test_convert_to_path(self):
        path_tuple = [(1.0, 2.0), (3.0, 4.0), (5.0, 6.0), (5.0, 7.0)]
        path_list = [GlobalPlannerNode(x, y) for x, y in path_tuple]

        path_msg = self.node.convert_to_path(path_list)

        self.assertIsInstance(path_msg, Path)
        self.assertEqual(len(path_msg.poses), len(path_list))
        for i, pose_stamped in enumerate(path_msg.poses):
            self.assertIsInstance(pose_stamped, PoseStamped)
            self.assertEqual(pose_stamped.pose.position.x, path_list[i].x)
            self.assertEqual(pose_stamped.pose.position.y, path_list[i].y)

    def tearDown(self) -> None:
        self.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    unittest.main()

    