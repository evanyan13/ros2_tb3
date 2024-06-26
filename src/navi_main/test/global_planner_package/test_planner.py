import rclpy
import unittest
import numpy as np
import matplotlib.pyplot as plt
from unittest.mock import MagicMock, patch
from geometry_msgs.msg import Point, Quaternion, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path

from navi_main.global_planner_package.global_planner import GlobalPlanner
from navi_main.global_planner_package.global_map import GlobalMap
from navi_main.global_planner_package.global_mover import GlobalMover
from navi_main.global_planner_package.global_node import GlobalPlannerNode

class TestGlobalPlanner(unittest.TestCase):
    def setUp(self) -> None:
        rclpy.init()
        self.node = GlobalPlanner()
    
    def test_map_callback(self):
        map_msg = OccupancyGrid()

        self.node.map_callback(map_msg)

        self.assertIsNotNone(self.node.map)
        self.assertIsNotNone(self.node.mover.robot_pos)
        self.assertIsInstance(self.node.map, GlobalMap)
        self.assertIsInstance(self.node.mover, GlobalMover)

    def test_goal_callback(self):
        goal_msg = PoseStamped()
        goal_msg.pose.position = Point(x=3.0, y=4.0, z=0.0)
        goal_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=1.0)
        self.node.map = MagicMock()
        self.node.mover.robot_pos = MagicMock()

        with patch.object(self.node, 'plan_path') as mock_plan_path:
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

    def test_state_machine_transition(self):
        self.assertEqual(self.node.state, 'IDLE')
        print(f"[CORRECT] Idle state at init: {self.node.state}")

        self.node.start_planning()
        self.assertEqual(self.node.state, 'PLANNING')
        print(f"[CORRECT] From idle to planning: {self.node.state}")

        self.node.path_found()
        self.assertEqual(self.node.state, 'NAVIGATING')
        print(f"[CORRECT] From planning to navigating: {self.node.state}")

        self.node.goal_reached()
        self.assertEqual(self.node.state, 'IDLE')
        print(f"[CORRECT] From navigating to idle: {self.node.state}")

        self.node.start_planning()
        self.assertEqual(self.node.state, 'PLANNING')
        self.node.fail()
        self.assertEqual(self.node.state, 'IDLE')
        print(f"[CORRECT] From any state to idle: {self.node.state}")      
    
    def test_state_machine_function_calls(self):
        self.assertEqual(self.node.state, 'IDLE')
        print(f"[CORRECT] Idle state at init: {self.node.state}")

        self.node.map = GlobalMap(OccupancyGrid())
        self.node.plan_path()
        self.assertEqual(self.node.state, 'IDLE')
        print(f"[CORRECT] Fail to plan_path: {self.node.state}")

        self.node.start = GlobalPlannerNode(0.0, 0.0)
        self.node.goal = GlobalPlannerNode(1.0, 2.0)
        with patch('navi_main.global_planner_package.global_planner.find_astar_path') as mock_astar_fail:
            mock_astar_fail.return_value = []
            self.node.plan_path()
        self.assertEqual(self.node.state, 'IDLE')
        print(f"[CORRECT] Fail to plan_path given no astar path: {self.node.state}")

        with patch('navi_main.global_planner_package.global_planner.find_astar_path') as mock_astar_success:
            mock_astar_success.return_value = [self.node.start, self.node.goal]
            self.node.plan_path()
        self.assertEqual(self.node.state, 'NAVIGATING')
        print(f"[CORRECT] Found astar path from planning to navigating: {self.node.state}")

    def tearDown(self) -> None:
        self.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    unittest.main()

    