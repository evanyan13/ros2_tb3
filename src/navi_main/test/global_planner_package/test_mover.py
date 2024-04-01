import rclpy
import random
import unittest
from math import pi
from unittest.mock import MagicMock, Mock
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Quaternion, Twist

from navi_main.global_planner_main import GlobalPlannerMain
from navi_main.global_planner_package.global_mover import GlobalMover
from navi_main.global_planner_package.global_planner_node import GlobalPlannerNode


class TestGlobalMover(unittest.TestCase):
    def setUp(self) -> None:
        rclpy.init()
        self.planner = GlobalPlannerMain()
        self.mover = GlobalMover(self.planner)
        self.mover.robot_pos = GlobalPlannerNode()
        self.mover.follow_path = MagicMock()
        self.mover.velocity_publisher = MagicMock()
    
    def test_path_callback(self):
        path_msg = Path()
        for _ in range(5):
            pose_stamped = PoseStamped()
            pose_stamped.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            path_msg.poses.append(pose_stamped)

        self.mover.path_callback(path_msg)

        self.mover.follow_path.assert_called_once()
        traversed_path = self.mover.follow_path.call_args[0][0]
        self.assertEqual(len(traversed_path), len(path_msg.poses))

    def test_scan_callback(self):

        def mock_scan_data(ranges):
            scan = LaserScan()
            scan.ranges = ranges
            # print(scan, end="\n")
            return scan

        scan = mock_scan_data([0.2] * 10 + [float('inf')] * 340)
        self.mover.scan_callback(scan)
        self.assertTrue(self.mover.is_obstacle_ahead)

    def test_stop_moving(self):
        self.mover.is_moving = True
        self.mover.stop_moving()

        self.assertFalse(self.mover.is_moving)

        # Check if a zero Twist was published to stop the bot
        self.mover.velocity_publisher.publish.assert_called_once()
        published_msg = self.mover.velocity_publisher.publish.call_args[0][0]
        self.assertIsInstance(published_msg, Twist)
        self.assertEqual(published_msg.linear.x, 0)
        self.assertEqual(published_msg.angular.z, 0)

    def test_handle_navi_fail(self):
        self.mover.handle_navi_fail()
        self.assertFalse(self.mover.is_moving)
        self.assertEqual(self.planner.state, 'IDLE')
        print(f"[CORRECT] Navi fail switch to idle: {self.planner.state}")

    def test_complete_navi(self):
        self.planner.start_planning()
        self.planner.path_found()

        self.mover.complete_navi()
        self.assertFalse(self.mover.is_moving)
        self.assertEqual(self.planner.state, 'IDLE')
        print(f"[CORRECT] Follow path complete switch to idle: {self.planner.state}")

    def test_move_to_point(self):
        # Function to mimic the bot moving closer to the point
        def mock_calculate_distance(*args, **kwargs):
            nonlocal initial_distance
            if initial_distance > 0:
                deduct = random.random()
                initial_distance -= deduct
            return initial_distance

        # Initialisation of parameters
        point = GlobalPlannerNode(1.0, 1.0)
        initial_distance = 10.0
        self.mover.robot_pos.calculate_distance = Mock(side_effect=mock_calculate_distance)
        self.mover.create_rate = Mock()
        self.mover.is_obstacle_ahead = False
        
        self.mover.move_to_point(point)

        # Check that the velocity publisher has been called
        assert self.mover.velocity_publisher.publish.call_count > 0
    
    def test_rotate_to_goal(self):
        # Function to mimic decreasing angular difference
        rotate_tolerance = 0.004
        mock_angular_differences = [0.5, 0.3, 0.1, rotate_tolerance - 0.001]
        mock_angular_differences_call = iter(mock_angular_differences)

        def get_mock_angular_difference(*args, **kwargs):
            try:
                return next(mock_angular_differences_call)
            except StopIteration:
                return mock_angular_differences[-1]
        
        # Initialising parameters
        goal = GlobalPlannerNode()
        self.mover.angular_difference = Mock(side_effect=get_mock_angular_difference)
        self.mover.create_rate = Mock()
        self.mover.velocity_publisher.publish = Mock()

        self.mover.rotate_to_goal(goal)

        self.assertFalse(self.mover.is_moving)
        self.assertTrue(self.mover.velocity_publisher.publish.called)

    def test_angular_difference(self):
        mover = GlobalMover(None)
        mover.robot_pos = GlobalPlannerNode(0.0, 0.0)

        # In Front
        target = GlobalPlannerNode(x=0, y=1)
        expected_angle = pi / 2
        actual_angle = mover.angular_difference(target)
        print(f"angular_difference: {actual_angle}")
        self.assertAlmostEqual(actual_angle, expected_angle, places=5)

        # On the right
        target = GlobalPlannerNode(x=1, y=0)
        expected_angle = 0
        actual_angle = mover.angular_difference(target)
        print(f"angular_difference: {actual_angle}")
        self.assertAlmostEqual(actual_angle, expected_angle, places=5)

        # Behind
        target = GlobalPlannerNode(x=-1, y=0)
        expected_angle = pi
        actual_angle = mover.angular_difference(target)
        print(f"angular_difference: {actual_angle}")
        self.assertAlmostEqual(actual_angle, expected_angle, places=5)

    def tearDown(self) -> None:
        self.mover.destroy_node()
        self.planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    unittest.main()