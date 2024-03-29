import rclpy
import random
import unittest
from math import pi
from unittest.mock import MagicMock, Mock
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from navi_main.global_planner_package.global_mover import GlobalMover
from navi_main.global_planner_package.global_planner_node import GlobalPlannerNode


class TestGlobalMover(unittest.TestCase):
    def setUp(self) -> None:
        rclpy.init()
        self.planner = MagicMock()
        self.global_mover = GlobalMover(self.planner)
        self.global_mover.robot_pos = GlobalPlannerNode()
        self.global_mover.follow_path = MagicMock()
        self.global_mover.velocity_publisher = MagicMock()
    
    def test_move_callback(self):
        path_msg = Path()
        for _ in range(5):
            pose_stamped = PoseStamped()
            pose_stamped.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            path_msg.poses.append(pose_stamped)

        self.global_mover.move_callback(path_msg)

        self.global_mover.follow_path.assert_called_once()
        traversed_path = self.global_mover.follow_path.call_args[0][0]
        self.assertEqual(len(traversed_path), len(path_msg.poses))

    def test_scan_callback(self):

        def mock_scan_data(ranges):
            scan = LaserScan()
            scan.ranges = ranges
            # print(scan, end="\n")
            return scan

        scan = mock_scan_data([0.2] * 10 + [float('inf')] * 340)
        self.global_mover.scan_callback(scan)
        self.assertTrue(self.global_mover.is_obstacle_ahead)

    def test_stop_moving(self):
        self.global_mover.is_moving = True
        self.global_mover.stop_moving()

        self.assertFalse(self.global_mover.is_moving)

        # Check if a zero Twist was published to stop the bot
        self.global_mover.velocity_publisher.publish.assert_called_once()
        published_msg = self.global_mover.velocity_publisher.publish.call_args[0][0]
        self.assertIsInstance(published_msg, Twist)
        self.assertEqual(published_msg.linear.x, 0)
        self.assertEqual(published_msg.angular.z, 0)

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
        self.global_mover.robot_pos.calculate_distance = Mock(side_effect=mock_calculate_distance)
        self.global_mover.create_rate = Mock()
        self.global_mover.is_shutdown_initiated = False
        self.global_mover.is_obstacle_ahead = False
        
        self.global_mover.move_to_point(point)

        # Check that the velocity publisher has been called
        assert self.global_mover.velocity_publisher.publish.call_count > 0
    
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
        self.global_mover.angular_difference = Mock(side_effect=get_mock_angular_difference)
        self.global_mover.create_rate = Mock()
        self.global_mover.is_shutdown_initiated = False
        self.global_mover.velocity_publisher.publish = Mock()

        self.global_mover.rotate_to_goal(goal)

        self.assertFalse(self.global_mover.is_moving)
        self.assertTrue(self.global_mover.velocity_publisher.publish.called)

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
        self.global_mover.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    unittest.main()