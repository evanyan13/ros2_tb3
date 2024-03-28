import unittest
from unittest.mock import MagicMock
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Quaternion
from navi_main.global_planner_package.global_mover import GlobalMover
from navi_main.global_planner_package.global_planner_node import GlobalPlannerNode
import rclpy

class TestGlobalMover(unittest.TestCase):
    def setUp(self) -> None:
        rclpy.init()
        self.planner = MagicMock()
        self.global_mover = GlobalMover(self.planner)
        self.global_mover.robot_pos = MagicMock()
        self.global_mover.follow_path = MagicMock()

    def mock_scan_date(self, ranges):
        scan = LaserScan()
        scan.ranges = ranges
        print(scan, end="\n")
        return scan
    
    def test_obstacle_detection(self):
        scan = self.mock_scan_date([0.2] * 10 + [float('inf')] * 340)
        self.global_mover.scan_callback(scan)
        test_obstacle = self.global_mover.is_obstacle_ahead
        print(f"is_obstacle_ahead: {test_obstacle}")
        self.assertTrue(test_obstacle)

    def test_move_callback(self):
        path_msg = Path()
        for _ in range(5):
            pose_stamped = PoseStamped()
            pose_stamped.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            path_msg.poses.append(pose_stamped)

        self.global_mover.move_callback(path_msg)

        self.global_mover.follow_path.assert_called_once()
        traversed_path = self.global_mover.follow_path.call_args[0][0]
        print(f"traversed_path: {traversed_path} \n")
        self.assertEqual(len(traversed_path), len(path_msg.poses))
    
    def tearDown(self) -> None:
        self.global_mover.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    unittest.main()