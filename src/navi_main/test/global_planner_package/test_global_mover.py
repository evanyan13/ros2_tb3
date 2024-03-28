import unittest
from unittest.mock import Mock
from sensor_msgs.msg import LaserScan
from navi_main.global_planner_package.global_mover import GlobalMover
from navi_main.global_planner_package.global_planner_node import GlobalPlannerNode
import rclpy

class TestGlobalMover(unittest.TestCase):
    def setUp(self) -> None:
        rclpy.init()
        self.planner = Mock()
        self.global_mover = GlobalMover(self.planner)

    def mock_scan_date(self, ranges):
        scan = LaserScan()
        scan.ranges = ranges
        print(scan)
        return scan
    
    def test_obstacle_detection(self):
        scan = self.mock_scan_date([0.2] * 10 + [float('inf')] * 340)
        self.global_mover.scan_callback(scan)
        test_obstacle = self.global_mover.is_obstacle_ahead
        print(f"is_obstacle_ahead: {test_obstacle}")
        self.assertTrue(test_obstacle)

    def test_move_to_point(self):
        self.global_mover.robot_pos = GlobalPlannerNode(0, 0)
        target_point = GlobalPlannerNode(1, 1)
        self.global_mover.move_to_point(target_point)
    
    def tearDown(self) -> None:
        self.global_mover.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    unittest.main()