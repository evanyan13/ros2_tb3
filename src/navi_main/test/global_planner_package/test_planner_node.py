import rclpy
import math
import unittest
from geometry_msgs.msg import Pose, PoseStamped, Quaternion

from navi_main.global_planner_package.global_planner_node import GlobalPlannerNode
from navi_main.global_planner_package.utils import euler_from_quaternion

class TestGlobalPlannerNode(unittest.TestCase):
    def setUp(self) -> None:
        rclpy.init()
        self.node = rclpy.create_node('test_node')

    def test_calculate_distance(self):
        node_a = GlobalPlannerNode(0, 0)
        node_b = GlobalPlannerNode(3, 4)
        self.assertEqual(node_a.calculate_distance(node_b), 5)

    def test_generate_neighbours(self):
        map_resolution = 1
        start_node = GlobalPlannerNode(1, 2)
        generated_neighbours = start_node.generate_neighbours(map_resolution)
        generated_neighbours_pos = [(n.x, n.y) for n in generated_neighbours]
        expected_neighbours = [(1, 3), (2, 3), (2, 2), (2, 1),
                               (1, 1), (0, 1), (0, 2), (0, 3)]
        print(f"Generated Neighbours: {generated_neighbours_pos}")
        self.assertEqual(set(generated_neighbours_pos), set(expected_neighbours))

    def test_backtrack_path(self):
        start_node = GlobalPlannerNode(0, 0)
        n1 = GlobalPlannerNode(1, 1, parent=start_node)
        n2 = GlobalPlannerNode(1, 2, parent=n1)
        n3 = GlobalPlannerNode(2, 1, parent=n2)
        end_node = GlobalPlannerNode(3, 2, parent=n3)

        generated_path = end_node.backtrack_path()
        expected_path = [start_node, n1, n2, n3, end_node]
        print(f"Generated Path: {[(n.x, n.y) for n in generated_path]}")
        self.assertEqual(generated_path, expected_path)

    def test_equals(self):
        node_a = GlobalPlannerNode(1, 1)
        node_b = GlobalPlannerNode(2, 3)
        node_c = GlobalPlannerNode(1, 1, parent=node_b)

        self.assertFalse(node_a.equals(node_b))
        self.assertTrue(node_c.equals(node_a))
        self.assertTrue(node_a.equals(node_c))

    def test_from_pose(self):
        pose = Pose()
        pose.position.x, pose.position.y = 1.0, 2.0
        pose.orientation = Quaternion(x=0.0, y=0.0, z=1.0, w=0.0)

        node = GlobalPlannerNode.from_pose(pose)

        self.assertEqual(node.x, 1.0)
        self.assertEqual(node.y, 2.0)
        self.assertAlmostEqual(node.theta, math.pi, places=5)

    def test_from_tf(self):
        test_position = [1.0, 2.0, 0.0]
        test_quaternion = [0.0, 0.0, 0.0, 1.0]

        test_node = GlobalPlannerNode.from_tf(test_position, test_quaternion)

        self.assertIsInstance(test_node, GlobalPlannerNode)
        self.assertEqual(test_node.x, test_position[0])
        self.assertEqual(test_node.y, test_position[1])

        _, _, expected_yaw = euler_from_quaternion(*test_quaternion)
        self.assertEqual(test_node.theta, expected_yaw)

    def test_from_pose_stamped(self):
        example_node = GlobalPlannerNode(2.0, 3.0)
        current_time = self.node.get_clock().now().to_msg()

        pose_stamped =example_node.to_pose_stamped(current_time)
        
        self.assertIsInstance(pose_stamped, PoseStamped)
        self.assertEqual(pose_stamped.header.stamp, current_time)

    def tearDown(self) -> None:
        self.node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    unittest.main()