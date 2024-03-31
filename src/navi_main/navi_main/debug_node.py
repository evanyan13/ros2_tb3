#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class TF2DebugNode(Node):
    def __init__(self):
        super().__init__('tf2_debug_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def lookup_transform(self):
        while rclpy.ok():
            try:
                trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                self.get_logger().info(f"Transform: {trans}")
                break
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().error(f"Error occurred: {e}")
                time.sleep(1)

def main():
    rclpy.init()
    node = TF2DebugNode()
    node.lookup_transform()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
