import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np

class Scanner(Node):
    def __init__(self):
        super().__init__('scanner')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_profile_sensor_data)
        self.publisher_ = self.create_publisher(Float32, 'min_distance', 10)

    def listener_callback(self, msg):
        laser_range = np.array(msg.ranges)
        laser_range[laser_range == 0] = np.nan
        if np.isnan(laser_range).all():
            min_range = np.nan
        else:
            min_range = np.nanmin(laser_range)
        self.get_logger().info('Shortest distance: %.2f meters' % min_range)
        min_distance_msg = Float32()
        min_distance_msg.data = min_range
        self.publisher_.publish(min_distance_msg)

def main(args=None):
    rclpy.init(args=args)
    scanner = Scanner()
    rclpy.spin(scanner)
    scanner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
