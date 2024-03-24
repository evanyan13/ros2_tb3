import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

class MinimalLidarSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_lidar_subscriber')
        # Change the subscription topic and message type to match LIDAR datz
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning

    def scan_callback(self, msg):
        laser_range = np.array(msg.ranges)
        laser_range[laser_range == 0] = np.nan

        if np.isnan(laser_range).all():
            min_range = np.nan
        else:
            min_range = np.nanmin(laser_range)

        self.get_logger().info('Shortest distance: %.2f meters' % min_range)

def main(args=None):
    print("Running")
    rclpy.init(args=args)
    minimal_lidar_subscriber = MinimalLidarSubscriber()
    rclpy.spin(minimal_lidar_subscriber)
    minimal_lidar_subscriber.destroy_node()
    rclpy.shutdown()
    print("Process Ended")

if __name__ == '__main__':
    main()
