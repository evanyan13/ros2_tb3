import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class MinDistanceSubscriber(Node):
    def __init__(self):
        super().__init__('min_distance_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            'min_distance',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        min_distance = msg.data
        self.get_logger().info('Received shortest distance: %.2f meters' % min_distance)

def main(args=None):
    rclpy.init(args=args)
    min_distance_subscriber = MinDistanceSubscriber()
    rclpy.spin(min_distance_subscriber)
    min_distance_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
