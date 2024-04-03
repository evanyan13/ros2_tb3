import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.publisher_cmd_vel = self.create_publisher(Twist,'cmd_vel',10)
        self.subscriber_color = self.create_subscription(String, 'color_info', self.color_callback, 10)
        self.setup()
        self.line_following = False
        self.sensor_left = 0
        self.sensor_right = 0
        
    def setup(self):
        GPIO.setmode(GPIO.BCM)
        self.IR_left = 3 #Rpi pin 5
        self.IR_right = 27 #Rpi pin 13
        GPIO.setup(self.IR_left,GPIO.IN)
        GPIO.setup(self.IR_right,GPIO.IN)
    
    def color_callback(self,msg):
        if msg.data == 'RED' and not self.line_following:
            self.start_line_following()
            
    def start_line_following(self):
        self.line_following = True
        twist = Twist()
        while self.line_following:
            self.sensor_left = GPIO.input(self.IR_left)
            self.sensor_right = GPIO.input(self.IR_right)
            if self.sensor_left and self.sensor_right:
                twist.linear.x = 0.5
            elif self.sensor_left and not self.sensor_right:
                twist.angular.z += 0.2
            elif self.sensor_right and not self.sensor_left:
                twist.angular.z -= 0.2
            else:
                self.check_end_line()
            self.publisher_cmd_vel.publish(twist)
    
    def check_end_line(self):
        if self.sensor_left == 0 and self.sensor_right == 0:
            self.stop_line_follower()
    
    def stop_line_follower(self):
        self.line_following = False
        twist_stop = Twist()
        twist_stop.linear.x = 0
        self.publisher_cmd_vel.publish(twist_stop)
        GPIO.cleanup()
        
def main(args=None):
    rclpy.init(args=args)
    line_follower = LineFollower()
    try:
        rclpy.spin(line_follower)
    except KeyboardInterrupt:
        line_follower.stop_line_follower()
    
    line_follower.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()    
                
     