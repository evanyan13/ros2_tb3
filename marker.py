import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

line_follow_count = 0

class Marker(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.setup()
        self.count_red = 0
        self.count_white = 0
        self.colour = ""
        self.sensor_left = 0
        self.sensor_right = 0
        self.publisher_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_enter_state = self.create_publisher(String,'enter_state',10)
        self.subcribe_check_2 = self.create_subscription(String,'check_2',self.check_red_white(),10)
        self.x = 0.01
        self.check_red_white()

    def setup(self):
        GPIO.setmode(GPIO.BCM)
        self.IR_left = 3 
        self.IR_right = 27
        GPIO.setup(self.IR_left,GPIO.IN)
        GPIO.setup(self.IR_right,GPIO.IN)
        self.s2 = 22
        self.s3 = 23
        self.out = 24
        GPIO.setup(self.s2,GPIO.OUT)
        GPIO.setup(self.s3,GPIO.OUT)
        GPIO.setup(self.out,GPIO.IN)

    def color_read_value(self, a0, a1):
        GPIO.output(self.s2, a0) 
        GPIO.output(self.s3, a1) 
        # Give the sensor some time to adjust
        time.sleep(0.1) 
        # Wait for a full cycle (this will make sure we only count_red full cycles)
        GPIO.wait_for_edge(self.out, GPIO.FALLING)
        # Measuring the time it takes for the output square wave to change from max value to min value
        # Basically measuring how long one half of the period of the square wave is 
        # because according to the spec, the output square wave is of 50% duty cycle with frequency directly proportional to light intensity
        GPIO.wait_for_edge(self.out, GPIO.RISING)
        start = time.time() 
        GPIO.wait_for_edge(self.out, GPIO.FALLING) 
        half_period = time.time() - start
        return half_period * 1000000 

    def color_detect(self):
        r = self.color_read_value(GPIO.LOW, GPIO.LOW)
        time.sleep(0.1) 
        g = self.color_read_value(GPIO.HIGH, GPIO.HIGH) 
        time.sleep(0.1) 
        b = self.color_read_value(GPIO.LOW, GPIO.HIGH)
        time.sleep(0.1) 
        print(f"Red:{r}, Green:{g}, Blue:{b}")
        # Larger the half period, larger the period, lower the frequency, lower the intensity for the particular colour.
        # Smaller the half period, smaller the period, higher the frequency, higher the intensity for the particular colour.
        if r < 500 and g < 400 and b < 400:
            return "WHITE"
        if (r < g) and (r < b):
            return  "RED"
        if (b < g) and (b < r): 
            return  "BLUE"
        if (g < b) and (g < r): 
            return  "GREEN"

    def check_red_white(self,msg):
        if msg.data == 'checkpoint 2':
            self.check_2 = 'reached'
        twist = Twist()
        string = String()
        self.colour = self.color_detect()
        self.sensor_left = GPIO.input(self.IR_left)
        self.sensor_right = GPIO.input(self.IR_right)
        print(self.colour)
        if self.colour == "RED" and (not self.sensor_left and not self.sensor_right):
            self.count_red += 1
            twist.linear.x = self.x
            twist.angular.z = 0.0
            self.publisher_cmd_vel.publish(twist)
            if self.count_red == 3:
                print(f"{self.count_red} reds in a row and not line following, starting line following")
                self.start_line_following()
                self.count_red = 0
                self.count_white = 0
        elif self.colour == 'WHITE':
            self.count_white += 1
            if self.count_white == 3:
                if self.check_2 == 'reached':
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.publisher_cmd_vel.publish(twist)
                    string.data = 'start launch sequence'
                    self.publisher_enter_state.publish(string)
                    self.count_red = 0
                    self.count_white = 0
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.publisher_cmd_vel.publish(twist)
                    string.data = 'checkpoint reached'
                    self.publisher_enter_state.publish(string)
                    self.count_red = 0
                    self.count_white = 0
        else:
            self.count_red = 0
            self.count_white = 0
        print("red ",self.count_red)
        print("white ", self.count_white)
        print("//////////////////////////////////////////////////")

    def start_line_following(self):
        twist = Twist()
        while True:
            self.sensor_left = GPIO.input(self.IR_left)
            self.sensor_right = GPIO.input(self.IR_right)
            if self.sensor_left or self.sensor_right:
                break
            else: 
                twist.linear.x = 2 * self.x
                twist.angular.z = 0.0
                self.publisher_cmd_vel.publish(twist)
                print("Searching for black line")
        while self.color_detect() != "WHITE":
            self.sensor_left = GPIO.input(self.IR_left)
            self.sensor_right = GPIO.input(self.IR_right)
            if  self.sensor_left and self.sensor_right:
                twist.linear.x = self.x * 2.5
                twist.angular.z = 0.0
                print("Both in black, moving forward")
                self.publisher_cmd_vel.publish(twist)
            elif self.sensor_left and not self.sensor_right:
                twist.angular.z = 0.2
                twist.linear.x = 0.0
                print("Left in black, turning left")
                self.publisher_cmd_vel.publish(twist)
            elif self.sensor_right and not self.sensor_left:
                twist.angular.z = -0.2
                twist.linear.x = 0.0
                print("Right in black, turning right")
                self.publisher_cmd_vel.publish(twist)
            else:
                print("Both not in black")
                # lol
        #raise KeyboardInterrupt

    def stop_line_following(self):
        line_follow_count =
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_cmd_vel.publish(twist)
        self.get_logger().info(f"Twist: {twist.linear.x} & {twist.angular.z}")
        print("Stop line following")

def main(args=None):
    rclpy.init(args=args)
    marker = Marker()
    try:
        rclpy.spin(marker)
    except KeyboardInterrupt:
        marker.stop_line_following()
        #marker.destroy_node()
        #rclpy.shutdown

if __name__ == '__main__':
    main() 
