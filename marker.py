import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

class Marker(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.setup()
        self.count_red = 0
        self.count_white = 0
        self.count_notwhite = 0
        self.colour = ""
        self.sensor_left = 0
        self.sensor_right = 0
        self.publisher_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_enter_state = self.create_publisher(String, 'enter_state', 10)
        self.x = 0.02
        self.z = 0.15
        self.check = 0

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
        # time.sleep(0.1) 
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
        b = self.color_read_value(GPIO.LOW, GPIO.HIGH) 
        print(f"Red:{r}, Green:{g}, Blue:{b}")
        # Larger the half period, larger the period, lower the frequency, lower the intensity for the particular colour.
        # Smaller the half period, smaller the period, higher the frequency, higher the intensity for the particular colour.
        if r > 900 and g > 900 and b > 700:
            return "GROUND"
        if r < 500 and g < 500 and b < 350:
            return "WHITE"
        if (r < g) and (r < b):
            return  "RED"
        if (b < g) and (b < r): 
            return  "BLUE"
        if (g < b) and (g < r): 
            return  "GREEN"

    def check_red_white(self):
        twist = Twist()
        string = String()
        #string.data = 'Not Yet'
        #self.publisher_enter_state.publish(string)
        self.colour = self.color_detect()
        self.sensor_left = GPIO.input(self.IR_left)
        self.sensor_right = GPIO.input(self.IR_right)
        print(self.colour)
        print(f"Left sensor: {self.sensor_left}")
        print(f"Right sensor: {self.sensor_right}")
        if self.colour == "RED" and not self.sensor_left and not self.sensor_right:
            self.count_red += 1
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_cmd_vel.publish(twist)
            self.get_logger().info(f"Twist: {twist.linear.x} & {twist.angular.z}")
            if self.count_red == 10 and self.check == 0:
                #string.data = 'Stop Nav'
                #print("Autonomous navigation stopped")
                #self.publisher_enter_state.publish(string)
                print(f"{self.count_red} reds in a row, starting line following")
                self.count_red = 0
                self.count_white = 0
                self.start_line_following()
            elif self.count_red == 10 and self.check == 1:
                self.count_red = 0
                self.count_white = 0
                self.start_line_following_slow()
        elif self.colour == 'WHITE':
            self.count_white += 1
            if self.count_white == 5 and self.check == 0:
                print("1.5 second delay")
                time.sleep(1.5)
                #self.sensor_left = GPIO.input(self.IR_left)
                #self.sensor_right = GPIO.input(self.IR_right)
                #print(f"Left sensor: {self.sensor_left}")
                #print(f"Right sensor: {self.sensor_right}")
                self.colour = self.color_detect()
                print(self.colour)
                #while self.sensor_left == 0 or self.sensor_right == 0:
                while self.colour == 'WHITE':
                    twist.linear.x = self.x
                    twist.angular.z = 0.0
                    self.publisher_cmd_vel.publish(twist)
                    self.get_logger().info(f"Twist: {twist.linear.x} & {twist.angular.z}")
                    self.colour = self.color_detect()
                    print(self.colour)
                    #self.sensor_left = GPIO.input(self.IR_left)
                    #self.sensor_right = GPIO.input(self.IR_right)
                    #print(f"Left sensor: {self.sensor_left}")
                    #print(f"Right sensor: {self.sensor_right}")
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publisher_cmd_vel.publish(twist)
                self.get_logger().info(f"Twist: {twist.linear.x} & {twist.angular.z}")
                string.data = 'checkpoint reached'
                print("//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////")
                print("First checkpoint reached")
                print("//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////")
                self.publisher_enter_state.publish(string)
                self.count_red = 0
                self.count_white = 0
                print("10 seconds countdown for robot to rotate")
                time.sleep(10)
                twist.angular.z = 0.0 
                twist.linear.x = self.x
                self.publisher_cmd_vel.publish(twist) 
                self.get_logger().info(f"Twist: {twist.linear.x} & {twist.angular.z}")
                while True:
                    self.colour = self.color_detect()
                    print(self.colour)
                    if self.colour != "WHITE":
                        self.count_notwhite += 1
                        print(self.count_notwhite)
                    else:
                        self.count_notwhite = 0
                    if self.count_notwhite == 5:
                        break
                print("Exited white")
                print("/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////")
                print("Second marker sequence initiated")
                print("/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////")
                self.check = 1
            elif self.count_white == 1 and self.check == 1:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publisher_cmd_vel.publish(twist)
                self.get_logger().info(f"Twist: {twist.linear.x} & {twist.angular.z}")
                string.data = 'start launch sequence'
                self.publisher_enter_state.publish(string)
                print(string.data)
                self.count_red = 0
                self.count_white = 0
                raise KeyboardInterrupt
        else:
            self.count_red = 0
            self.count_white = 0
        print("Red count: ",self.count_red)
        print("White count: ", self.count_white)
        print("////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////")

    def start_line_following(self):
        twist = Twist()
        while True:
            self.sensor_left = GPIO.input(self.IR_left)
            self.sensor_right = GPIO.input(self.IR_right)
            if self.sensor_left or self.sensor_right:
                break
            else: 
                twist.linear.x = self.x
                twist.angular.z = 0.0
                self.publisher_cmd_vel.publish(twist)
                self.get_logger().info(f"Twist: {twist.linear.x} & {twist.angular.z}")
                print("Searching for black line")
        while self.colour != "WHITE":
            self.colour = self.color_detect()
            self.sensor_left = GPIO.input(self.IR_left)
            self.sensor_right = GPIO.input(self.IR_right)
            if  self.sensor_left and self.sensor_right:
                twist.linear.x = self.x * 2.5
                twist.angular.z = 0.0
                print("Both in black, moving forward")
                self.publisher_cmd_vel.publish(twist)
                self.get_logger().info(f"Twist: {twist.linear.x} & {twist.angular.z}")
            elif self.sensor_left and not self.sensor_right:
                twist.angular.z = self.z
                twist.linear.x = 0.0
                print("Left in black, turning left")
                self.publisher_cmd_vel.publish(twist)
                self.get_logger().info(f"Twist: {twist.linear.x} & {twist.angular.z}")
            elif self.sensor_right and not self.sensor_left:
                twist.angular.z = self.z * -1
                twist.linear.x = 0.0
                print("Right in black, turning right")
                self.publisher_cmd_vel.publish(twist)
                self.get_logger().info(f"Twist: {twist.linear.x} & {twist.angular.z}")
            else:
                print("Both not in black")

    def start_line_following_slow(self):
        twist = Twist()
        while True:
            self.sensor_left = GPIO.input(self.IR_left)
            self.sensor_right = GPIO.input(self.IR_right)
            if self.sensor_left or self.sensor_right:
                break
            else: 
                twist.linear.x = self.x
                twist.angular.z = 0.0
                self.publisher_cmd_vel.publish(twist)
                self.get_logger().info(f"Twist: {twist.linear.x} & {twist.angular.z}")
                print("Searching for black line")
        while self.colour != "WHITE":
            self.colour = self.color_detect()
            self.sensor_left = GPIO.input(self.IR_left)
            self.sensor_right = GPIO.input(self.IR_right)
            if  self.sensor_left and self.sensor_right:
                twist.linear.x = self.x
                twist.angular.z = 0.0
                print("Both in black, moving forward")
                self.publisher_cmd_vel.publish(twist)
                self.get_logger().info(f"Twist: {twist.linear.x} & {twist.angular.z}")
            elif self.sensor_left and not self.sensor_right:
                twist.angular.z = self.z
                twist.linear.x = 0.0
                print("Left in black, turning left")
                self.publisher_cmd_vel.publish(twist)
                self.get_logger().info(f"Twist: {twist.linear.x} & {twist.angular.z}")
            elif self.sensor_right and not self.sensor_left:
                twist.angular.z = self.z * -1
                twist.linear.x = 0.0
                print("Right in black, turning right")
                self.publisher_cmd_vel.publish(twist)
                self.get_logger().info(f"Twist: {twist.linear.x} & {twist.angular.z}")
            else:
                print("Both not in black")


    def stop_line_following(self):
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
        while True:
            marker.check_red_white()
    except:
        marker.stop_line_following()
        marker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
ubuntu@ubuntu
