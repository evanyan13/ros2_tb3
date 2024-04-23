import requests 
import json 
import rclpy 
from nav_msgs.msg import Odometry
from rclpy.node import Node 
from std_msgs.msg import String 
from geometry_msgs.msg import Twist
import numpy as np
import math
import cmath 
import time 

IP_ADDRESS = '192.168.43.24'

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

class Rotate(Node):
    def __init__(self):
        super().__init__('auto_nav')
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.odom_subscription
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.rotatechange = 0.3

    def odom_callback(self, msg):
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)

    def rotatebot(self, rot_angle):
        print(f"Rotating bot {rot_angle}")
        rclpy.spin_once(self)
        twist = Twist()
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current yaw angle: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired yaw angle: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * self.rotatechange
        # start rotation
        self.publisher_.publish(twist)
        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)

class PostRequest(Node): 
    def __init__(self): 
        super().__init__("post_request") 
        self.subscribe_send_state = self.create_subscription(String,'enter_state',self.check_call,10) 
        self.publish_cmd_vel = self.create_publisher(Twist,'cmd_vel',10) 
        self.post_call_state = '' 
        self.trigger = 0
        self.duration = 5
        print("ESP started")

    def check_call(self,msg):
        string = String() 
        if msg.data == 'checkpoint reached': 
            self.post_call() 
            if self.post_call_state == 'door1': 
                self.turn_left() 
            elif self.post_call_state == 'door2': 
                self.turn_right()    
            else: 
                print("bad msg rcvd") 
        else: 
            print("checkpoint not reached yet")  

    def post_call(self): 
        url = 'http://' + IP_ADDRESS + '/openDoor' 
        payload = { 
                "action": "openDoor", "parameters":{ 
                    "robotId": "41"  
                    }  
                } 
        headers = {'Content-Type': 'application/json'} 
        while True: 
            string_msg = String() 
            response = requests.post(url, json=payload, headers=headers) 
            if response.status_code == 200: 
                resp_data = response.json() 
                print(resp_data['data']['message']) 
                string_msg = resp_data['data']['message'] 
                self.post_call_state = string_msg  
                break 
            elif response.status_code == 400: 
                resp_data = response.json() 
                print("Error:",resp_data['data']['message']) 
                self.post_call_state = string_msg 
                break 
            elif response.status_code == 400: 
                resp_data = response.json() 
                print("Error:",resp_data['data']['message']) 
                self.post_call_state = string_msg 
                break 

    def turn_left(self): 
        twist = Twist() 
        string = String() 
        # turn 90 degress
        twist.angular.z = 0.3 
        twist.linear.x = 0.0
        self.publish_cmd_vel.publish(twist) 
        self.get_logger().info(f"Twist: {twist.linear.x} & {twist.angular.z}")
        time.sleep(5.5)
        twist.angular.z = 0.0 
        twist.linear.x = 0.0 
        self.publish_cmd_vel.publish(twist) 
        self.get_logger().info(f"Twist: {twist.linear.x} & {twist.angular.z}")
        #Rotate().rotatebot(90)
        # move forward
        #twist.angular.z = 0.0 
        #twist.linear.x = 0.05
        #self.publish_cmd_vel.publish(twist) 
        #self.get_logger().info(f"Twist: {twist.linear.x} & {twist.angular.z}")
        self.trigger = 1
        #raise KeyboardInterrupt  

    def turn_right(self): 
        twist = Twist() 
        string = String() 
        # turn 90 degress
        twist.angular.z = 0.3*-1
        twist.linear.x = 0.0 
        self.publish_cmd_vel.publish(twist) 
        self.get_logger().info(f"Twist: {twist.linear.x} & {twist.angular.z}")
        time.sleep(5.5)
        twist.angular.z = 0.0 
        twist.linear.x = 0.0 
        self.publish_cmd_vel.publish(twist) 
        self.get_logger().info(f"Twist: {twist.linear.x} & {twist.angular.z}")
        #Rotate().rotatebot(270) 
        # move forward
        #twist.angular.z = 0.0 
        #twist.linear.x = 0.05 
        #self.publish_cmd_vel.publish(twist)
        #self.get_logger().info(f"Twist: {twist.linear.x} & {twist.angular.z}") 
        self.trigger = 1
        #raise KeyboardInterrupt

    def stop_movement(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publish_cmd_vel.publish(twist)
        self.get_logger().info(f"Twist: {twist.linear.x} & {twist.angular.z}")
        print("Stopped all movement")

def main(args=None): 
    rclpy.init(args=args) 
    post_request = PostRequest() 
    try: 
        rclpy.spin(post_request) 
    except:
        post_request.stop_movement()
        post_request.destroy_node() 
        rclpy.shutdown()

if __name__ == "__main__":
    main()
