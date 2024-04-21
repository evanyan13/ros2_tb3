import requests
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

trigger = 0

class PostRequest(Node):
    def __init__(self):
        super().__init__("post_request")
        #self.publish_post_state = self.create_publisher(String,'post_state',10)
        self.subscribe_send_state = self.create_subscription(String,'enter_state',self.check_call,10)
        self.publish_cmd_vel = self.create_publisher(Twist,'cmd_vel',10)
        self.publish_check_2 = self.create_publisher(String,'chcek_2',10)
        self.post_call_state = ''
        self.check_call()
        
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
        url = 'http://' + '192.168.43.111' + '/openDoor'

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
                #self.publish_post_state.publish(string_msg)
                break
            elif response.status_code == 400:
                resp_data = response.json()
                print("Error:",resp_data['data']['message'])
                self.post_call_state = string_msg
                #self.publish_post_state.publish(string_msg)
                break
            elif response.status_code == 400:
                resp_data = response.json()
                print("Error:",resp_data['data']['message'])
                self.post_call_state = string_msg
                #self.publish_post_state.publish(string_msg)
                break
            
    def turn_left(self):
        twist = Twist()
        string = String()
        #turn 90 degress
        twist.angular.z = 0.3
        twist.linear.x = 0.0
        self.publish_cmd_vel.publish(twist)
        time.sleep(3)
        #stop after 3 secs after 90 deg achvd
        twist.angular.z = 0.0
        twist.linear.x = 0.5
        #move fwd
        self.publish_cmd_vel.publish(twist)
        time.sleep(5)
        trigger = 1
        string = 'checkpoint 2'
        self.publish_check_2.publish(string)
        
    def turn_right(self):
        twist = Twist()
        string = String()
        #turn 90 degress
        twist.angular.z = (-1) * 0.3
        twist.linear.x = 0.0
        self.publish_cmd_vel.publish(twist)
        time.sleep(3)
        #stop after 3 secs after 90 deg achvd
        twist.angular.z = 0.0
        twist.linear.x = 0.5
        #move fwd
        self.publish_cmd_vel.publish(twist)
        time.sleep(5)
        trigger = 1
        string = 'checkpoint 2'
        self.publish_check_2.publish(string)
        
def main(args=None):
    rclpy.init(args=args)
    post_request = PostRequest()
    while True:
        rclpy.spin(post_request)
        if trigger == 1:
            break
    post_request.destroyNode()
    rclpy.shutdown()
    

        
            