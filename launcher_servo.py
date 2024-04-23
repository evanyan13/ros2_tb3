import time 
import RPi.GPIO as GPIO 
import rclpy 
from rclpy.node import Node 
from std_msgs.msg import String 
from geometry_msgs.msg import Twist
import pigpio 

pwm = pigpio.pi()
cont = 18
half = 20
pwm.set_mode(cont, pigpio.OUTPUT) 
pwm.set_PWM_frequency(cont, 50) 
pwm.set_mode(half, pigpio.OUTPUT) 
pwm.set_PWM_frequency(half, 50) 

class Launcher(Node): 
    def __init__(self): 
        super().__init__("launcher") 
        self.subscriber_state = self.create_subscription(String,'enter_state',self.check_launch,10)
        self.publisher_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        print("Launcher script started")

    def check_launch(self,msg):  
        if msg.data == 'start launch sequence': 
            self.start_launch() 
        else: 
            print("launcher msg not received") 

    def start_launch(self):
        self.stop_movement()
        print("----------------------------------------------------------------------------------------------------------------------------------------------------------------------------")
        print("Launcher started")
        print("-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------")
        pwm.set_PWM_dutycycle(half, 0)     
        time.sleep(1)      # The delay is to allow time for the servo to react 
        pwm.set_PWM_dutycycle(cont, 0)      
        time.sleep(1) 

        # Flip PVC pipe upright
        print("Flipping PVC pipe upright") 
        pwm.set_PWM_dutycycle(half, 6) # slow counterclockwise 
        time.sleep(0.85) # Calibrated time to rotate approximately 180 
        pwm.set_PWM_dutycycle(half, 0) 
        time.sleep(1)

        # Rotate the continuous rotation servo
        print("Rotating rack") 
        pwm.set_PWM_dutycycle(cont, 25) # 100% clockwise 
        time.sleep(3.5)  # Rotate for 3.25 sec 
        pwm.set_PWM_dutycycle(cont, 0) # Stop 
        time.sleep(1)

        raise KeyboardInterrupt

    def stop_movement(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_cmd_vel.publish(twist)
        self.get_logger().info(f"Twist: {twist.linear.x} & {twist.angular.z}")
        print("stopped movement")

def main(args=None): 
    rclpy.init(args=args) 
    launcher = Launcher() 
    try: 
        rclpy.spin(launcher) 
    except:
        pwm.set_PWM_dutycycle(half, 0)
        pwm.set_PWM_dutycycle(cont, 0)
        launcher.destroy_node() 
        rclpy.shutdown() 

if __name__ == '__main__': 
    main()
