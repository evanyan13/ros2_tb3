import time
import RPi.GPIO as GPIO
import rclpy
from rclpy import Node
from std_msgs.msg import String
import pigpio
pwm = pigpio.pi()
cont = 18
half = 20

class Launcher(Node):
    def __init__(self):
        super().__init__("launcher")
        self.subscriber_state = self.create_subscription(String,'enter_state',self.check_launch(),10)
        self.setup()
        
    # Set pin numbering convention
    def setup(self):
        pwm.set_mode(cont, pigpio.OUTPUT)
        pwm.set_PWM_frequency(cont, 50)
        pwm.set_mode(half, pigpio.OUTPUT)
        pwm.set_PWM_frequency(half, 50)
        self.check_launch()
    
    def check_launch(self,msg):
        if msg.data == 'start launch sequence':
            self.start_launch()
        else:
            print("error no launch msg")
    
    def start_launch(self):
        pwm.set_PWM_dutycycle(half, 0)    
        time.sleep(1)      # The delay is to allow time for the servo to react
        pwm.set_PWM_dutycycle(cont, 0)     
        time.sleep(1)
        
        # Flip PVC pipe upright
        pwm.set_PWM_dutycycle(half, 15) # slow counterclockwise
        time.sleep(2.5) # Calibrated time to rotate approximately 180
        pwm.set_PWM_dutycycle(half, 0)
        time.sleep(2.5)
        
        # Rotate the continuous rotation servo
        pwm.set_PWM_dutycycle(cont, 25) # 100% clockwise
        time.sleep(3.25)  # Rotate for 3.25 sec
        pwm.set_PWM_dutycycle(cont, 0) # Stop
        time.sleep(2.5)
        
def main(args=None):
    rclpy.inti(args=args)
    launcher = Launcher()
    try:
        rclpy.spin(launcher)
    except KeyboardInterrupt():
        cont.stop()
        half.stop()
        GPIO.cleanup()
        rclpy.destroyNode()
        
if __name__ == '__main__':
    main()
