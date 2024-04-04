import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

class ColorSensor(Node):
    def __init__(self):
        super().__init__('color_sensor')
        self.publisher_color = self.create_publisher(String, 'color_info', 10)
        self.timer = self.create_timer(3,self.color_detect) #publishes every 3 sec
        
    def setup(self):
        GPIO.setmode(GPIO.BCM)
        self.S2 = 22
        self.S3 = 23
        self.OUT = 24
        GPIO.setup(self.S2,GPIO.OUT)
        GPIO.setup(self.S3,GPIO.OUT)
        GPIO.setup(self.OUT,GPIO.IN)
        
    def color_read_value(self,a0,a1):
        GPIO.output(self.s2, a0) 
        GPIO.output(self.s3, a1) 
        # Give the sensor some time to adjust 
        time.sleep(0.1) 
        # Wait for a full cycle (this will make sure we only count full cycles) 
        GPIO.wait_for_edge(self.out, GPIO.FALLING) 
        GPIO.wait_for_edge(self.out, GPIO.RISING) 
        # print("waiting")
        start = time.time() 
        GPIO.wait_for_edge(self.out, GPIO.FALLING) 
        # The time that passed while we were waiting for the out to change
        # print("returning successfully")
        return (time.time() - start) * 1000000 
        
    
    def color_detect(self):
        color_val = String()
        while True:
            # print("in loop")
            r = self.color_read_value(GPIO.LOW, GPIO.LOW) 
            time.sleep(0.1) 
            g = self.color_read_value(GPIO.HIGH, GPIO.HIGH) 
            time.sleep(0.1) 
            b = self.color_read_value(GPIO.LOW, GPIO.HIGH) 
            if (b < g) and (b < r): 
                color_val.data = "BLUE"
            elif (g < b) and (g < r): 
                color_val.data = "GREEN"
            elif (r < g) and (r < b): 
                color_val.data = "RED"
            time.sleep(1)
            self.publisher_color.publish(color_val)

def main(args=None):
    rclpy.init(args=args)
    color_sensor = ColorSensor()
    rclpy.spin(color_sensor)
    color_sensor.destroyNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
       

    
    
          
          
        