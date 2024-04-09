import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

class ColorSensor(Node):
    def __init__(self):
        super().__init__('color_sensor')
        self.setup()
        self.publisher_color = self.create_publisher(String, 'color_info', 10)
        self.color_detect()

    def setup(self):
        GPIO.setmode(GPIO.BCM)
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
        # Wait for a full cycle (this will make sure we only count full cycles)
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
        color_val = String()
        while True:
            r = self.color_read_value(GPIO.LOW, GPIO.LOW)
            time.sleep(0.1) 
            g = self.color_read_value(GPIO.HIGH, GPIO.HIGH) 
            time.sleep(0.1) 
            b = self.color_read_value(GPIO.LOW, GPIO.HIGH)
            time.sleep(0.1) 
            print(f"Red:{r}, Green:{g}, Blue:{b}")
            # Larger the half period, larger the period, lower the frequency, lower the intensity for the particular colour.
            # Smaller the half period, smaller the period, higher the frequency, higher the intensity for the particular colour.
            if (b < g) and (b < r): 
                color_val.data = "BLUE"
            elif (g < b) and (g < r): 
                color_val.data = "GREEN"
            elif (r < g) and (r < b): 
                color_val.data = "RED"
            time.sleep(1)
            self.publisher_color.publish(color_val)
            self.get_logger().info(f"Published colour: {color_val.data}")

def main(args=None):
    rclpy.init(args=args)
    color_sensor = ColorSensor()
    try:
        rclpy.spin(color_sensor)
    except KeyboardInterrupt:
        color_sensor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

