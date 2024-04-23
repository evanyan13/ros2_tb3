import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

range_r = [0, 0]
range_g = [0, 0]
range_b = [0, 0]

class ColorSensor(Node):
    def __init__(self):
        super().__init__('color_sensor')
        self.setup()
        print("Colour sensor started")

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
        r = self.color_read_value(GPIO.LOW, GPIO.LOW)
        range_r[0] = r
        range_r[1] = r
        g = self.color_read_value(GPIO.HIGH, GPIO.HIGH)
        range_g[0] = g
        range_g[1] = g 
        b = self.color_read_value(GPIO.LOW, GPIO.HIGH)
        range_b[0] = b
        range_b[1] = b
        while True:
            r = self.color_read_value(GPIO.LOW, GPIO.LOW)
            g = self.color_read_value(GPIO.HIGH, GPIO.HIGH) 
            b = self.color_read_value(GPIO.LOW, GPIO.HIGH) 
            print(f"Red:{r}, Green:{g}, Blue:{b}")
            # Larger the half period, larger the period, lower the frequency, lower the intensity for the particular colour.
            # Smaller the half period, smaller the period, higher the frequency, higher the intensity for the particular colour.
            if r < range_r[0]:
                range_r[0] = r
            if r > range_r[1]:
                range_r[1] = r 
            if g < range_g[0]: 
                range_g[0] = g 
            if g > range_g[1]: 
                range_g[1] = g 
            if b < range_b[0]: 
                range_b[0] = b 
            if b > range_b[1]: 
                range_b[1] = b
            if r > 900  and g > 900 and b > 700:
                print("GROUND")
            elif r < 500 and g < 500 and b < 350:
                print( "WHITE")
            elif (r < g) and (r < b):
                print("RED")
            elif (b < g) and (b < r): 
                print("BLUE")
            elif (g < b) and (g < r): 
                print("GREEN")

def main(args=None):
    rclpy.init(args=args)
    color_sensor = ColorSensor()
    try:
        color_sensor.color_detect()
    except KeyboardInterrupt:
        print(f"Range of red: {range_r}")
        print(f"Range of green: {range_g}")
        print(f"Range of blue: {range_b}")
        color_sensor.destroy_node()
        rclpy.shutdown()
        GPIO.cleanup()

if __name__ == '__main__':
    main()
