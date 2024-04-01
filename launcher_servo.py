import time
import RPi.GPIO as GPIO

# Set pin numbering convention
GPIO.setmode(GPIO.BCM)

# Choose appropriate pwm channels to be used to control the servos
import time
import RPi.GPIO as GPIO

# Set pin numbering convention
GPIO.setmode(GPIO.BCM)

# Choose appropriate pwm channels to be used to control the servos
cont_servo = 18
half_servo = 20

# Configure the servo pins to output
GPIO.setup(cont_servo, GPIO.OUT)
GPIO.setup(half_servo, GPIO.OUT)

# Initialise the servo pins to output pwm with 50 Hz frequency
cont = GPIO.PWM(cont_servo, 50)
half = GPIO.PWM(half_servo, 50)

# For a continuous rotation servo, altering duty cycle / pulse width
# controls the speed of rotation of the servo, not the position of the shaft.

# Both motors for cont and half are continuous rotation servos

# SG92R 360 by Continental & FEETECH FS90R (cont)
# 100% counterclockwise speed = 1 ms pulse width = 5% duty cycle (for 50 Hz = 20 ms)
# Stop = around 1.5 ms pulse width = around 7.5% duty cycle (for 50 Hz = 20 ms)
# 100% clockwise speed = 2 ms pulse width = 10% duty cycle (for 50 Hz = 20 ms)
# May require some simple calibration:
# Output the desired stop duty cycle to the servo
# Then gently adjust the potentiometer in the recessed hole with a small screwdriver until the servo>

# SPT5535LV-360W (half)
# 100% counterclockwise speed = 0.5 - 0.95 ms pulse width = around 2.5% - 4.75% duty cycle (for 50 H>
# Stop = between 1.45 - 1.55 ms pulse width = around 7.25% - 7.75% duty cycle (for 50 Hz = 20 ms)
# 100% clockwise speed = 1.95 - 2.5 ms pulse width = around 9.75% - 12.5% duty cycle (for 50 Hz = 20>

half.start(7.25)    # The pin starts to output PWM signals at 50 Hz with an initial duty cycle of 7.>
time.sleep(1)      # The delay is to allow time for the servo to react
cont.start(7.5)    
time.sleep(1)      

# Flip PVC pipe upright
half.ChangeDutyCycle(4.75) # slow counterclockwise
time.sleep(0.85) # Calibrated time to rotate approximately 180 assuming 61 RPM
half.ChangeDutyCycle(7.25)
time.sleep(2.5)

# Flip PVC pipe back down test code
# half.ChangeDutyCycle(9.75) # slow clockwise
# time.sleep(0.5) # Calibrated time to rotate approximately 180 assuming 61 RPM
# half.ChangeDutyCycle(7.25)
# time.sleep(2.5)

# Rotate the continuous rotation servo
cont.ChangeDutyCycle(10) # 100% clockwise
time.sleep(5)  # Rotate for 5 sec
cont.ChangeDutyCycle(7.5) # Stop
time.sleep(2.5)

# Cleanup
cont.stop()
half.stop()
GPIO.cleanup()
