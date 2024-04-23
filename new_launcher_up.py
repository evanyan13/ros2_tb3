import time
import pigpio

# Choose appropriate pwm channels to be used to control the servos
cont = 18
half = 20

pwm = pigpio.pi()

pwm.set_mode(cont, pigpio.OUTPUT)
pwm.set_PWM_frequency(cont, 50)

pwm.set_mode(half, pigpio.OUTPUT)
pwm.set_PWM_frequency(half, 50)

# For a continuous rotation servo, altering duty cycle / pulse width
# controls the speed of rotation of the servo, not the position of the shaft.

# Both motors for cont and half are continuous rotation servos

# SG92R 360 by Continental & FEETECH FS90R (cont)
# 100% counterclockwise speed = 1 ms pulse width = 5% duty cycle (for 50 Hz = 20 ms)
# Stop = around 1.5 ms pulse width = around 7.5% duty cycle (for 50 Hz = 20 ms)
# 100% clockwise speed = 2 ms pulse width = 10% duty cycle (for 50 Hz = 20 ms)
# May require some simple calibration:
# Output the desired stop duty cycle to the servo
# Then gently adjust the potentiometer in the recessed hole with a small screwdriver until the servo stops moving.

# SPT5535LV-360W (half)
# 100% counterclockwise speed = 0.5 - 0.95 ms pulse width = around 2.5% - 4.75% duty cycle (for 50 Hz = 20 ms)
# Stop = between 1.45 - 1.55 ms pulse width = around 7.25% - 7.75% duty cycle (for 50 Hz = 20 ms)
# 100% clockwise speed = 1.95 - 2.5 ms pulse width = around 9.75% - 12.5% duty cycle (for 50 Hz = 20 ms)

try:
    pwm.set_PWM_dutycycle(half, 0)    
    time.sleep(1)      # The delay is to allow time for the servo to react
    pwm.set_PWM_dutycycle(cont, 0)     
    time.sleep(1)      

    # Flip PVC pipe upright
    pwm.set_PWM_dutycycle(half, 6) # 100% counterclockwise
    time.sleep(0.85) # Calibrated time to rotate approximately 180
    pwm.set_PWM_dutycycle(half, 0)
    time.sleep(2.5)

    # Rotate the continuous rotation servo
    pwm.set_PWM_dutycycle(cont, 25) # 100% clockwise
    time.sleep(3.5)  # Rotate for 3.25 sec
    pwm.set_PWM_dutycycle(cont, 0) # Stop
    time.sleep(2.5)

except:
    pwm.set_PWM_dutycycle(half, 0)
    pwm.set_PWM_dutycycle(cont, 0)
    print("Forced stopped")
