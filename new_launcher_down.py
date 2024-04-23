import time
import pigpio

cont = 18
half = 20

pwm = pigpio.pi()

pwm.set_mode(cont, pigpio.OUTPUT)
pwm.set_PWM_frequency(cont, 50)

pwm.set_mode(half, pigpio.OUTPUT)
pwm.set_PWM_frequency(half, 50)

try:
    pwm.set_PWM_dutycycle(half, 0)
    time.sleep(1)      # The delay is to allow time for the servo to react
    pwm.set_PWM_dutycycle(cont, 0)
    time.sleep(1)

    pwm.set_PWM_dutycycle(cont, 12) # 100% counterclockwise
    time.sleep(3.25)
    pwm.set_PWM_dutycycle(cont, 0)
    time.sleep(2.5)

    pwm.set_PWM_dutycycle(half, 25) # slow clockwise
    time.sleep(0.5)
    pwm.set_PWM_dutycycle(half, 0)
    time.sleep(2.5)

except:
    pwm.set_PWM_dutycycle(half, 0)
    pwm.set_PWM_dutycycle(cont, 0)
    print("Forced stopped")

