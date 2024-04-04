import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

cont_servo = 18
half_servo = 20

GPIO.setup(cont_servo, GPIO.OUT)
GPIO.setup(half_servo, GPIO.OUT)

cont = GPIO.PWM(cont_servo, 50)
half = GPIO.PWM(half_servo, 50)

half.start(7.25)
time.sleep(1)
cont.start(7.5)
time.sleep(1)

cont.ChangeDutyCycle(5) # 100% counterclockwise
time.sleep(3.25)
cont.ChangeDutyCycle(7.5)
time.sleep(2.5)

#half.ChangeDutyCycle(8) # slow clockwise
#time.sleep(2)
#half.ChangeDutyCycle(7.25)
#time.sleep(2.5)

cont.stop()
half.stop()
GPIO.cleanup()


