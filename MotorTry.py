import time
import RPi.GPIO as GPIO

directtion = 10
Pulse = 9
cw = 0
ccw  =1 
GPIO.setmode(GPIO.BCM)
GPIO.setup(Pulse,GPIO.OUT)
GPIO.setup(directtion,GPIO.OUT)

try:
    while True:
        GPIO.output(directtion, ccw)
        GPIO.output(Pulse,GPIO.HIGH)
        time.sleep(.002)
        GPIO.output(Pulse,GPIO.LOW)
        time.sleep(.001)
except KeyboardInterrupt:
    GPIO.cleanup()
