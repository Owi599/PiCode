import time
import RPi.GPIO as GPIO

direction = 10
Pulse = 9
cw = 0
ccw  =1 
GPIO.setmode(GPIO.BCM)
GPIO.setup(Pulse,GPIO.OUT)
GPIO.setup(direction,GPIO.OUT)
n = 0
try:
    while n <= 10:
        GPIO.output(direction, cw)
        GPIO.output(Pulse,GPIO.HIGH)
        time.sleep(.001e-12)
        GPIO.output(Pulse,GPIO.LOW)
        time.sleep(.001e-12)
      
        

except KeyboardInterrupt:
    GPIO.cleanup()
