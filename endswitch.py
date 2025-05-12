import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

class EndSwitch():
    def __init__(self, pin):
        self.pin = pin
        GPIO.setup(self.pin,GPIO.IN)
        

    def read_sensor(self):
        while True:
            if GPIO.input(self.pin):
                return 1
            else:
                return 0
            
    def is_object_detected(self):
        while True:
            if self.read() == 0:
                return True
    
