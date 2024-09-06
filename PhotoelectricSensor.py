import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

class EndSwitch():
    def __init__(self, pin):
        self.pin = pin
        self.pin.set_mode(pin.IN)
        self.pin.set_pull(pin.PULL_UP)
        

    def read(self):
        while True:
            if GPIO.input(self.pin):
                return 1
            else:
                return 0
            
    def Object_detected(self):
        while True:
            if self.read() == 1:
                return 1
    
