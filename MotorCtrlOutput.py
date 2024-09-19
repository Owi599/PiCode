import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

class CTRL():

    def __init__(self,PulsePin,DirPin,StepsPerRev,PulleyRad,HoldingTorque):
        self.Pulse = PulsePin
        self.DIR = DirPin
        GPIO.setup(PulsePin,GPIO.OUT)
        GPIO.setup(DirPin,GPIO.OUT)
        self.SPR = StepsPerRev
        self.Rad = PulleyRad
        self.HT = HoldingTorque

    def calculate_steps(self, Force):
        torque = Force * self.Rad

        torque_per_step = self.HT/self.SPR


        steps_int =    int(round(torque/torque_per_step))
        return steps_int
    
    def Stepper(self,Force,direction):

        if direction > 0:
            GPIO.output(self.DIR, GPIO.HIGH)
        else:
            GPIO.output(self.DIR,GPIO.LOW)
        steps = self.calculate_steps(Force)

        for step in range(abs(steps)):
            GPIO.output(self.Pulse,GPIO.HIGH)
            time.sleep(.001)
            GPIO.output(self.Pulse,GPIO.LOW)
            time.sleep(.0005)