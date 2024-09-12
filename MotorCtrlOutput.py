import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

class CTRL():

    def __init__(self,PulsePin,DirPin,StepsPerRev,PulleyRad,HoldingTorque):
        self.Pulse = PulsePin
        self.DIR = DirPin
        self.SPR = StepsPerRev
        self.Rad = PulleyRad
        self.HT = HoldingTorque

    def calculate_steps(self, Force):
        torque = Force * self.Rad

        torque_per_step = self.HT/self.SPR

        steps = int(torque/torque_per_step)

        return steps
    
    def Stepper(self,Force,direction):
        GPIO.setup(self.Pulse,GPIO.output)
        GPIO.setup(self.DIR, GPIO.output)

        GPIO.output(self.DIR, GPIO.HIGH if direction > 0 else GPIO.LOW)

        for _ in range(abs(self.calculate_steps(Force))):
            GPIO.output(self.Pulse,GPIO.HIGH)
            