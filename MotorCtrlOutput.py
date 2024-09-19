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

        steps = round(torque/torque_per_step)

        steps_int =    int(steps)
        print(steps_int)
        return steps_int
    
    def Stepper(self,Force,direction):


        GPIO.output(self.DIR, GPIO.HIGH if direction > 0 else GPIO.LOW)

        for _ in range(abs(self.calculate_steps(Force))):
            GPIO.output(self.Pulse,GPIO.HIGH)
            time.sleep(.001)
            GPIO.output(self.Pulse,GPIO.LOW)
            time.sleep(.0005)