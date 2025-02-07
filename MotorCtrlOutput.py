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

    def calculate_steps(self, Force,Speed):
        torque = Force * self.Rad
        
        V_max = 1000 #rpm
        
        
        if self.SPR == 0:
            raise ValueError('Steps per revolution cannot be zero')
        if Speed >= V_max:
            raise ValueError('Speed cannot be greater than 1000 rpm')
            Speed = V_max -1
        
        availible_tourque = self.HT * (1 - (Speed/V_max))
        
        if availible_tourque <= 0:
            raise ValueError('Speed is too high for the motor')
            steps_int = 0
        
        torque_per_step = availible_tourque/self.SPR
        
        steps_int = max(1, int(round(torque / torque_per_step)))  # Ensures at least 1 step
        return steps_int
    
    def Stepper(self,Force,direction):

        if direction == 1:
            GPIO.output(self.DIR, GPIO.HIGH)
        elif direction == -1:
            GPIO.output(self.DIR,GPIO.LOW)
        else:
            raise ValueError('Direction must be 1 or -1')
        
        steps = self.calculate_steps(Force)

        for step in range(abs(steps)):
            GPIO.output(self.Pulse,GPIO.HIGH)
            time.sleep(.002)
            GPIO.output(self.Pulse,GPIO.LOW)
            time.sleep(.001)