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
        
        V_max = 2000 #rpm
        
        
        if self.SPR == 0:
            raise ValueError('Steps per revolution cannot be zero')
        if Speed >= V_max:
            Speed = V_max -1
            raise ValueError('Speed cannot be greater than 2000 rpm')
            
        if Speed <= (-V_max):
            Speed = -V_max +1
            raise ValueError('Speed cannot be lesser than -2000 rpm')
        # Set minimum speed to prevent extremely large delays
        MIN_SPEED = 1  # 1 RPM minimum
        if Speed == 0 or abs(Speed) < MIN_SPEED:
            Speed = MIN_SPEED if Speed >= 0 else -MIN_SPEED

        
        availible_torque = self.HT * (1 - (abs(Speed)/V_max))
        
        if availible_torque <= 0:
            raise ValueError('Speed is too high for the motor')
            steps_int = 0
        
        torque_per_step = availible_torque/self.SPR
        
        steps_int = max(1, int(round(torque / torque_per_step)))  # Ensures at least 1 step
         
        step_freq = (abs(Speed) * self.SPR) / 60
        step_delay = 1 / step_freq
        
        # Set maximum delay to prevent out-of-bound values
        MAX_DELAY = 1.0  # 1 second maximum delay
        step_delay = min(step_delay, MAX_DELAY)

		
        
        return steps_int, step_delay
        
        
    def Stepper(self,Force,direction, speed):

        if direction == 1:
            GPIO.output(self.DIR, GPIO.HIGH)
        elif direction == -1:
            GPIO.output(self.DIR,GPIO.LOW)
        else:
            raise ValueError('Direction must be 1 or -1')
        
        steps, step_delay = self.calculate_steps(Force,speed)

        for step in range(abs(steps)):
            GPIO.output(self.Pulse,GPIO.HIGH)
            time.sleep(step_delay/2)
            GPIO.output(self.Pulse,GPIO.LOW)
            time.sleep(step_delay/2)
