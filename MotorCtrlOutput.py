import RPi.GPIO as GPIO
import time
import scipy.integrate as spi
import math as m
import numpy as np
GPIO.setmode(GPIO.BCM)

class CTRL():

    def __init__(self,PulsePin,DirectionPin,StepsPerRev,PulleyRad,HoldingTorque):
        self.PulsePin = PulsePin
        self.DIR = DirectionPin
        GPIO.setup(PulsePin,GPIO.OUT)
        GPIO.setup(DirectionPin,GPIO.OUT)
        self.StepsPerRev = StepsPerRev
        self.PulleyRad = PulleyRad
        self.HoldingTorque = HoldingTorque

    def calculate_steps(self, Force,t_sample):
        m_total = 0.232 + 0.127 + 0.127
        CPR = 500        
        Acceleration = Force/m_total
        Velocity = Acceleration*t_sample
        RPM = (Velocity * 60)/(2*np.pi*0.025)
        if RPM >= 2000:
            RPM = 1999
            Velocity = (RPM * 2*np.pi*0.025)/60
            raise ValueError('Speed cannot be greater than 2000 rpm')
        if RPM <= -2000:
            RPM = -1999
            Velocity = (RPM * 2*np.pi*0.025)/60
            raise ValueError('Speed cannot be lesser than -2000 rpm')
        Position = Velocity*t_sample
        
        steps_int = int((CPR/0.05*np.pi)*Position)
        
        # torque = Force * self.PulleyRad
        # if self.StepsPerRev == 0:
        #     raise ValueError('Steps per revolution cannot be zero')
        # if Speed >= V_max:
        #     Speed = V_max -1
        #     raise ValueError('Speed cannot be greater than 2000 rpm')
            
        # if Speed <= (-V_max):
        #     Speed = -V_max +1
        #     raise ValueError('Speed cannot be lesser than -2000 rpm')
        # # Set minimum speed to prevent extremely large delays
        # MIN_SPEED = 1  # 1 RPM minimum
        # if Speed == 0 or abs(Speed) < MIN_SPEED:
        #     Speed = MIN_SPEED if Speed >= 0 else -MIN_SPEED

        
        # availible_torque = self.HoldingTorque * (1 - (abs(Speed)/V_max))
        
        # if availible_torque <= 0:
        #     raise ValueError('Speed is too high for the motor')
        #     steps_int = 0
        
        # torque_per_step = availible_torque/self.StepsPerRev
        
        # steps_int = max(1, int(round(torque / torque_per_step)))  # Ensures at least 1 step
         
        step_freq = (abs(Velocity) * self.StepsPerRev) / 60
        step_period = 1 / step_freq
        
        # # Set maximum delay to prevent out-of-bound values
        # MAX_DELAY = 1.0  # 1 second maximum delay
        # step_period = min(step_period, MAX_DELAY)

		
        
        return steps_int, step_period
        
        
    def Stepper(self,Force,direction,t_sample):

        if direction == 1:
            GPIO.output(self.DIR, GPIO.HIGH)
        elif direction == -1:
            GPIO.output(self.DIR,GPIO.LOW)
        else:
            raise ValueError('Direction must be 1 or -1')
        
        steps, step_period = self.calculate_steps(Force,t_sample)

        for step in range(abs(steps)):
            GPIO.output(self.PulsePin,GPIO.HIGH)
            time.sleep(step_period/2)
            GPIO.output(self.PulsePin,GPIO.LOW)
            time.sleep(step_period/2)
