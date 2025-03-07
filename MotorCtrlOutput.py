import RPi.GPIO as GPIO
import time
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
        self.Microresolution = 16
        self.m_total = 0.232 + 0.127 + 0.127
        self.CPR = 500
        self.V_max = 2000
        
    def calculate_steps(self, Force,V_intit,X_init,t_sample):
        
        #Velocity and Speed in RPM
        Acceleration = Force/self.m_total
        Velocity = Acceleration*t_sample + V_intit
        Radial_Velocity = Velocity *self.PulleyRad
        Speed_RPM = Radial_Velocity * 9.549297
        
        #Check for maximum speed
        if abs(Speed_RPM) > self.V_max:
            print(Speed_RPM)
            Speed_RPM = np.sign(Speed_RPM)*self.V_max + (-np.sign(Speed_RPM)*1)  
            Velocity = Speed_RPM * 0.01047198
            print('Warning:Speed cannot be greater than 2000 rpm')
        
        #caclcuate the positison
        X = 0.5*Acceleration*t_sample**2 + V_intit*t_sample + X_init
        
        steps_int = max(int(round(((40*self.CPR)/2*np.pi)*X)), 1)  # Ensures at least 1 step
        print(steps_int)
        step_freq = (abs(Velocity) * 3200) / (self.PulleyRad/2*np.pi)  # Frequency in Hz
        
        if step_freq == 0:
            step_period = 0  # Motor is not moving
        else:
            step_period = 1 / step_freq
        
        # # Set maximum delay to prevent out-of-bound values
        MAX_DELAY = t_sample   # maximum delay
        step_period = min(step_period, MAX_DELAY)
		
        
        return steps_int, step_period, Velocity, X
        
        
    def Stepper(self,Force,direction,V_init,X_init,t_sample):

        if direction == 1:
            GPIO.output(self.DIR, GPIO.HIGH)
        elif direction == -1:
            GPIO.output(self.DIR,GPIO.LOW)
        else:
            raise ValueError('Direction must be 1 or -1')
        
        steps, step_period, Velocity, Position = self.calculate_steps(Force,V_init,X_init,t_sample)

        for step in range(abs(steps)):
            GPIO.output(self.PulsePin,GPIO.HIGH)
            time.sleep(step_period/2)
            GPIO.output(self.PulsePin,GPIO.LOW)
            time.sleep(step_period/2)
