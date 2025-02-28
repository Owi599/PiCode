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
        
    def calculate_steps(self, Force,t_sample):
        
        #Force to toruqe conversion
        Torque_required = Force * self.PulleyRad

        #Velocity and Speed in RPM
        Acceleration = Force/self.m_total
        Velocity = Acceleration*t_sample
        Speed_RPM = (Velocity * 60)/(2*np.pi*self.PulleyRad)
        
        #Check for maximum speed
        if abs(Speed_RPM) > self.V_max:
            print(Speed_RPM)
            Speed_RPM = np.sign(Speed_RPM)*self.V_max - 1
            Velocity = (Speed_RPM * 2*np.pi*self.PulleyRad)/60
            print('Warning:Speed cannot be greater than 2000 rpm')
        
        
        #Calculate available torque
        Torque_available = self.HoldingTorque * (1 - (abs(Speed_RPM)/self.V_max))
        
        #Check if required torque is greater than available torque
        if Torque_available < Torque_required:
            raise ValueError('Required torque is greater than available torque at this speed')
        
        #Calculate steps per revolution with microstepping    
        Steps_per_rev = self.StepsPerRev * self.Microresolution
        
        
        TorquePerStep = Torque_available/Steps_per_rev
        
        steps_int = max(1, int(round(Torque_required / TorquePerStep)))  # Ensures at least 1 step
        print(steps_int)
        step_freq = (abs(Velocity) * Steps_per_rev) / (2*np.pi*self.PulleyRad)  # Frequency in Hz
        
        if step_freq == 0:
            step_period = 0  # Motor is not moving
        else:
            step_period = 1 / step_freq
        
        # # Set maximum delay to prevent out-of-bound values
        MAX_DELAY = t_sample   # maximum delay
        step_period = min(step_period, MAX_DELAY)
        print(step_period)
		
        
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
