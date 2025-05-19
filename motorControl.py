import RPi.GPIO as GPIO # GPIO library for Raspberry Pi
import time             # Time library for time operations
import math as m        # Math library for mathematical operations 
import numpy as np      # Numpy library for numerical operations

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Class for controlling motor output
class MotorControl():
    # Initialize the motor control with GPIO pins and parameters
    def __init__(self,pulsePin,directionPin,stepsPerRev,pulleyRad,holdingTorque):
        self.pulsePin = pulsePin
        self.DIR = directionPin
        GPIO.setup(pulsePin,GPIO.OUT)
        GPIO.setup(directionPin,GPIO.OUT)
        GPIO.output(pulsePin,GPIO.LOW)
        self.stepsPerRev = stepsPerRev
        self.pulleyRad = pulleyRad
        self.holdingTorque = holdingTorque
        self.microresolution = 16
        self.m_total = 0.232 + 0.127 + 0.127
        self.cpr = 500
        self.velocity_max = 2000
        
    def calculate_steps(self, force,velocity_intit,position_init,t_sample):
        
        #velocity and Speed in RPM
        acceleration = force/self.m_total
        velocity = acceleration*t_sample + velocity_intit
        radialvelocity = velocity *self.pulleyRad
        speed_rpm = radialvelocity * 9.549297
        
        #Check for maximum speed
        if abs(speed_rpm) > self.velocity_max:
            #print(speed_rpm)
            speed_rpm = np.sign(speed_rpm)*self.velocity_max + (-np.sign(speed_rpm)*1)  
            velocity = speed_rpm * 0.01047198
            print('Warning:Speed cannot be greater than 2000 rpm')
        
        #caclcuate the positison
        position = 0.5*acceleration*t_sample**2 + velocity_intit*t_sample + position_init
        
        steps_int = max(int(round(((40*self.cpr)/2*np.pi)*position)), 1)  # Ensures at least 1 step
        #print(steps_int)
        stepFreq = (abs(velocity) * 3200) / (self.pulleyRad/2*np.pi)  # Frequency in Hz
        
        if stepFreq == 0:
            stepPeriod = 0  # Motor is not moving
        else:
            stepPeriod = 1 / stepFreq
        
        # # Set maximum delay to prevent out-of-bound values
        maxDelay = t_sample   # maximum delay
        stepPeriod = min(stepPeriod, maxDelay)
		
        
        return steps_int, stepPeriod, velocity, position
        
        
    def move_stepper(self,steps, stepPeriod, direction):

        if direction == 1:
            GPIO.output(self.DIR, GPIO.HIGH)
        elif direction == -1:
            GPIO.output(self.DIR,GPIO.LOW)
        else:
            raise ValueError('Direction must be 1 or -1')
        

        for step in range(abs(steps)):
            GPIO.output(self.pulsePin,GPIO.HIGH)
            time.sleep(stepPeriod/2)
            GPIO.output(self.pulsePin,GPIO.LOW)
            time.sleep(stepPeriod/2)

    def stop_motor(self):
        print('Motor stopping')
        GPIO.output(self.pulsePin,GPIO.LOW)
        GPIO.output(self.DIR,GPIO.LOW)
        print('Motor stopped')
        time.sleep(3)