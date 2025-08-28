import RPi.GPIO as GPIO  # GPIO library for Raspberry Pi
import time              # Time library for time operations
import math as m         # Math library for mathematical operations 
import numpy as np       # Numpy library for numerical operations
import timeout_decorator # Timeout decorator for function execution time limits

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Class for controlling motor output
class MotorControl():
    # Initialize the motor control with GPIO pins and parameters
    def __init__(self,pulsePin,directionPin,stepsPerRev,pulleyRad,holdingTorque,t_sample):
        self.pulsePin = pulsePin
        self.DIR = directionPin
        GPIO.setup(pulsePin,GPIO.OUT)
        GPIO.setup(directionPin,GPIO.OUT)
        GPIO.output(pulsePin,GPIO.LOW)
        self.stepsPerRev = stepsPerRev
        self.pulleyRad = pulleyRad
        self.holdingTorque = holdingTorque
        self.t_sample = t_sample 
        self.microresolution = 4
        self.m_total = 0.6 + 0.104 + 0.102
        self.cpr = 500
        self.velocity_max = 0.1047*self.pulleyRad*2000  # Maximum speed in m/s (2000 RPM)

        # Integration terms for vel and position
        self.velocity_integral = 0
        self.position_integral = 0
        self.dt = self.t_sample  # Time step for integration
        
        
    def calculate_steps(self, force):
        startTime = time.perf_counter()
        #velocity and Speed in RPM
        acceleration = force/self.m_total
        print('Acceleration:',acceleration)
        self.velocity_integral += acceleration*self.dt 
        self.position_integral += self.velocity_integral*self.dt
        print('End position',self.position_integral)
        print('Velocity before clipping',self.velocity_integral)
        #Check for maximum speed
        if abs(self.velocity_integral) > self.velocity_max:
            self.velocity_integral = np.sign(self.velocity_integral) * self.velocity_max
            print('Warning:Speed cannot be greater than 2000 rpm')
        print('Velocity after clipping:',self.velocity_integral)
        
        steps_int = abs(max(int((self.position_integral*self.stepsPerRev*self.microresolution)/(2*np.pi*self.pulleyRad)), 1))  # Ensures at least 1 step
        print(steps_int)
        stepFreq = (abs(self.velocity_integral) * self.microresolution) / (self.pulleyRad/(2*np.pi))  # Frequency in Hz
        print(stepFreq)
        if stepFreq == 0:
            stepPeriod = 0  # Motor is not moving
        else:
            stepPeriod = 1 / stepFreq
        
        # # Set maximum delay to prevent out-of-bound values
        maxDelay = self.t_sample   # maximum delay
        print(maxDelay)
        stepPeriod = float(min(stepPeriod, maxDelay))
        print(stepPeriod)
        endTime = time.perf_counter()
        stepCalculationTime = endTime - startTime
        direction = int(1*np.sign(self.velocity_integral))  # Determine direction based on velocity sign
        print(direction)
        return steps_int, stepPeriod, stepCalculationTime , direction
    
   
    
    
    @timeout_decorator.timeout(0.02)  # Set a timeout of 0.02 seconds for the move_stepper function
    def move_stepper(self,steps:int, stepPeriod:float, direction:int):
        if direction == 1:
            GPIO.output(self.DIR, GPIO.HIGH)
        elif direction == -1:
            GPIO.output(self.DIR,GPIO.LOW)
        else:
            raise ValueError('Direction must be 1 or -1')
        
            
        startTime = time.perf_counter()
        for step in range(steps):
            GPIO.output(self.pulsePin,GPIO.HIGH)
            time.sleep(stepPeriod/2)
            GPIO.output(self.pulsePin,GPIO.LOW)
            time.sleep(stepPeriod/2)
        endTime = time.perf_counter
        movementTime = endTime - startTime
        return movementTime

    def stop_motor(self):
        print('Motor stopping')
        GPIO.output(self.pulsePin,GPIO.LOW)
        GPIO.output(self.DIR,GPIO.LOW)
        print('Motor stopped')
        time.sleep(3)