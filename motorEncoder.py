import pigpio
import time
import numpy as np
from pigpio_encoder.rotary import Rotary

class ReadMotorEncoder:
    def __init__(self, clk_gpio, dt_gpio, pi, wheel_radius = 0.0125):
        self.pi = pi 
        self.steps = 0
        self.wheel_radius = wheel_radius
        self.last_time = time.time()
        self.last_steps = 0

        self.rotary = Rotary(clk_gpio=clk_gpio,dt_gpio=dt_gpio,pi= self.pi)
        self.rotary.setup_rotary(rotary_callback =self._rotary_callback)

    def _rotary_callback(self,counter):
        self.steps = counter

    def calibrate(self):
        self.rotary.counter = 0
        self.steps = 0
        self.last_steps = 0
        print('Cart position caibrated to 0 meters')

    def read_position(self,cpr):
        circumference = 2* np.pi * self.wheel_radius
        distance_per_step = circumference/cpr

        position = self.steps * distance_per_step
        return position
    
    def read_velocity(self, cpr):
        current_time = time.time()
        current_steps = self.steps

        time_diff = current_time - self.last_time
        step_diff = current_steps - self.last_steps

        if time_diff < 0.00001:
            return 0 , current_time, current_steps
        
        circumference = 2* np.pi * self.wheel_radius
        distance_per_step = circumference/cpr
        velocity = (step_diff/time_diff) *distance_per_step

        self.last_time = current_time
        self.last_steps = current_steps

        return velocity, current_time, current_steps
    
    def cleanup(self):
        self.rotary.cancel()

