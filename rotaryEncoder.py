import pigpio
import time
import numpy as np
from pigpio_encoder.rotary import Rotary

class ReadRotaryEncoder:
    def __init__(self, clk_gpio, dt_gpio,id ):
        
        self.steps = 0
        self._step_offset = 0
        self.last_time = time.time()
        self.last_steps = 0
        self.id = id
        self.rotary = Rotary(clk_gpio=clk_gpio, dt_gpio=dt_gpio,sw_gpio =None)
        self.rotary.setup_rotary(rotary_callback=self._rotary_callback)

    def _rotary_callback(self, counter):
        self.steps = counter

    def calibrate(self, cpr ,target_angle):
        new_counter_value = int((target_angle * cpr)/(2*np.pi))
        self.rotary.counter = new_counter_value
        self.steps = new_counter_value
        self.last_steps = new_counter_value
        print(f'encoder {self.id} calibrated. Current angular position set to {target_angle:.f} rad (with raw step count at: {self.steps}).')
    
    def read_position(self, cpr):
        raw_value = (2*np.pi/cpr)*self.steps
        wrapped_angle = (raw_value + np.pi) % (2 *np.pi) - np.pi
        return wrapped_angle
    
    def read_velocity(self,cpr):
        
        current_time = time.time()
        current_steps = self.steps     

        time_diff = current_time - self.last_time
        step_diff = current_steps - self.last_steps

        if time_diff < 0.000001:
            return 0 , current_time, current_steps

        angular_velocity = (step_diff / time_diff) * (2* np.pi /cpr)

        self.last_time = current_time
        self.last_steps = current_steps

        return angular_velocity, current_time, current_steps
    
    def cleanup(self):
        self.rotary.cancel()