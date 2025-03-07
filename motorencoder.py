import time
import numpy as np
from gpiozero import RotaryEncoder

class ReadMotorEncoder(RotaryEncoder):
	def readPosition(self,CPR):
		# Calculate the angle and position
		angle = (2 * np.pi / CPR) * self.steps
		position = (angle * 25) / 1000  # m
		return position

	def readVelocity(self,CPR,l_t,l_s):
		current_time = time.time()
		current_steps = self.steps
        
		# Calculate the time difference and step difference
		time_diff = current_time - l_t
		step_diff = current_steps - l_s
		
		min_time_diff = 0.001
		
		time_diff = max(time_diff, min_time_diff)

        
		# Calculate speed 
		velocity = (step_diff /time_diff) * (2*np.pi/ CPR)  # rad/s
        
		# Update last_time and last_steps
		
		return velocity, current_time, current_steps
 
