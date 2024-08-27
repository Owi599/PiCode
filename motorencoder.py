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
		time.sleep(0.002)			
		current_time = time.time()
		current_steps = self.steps
        
		# Calculate the time difference and step difference
		time_diff = current_time - l_t
		step_diff = current_steps - l_s
        
		# Calculate speed (in cm per second)
		velocity = (step_diff * (2 * np.pi / CPR) * (25 / 1000)) / time_diff
        
		# Update last_time and last_steps
		l_t = current_time
		l_s = current_steps
		return velocity
 
