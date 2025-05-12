import time
import numpy as np
from gpiozero import RotaryEncoder

class ReadMotorEncoder(RotaryEncoder):
	def read_position(self,cpr):
		# Calculate the angle and position
		angle = (2 * np.pi / cpr) * self.steps
		position = (angle * 25) / 1000  # m
		return position

	def read_velocity(self,cpr,l_t,l_s):
		currentTime = time.time()
		currentSteps = self.steps
        
		# Calculate the time difference and step difference
		timeDiff = currentTime - l_t
		stepDiff = currentSteps - l_s
		
		minTimeDiff = 0.001
		
		timeDiff = max(timeDiff, minTimeDiff)

        
		# Calculate speed 
		velocity = (stepDiff /timeDiff) * (2*np.pi/ cpr)  # rad/s
        
		# Update last_time and last_steps
		
		return velocity, currentTime, currentSteps
 
