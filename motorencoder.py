import time
import numpy as np
from gpiozero import RotaryEncoder

class ReadMotorEncoder(RotaryEncoder):
	def read_position(self,cpr,microstep):
		# Calculate the angle and position
		distancePerStep = (2 * np.pi * 0.0125) /   cpr # m
		position = (distancePerStep * self.steps) # m
		return position

	def read_velocity(self,cpr,l_t,l_s):
		currentTime = time.time()
		currentSteps = self.steps
        
		# Calculate the time difference and step difference
		timeDiff = currentTime - l_t
		stepDiff = currentSteps - l_s
		
		minTimeDiff = 0.001
		
		timeDiff = max(timeDiff, minTimeDiff)

        
		# Calculate velocity
		distancePerStep = (2 * np.pi * 0.0125) /  cpr  # m 
		velocity = (stepDiff /timeDiff) * distancePerStep  # m/s
        
		# Update last_time and last_steps
		
		return velocity, currentTime, currentSteps
 
