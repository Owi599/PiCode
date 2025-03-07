from gpiozero import RotaryEncoder #to creat Rotary encoder instance
import numpy as np
import time

class ReadRotaryEncoder(RotaryEncoder):
	
	def readPosition(self,CPR):
		
		sensorFloat = (2*np.pi/ CPR)*self.steps
		return sensorFloat

	def readVelocity(self,CPR,l_t ,l_s):
		
		current_time = time.time()
		current_steps = self.steps
		
		# Calculate the time difference and step difference
		time_diff = current_time - l_t
		step_diff = current_steps - l_s
		
		max_steps = 625
		step_diff = step_diff 
		
		min_time_diff = 0.001
		time_diff = max(time_diff, min_time_diff)
		
		# Calculate angular velocity
		angular_velocity = (step_diff / time_diff) * (2*np.pi/CPR)  

		
		return angular_velocity, current_time, current_steps
