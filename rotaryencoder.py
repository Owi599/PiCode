from gpiozero import RotaryEncoder #to creat Rotary encoder instance
import numpy as np
import time

class ReadRotaryEncoder(RotaryEncoder):
	
	def readPosition(self,CPR):
		sensorFloat = (2*np.pi/ CPR*self.steps)
		return sensorFloat

	def readVelocity(self,CPR,l_t ,l_s):
		time.sleep(0.002)			
		current_time = time.time()
		current_steps = self.steps
		# Calculate the time difference and step difference
		time_diff = current_time - l_t
		step_diff = current_steps - l_s

		# Calculate angular velocity (in radians per second)
		angular_velocity = (step_diff * (2 * np.pi / CPR)) / time_diff

		# Update last_time and last_steps
		l_t = current_time
		l_s = current_steps
		return angular_velocity
