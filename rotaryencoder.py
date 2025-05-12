from gpiozero import RotaryEncoder #to creat Rotary encoder instance
import numpy as np
import time

class ReadRotaryEncoder(RotaryEncoder):
	
	def read_position(self,cpr):
		
		sensorFloat = (2*np.pi/ cpr)*self.steps
		return sensorFloat

	def read_velocity(self,cpr,l_t ,l_s):
		
		currentTime = time.time()
		currentSteps = self.steps
		
		# Calculate the time difference and step difference
		timeDiff = currentTime - l_t
		stepDiff = currentSteps - l_s
		
		
		stepDiff = stepDiff 
		
		minTimeDiff = 0.001
		timeDiff = max(timeDiff, minTimeDiff)
		
		# Calculate angular velocity
		angularVelocity = (stepDiff / timeDiff) * (2*np.pi/cpr)  

		
		return angularVelocity, currentTime, currentSteps
