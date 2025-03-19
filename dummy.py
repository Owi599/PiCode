import numpy as np
import random
import time
from mat4py import loadmat

def calculate_steps(Force,V_intit,X_init,step_init,t_sample):
        #Velocity and Speed in RPM
        Acceleration = (Force)/(.232+.127+.127)  # Force/Mass = Acceleration in m/s^2
        Velocity = Acceleration*t_sample + V_intit # Velocity in m/s
        Radial_Velocity = Velocity * 0.0125 # Radial Velocity in rad/s
        Speed_RPM = Radial_Velocity * 9.549297 # Speed in RPM
        
        #Check for maximum speed
        if abs(Speed_RPM) >2000:
            print(Speed_RPM)
            Speed_RPM = np.sign(Speed_RPM)*2000 + (-np.sign(Speed_RPM)*1)  
            Velocity = Speed_RPM * 0.01047198
            print('Warning:Speed cannot be greater than 2000 rpm')
        
        #caclcuate the positison
        X = 0.5*Acceleration*t_sample**2 + V_intit*t_sample + X_init
        
        steps_int = max(int(round(((40*500)/(0.0625*2*np.pi))*(X))),1)  # Ensures at least 1 step
        step_freq = (abs(Velocity) * 3200) / (0.0125/2*np.pi)  # Frequency in Hz
        
        if step_freq == 0:
            step_period = 0  # Motor is not moving
        else:
            step_period = 1 / step_freq
        
        # # Set maximum delay to prevent out-of-bound values
        MAX_DELAY = t_sample   # maximum delay
        step_period = min(step_period, MAX_DELAY) # Ensures that the period is not greater than the sample time
		
        
        return steps_int, step_period  ,Velocity, X


f = loadmat('matlab.mat')
#print(f)
Force = np.array(f['u'])

Velocity = 0
Position = 0
steps = 0
t_sample = 0.02
t = 0
n= 0
while n <= 1000:
    try:
        steps, step_period,Velocity, Position = calculate_steps(Force[n],Velocity,Position,steps,t_sample)
        print('u: ',Force[n])
        print('Velocity: ',Velocity)
        print('Position: ',Position)
        print('Steps: ',steps)
        print('Step Period: ',step_period)
        n += 1    
        time.sleep(t_sample)
        t += t_sample
        print(t)
    except KeyboardInterrupt:
        break
    except ValueError as e:
        print(e)
        break