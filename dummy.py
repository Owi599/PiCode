import numpy as np
import random
import time

def calculate_steps(Force,V_intit,X_init,step_init,t_sample):
        
        #Velocity and Speed in RPM
        Acceleration = (Force)/(.232+.127+.127) 
        Velocity = Acceleration*t_sample + V_intit
        Radial_Velocity = Velocity * 0.0125
        Speed_RPM = Radial_Velocity * 9.549297
        
        #Check for maximum speed
        if abs(Speed_RPM) >2000:
            print(Speed_RPM)
            Speed_RPM = np.sign(Speed_RPM)*2000 + (-np.sign(Speed_RPM)*1)  
            Velocity = Speed_RPM * 0.01047198
            print('Warning:Speed cannot be greater than 2000 rpm')
        
        #caclcuate the positison
        X = 0.5*Acceleration*t_sample**2 + V_intit*t_sample + X_init
        
        steps_int = max(int(round(((40*500)/2*np.pi)*abs(X))),1) - step_init # Ensures at least 1 step
        step_freq = (abs(Velocity) * 3200) / (0.0125/2*np.pi)  # Frequency in Hz
        
        if step_freq == 0:
            step_period = 0  # Motor is not moving
        else:
            step_period = 1 / step_freq
        
        # # Set maximum delay to prevent out-of-bound values
        MAX_DELAY = t_sample   # maximum delay
        step_period = min(step_period, MAX_DELAY)
		
        
        return steps_int, step_period, Velocity, X


Velocity = 0
Position = 0
steps = 0
t_sample = 0.02
t = 0
n= 0
while n <= 15:
    try:
        Force = [7.1184,5.3633,3.6816,2.0938,0.6177,-0.7314,-1.9410,-3.0014,-3.9057,-4.6494,-5.2310,-5.6513,-5.9136,-6.0235,-5.9884,-5.8179]
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