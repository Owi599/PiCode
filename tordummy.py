import numpy as np
import time
import pigpio  # For accurate timing control (simulation only)
from mat4py import loadmat

# Physical system parameters
mass = 0.232 + 0.127 + 0.127  # Total mass (kg)
pulley_radius = 0.0125  # Pulley radius (m)
steps_per_rev = 200  # Stepper motor steps per revolution (1.8° stepper)
microstepping = 16  # Microstepping factor
total_steps_per_rev = steps_per_rev * microstepping  # Total microstepped steps
t_sample = 0.02  # Sampling time (s)
max_rpm = 2000  # Max allowable motor speed

# Initialize pigpio (Simulation only)
pi = pigpio.pi()  # Initializes pigpio library

def force_to_velocity(force, v_init):
    """ Convert force to velocity using Newton's Law and sampling time """
    acceleration = force / mass  # a = F/m
    velocity = v_init + acceleration * t_sample  # Integrate acceleration

    # Convert linear velocity to angular velocity (rad/s)
    omega = velocity / pulley_radius

    # Convert to RPM and apply limits
    speed_rpm = omega * (60 / (2 * np.pi))
    speed_rpm = np.clip(speed_rpm, -max_rpm, max_rpm)

    # Convert back to linear velocity (adjusted due to speed limits)
    velocity = speed_rpm * (pulley_radius * 2 * np.pi) / 60

    return velocity

def velocity_to_step_frequency(velocity):
    """ Convert velocity to step frequency for stepper motor """
    if velocity == 0:
        return 0  # No movement

    omega = velocity / pulley_radius  # Angular velocity (rad/s)
    step_freq = (omega / (2 * np.pi)) * total_steps_per_rev  # Convert to step frequency (Hz)
    
    return abs(step_freq)

# Load force input from MATLAB file
f = loadmat('matlab.mat')
Force = np.array(f['u'])  # Assuming 'u' is the force vector

# Initialize state variables
Velocity = 0
n = 0
max_steps = min(1501, len(Force))  # Limit simulation steps to data size

# Run velocity-based simulation
try:
    while n < max_steps:
        # Compute velocity from force
        Velocity = force_to_velocity(Force[n], Velocity)
        
        # Convert velocity to stepper motor frequency
        step_freq = velocity_to_step_frequency(Velocity)

        # Print simulation results
        print(f"Time: {n * t_sample:.2f}s | Force: {Force[n]:.3f} N | Velocity: {Velocity:.3f} m/s | Step Frequency: {step_freq:.2f} Hz")

        # Advance simulation
        n += 1
        time.sleep(t_sample)

except KeyboardInterrupt:
    print("\nSimulation stopped by user.")

finally:
    pi.stop()  # Clean up pigpio (Simulation only)
