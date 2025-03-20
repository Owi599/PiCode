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
inertia = 0.5 * mass * pulley_radius**2  # Moment of inertia for a disk (J = 1/2 m r^2)

# Initialize pigpio (Simulation only)
pi = pigpio.pi()  # Initializes pigpio library

def force_to_angular_velocity(force, omega_init):
    """ Convert force to angular velocity using torque and moment of inertia """
    torque = force * pulley_radius  # τ = F * r
    angular_acceleration = torque / inertia  # α = τ / I
    omega = omega_init + angular_acceleration * t_sample  # Integrate angular acceleration

    # Convert to RPM and apply limits
    speed_rpm = omega * (60 / (2 * np.pi))
    speed_rpm = np.clip(speed_rpm, -max_rpm, max_rpm)

    # Convert back to angular velocity (adjusted)
    omega = speed_rpm * (2 * np.pi) / 60

    return omega

def angular_velocity_to_step_frequency(omega):
    """ Convert angular velocity to step frequency for stepper motor """
    if omega == 0:
        return 0, 0  # No movement

    step_freq = (omega / (2 * np.pi)) * total_steps_per_rev  # Convert to step frequency (Hz)

    # Calculate the number of steps in the given time sample
    steps = int(round(step_freq * t_sample))  # Steps in this interval

    return abs(step_freq), abs(steps)

# Load force input from MATLAB file
f = loadmat('matlab.mat')
Force = np.array(f['u'])  # Assuming 'u' is the force vector

# Initialize state variables
Omega = 0  # Initial angular velocity
n = 0
max_steps = min(1501, len(Force))  # Limit simulation steps to data size

# Run torque-based simulation
try:
    while n < max_steps:
        # Compute angular velocity from force (via torque)
        Omega = force_to_angular_velocity(Force[n], Omega)
        
        # Convert angular velocity to step frequency and compute steps
        step_freq, steps = angular_velocity_to_step_frequency(Omega)

        # Print simulation results
        print(f"Time: {n * t_sample:.2f}s | Force: {Force[n]:.3f} N | Angular Velocity: {Omega:.3f} rad/s | "
              f"Step Frequency: {step_freq:.2f} Hz | Steps: {steps}")

        # Advance simulation
        n += 1
        time.sleep(t_sample)

except KeyboardInterrupt:
    print("\nSimulation stopped by user.")

finally:
    pi.stop()  # Clean up pigpio (Simulation only)
