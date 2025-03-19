import numpy as np
import time
from mat4py import loadmat

def calculate_steps(Force, V_init, X_init, t_sample, pulley_radius):
    """
    Calculate stepper motor movement based on LQR force input for a pulley-driven system.

    Parameters:
    - Force (float): Input force (N).
    - V_init (float): Initial velocity (m/s).
    - X_init (float): Initial position (m).
    - t_sample (float): Sampling time (s).
    - pulley_radius (float): Pulley radius (m).

    Returns:
    - steps (int): Number of steps to take.
    - step_period (float): Time period per step (s).
    - Velocity (float): Updated velocity (m/s).
    - X (float): Updated position (m).
    """
    
    # System parameters
    mass = 0.232 + 0.127 + 0.127  # Total mass (kg)
    steps_per_rev = 200  # Microstepping-enabled steps per revolution

    # Compute acceleration
    Acceleration = Force / mass  # (m/s^2)
    
    # Compute velocity
    Velocity = V_init + Acceleration * t_sample  # (m/s)
    Radial_Velocity = Velocity / pulley_radius  # (rad/s)
    Speed_RPM = Radial_Velocity * (60 / (2 * np.pi))  # Convert to RPM

    # Limit maximum speed
    Speed_RPM = np.clip(Speed_RPM, -2000, 2000)
    Velocity = Speed_RPM * (pulley_radius * 2 * np.pi) / 60  # Adjusted velocity

    # Compute position
    X = X_init + V_init * t_sample + 0.5 * Acceleration * t_sample**2

    # Convert position to steps
    steps = max(int(round((steps_per_rev / (2 * np.pi)) * (X / pulley_radius))), 1)  # At least 1 step

    # Compute step frequency and period
    step_freq = abs(Velocity) * steps_per_rev / (pulley_radius * 2 * np.pi)  # Hz
    step_period = 1 / step_freq if step_freq > 0 else 0  # Avoid division by zero

    # Ensure step period is within sample time limit
    step_period = min(step_period, t_sample)

    return steps, step_period, Velocity, X


# Load force input from MATLAB file
f = loadmat('matlab.mat')
Force = np.array(f['u'])  # Assuming 'u' is the force vector

# Initialize variables
Velocity = 0
Position = 0
t_sample = 0.02  # Sampling time (s)
pulley_radius = 0.0125  # Pulley radius in meters
n = 0
max_steps = 1501  # Maximum simulation iterations

# Run simulation
while n < max_steps:
    try:
        steps, step_period, Velocity, Position = calculate_steps(Force[n], Velocity, Position, t_sample, pulley_radius)

        print(f"Time: {n * t_sample:.2f}s | Force: {Force[n]:.3f} N | Velocity: {Velocity:.3f} m/s | "
              f"Position: {Position:.3f} m | Steps: {steps} | Step Period: {step_period:.6f} s")

        n += 1    
        time.sleep(t_sample)

    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")
        break
    except ValueError as e:
        print(f"Error: {e}")
        break
