# Necessary imports
from rotaryencoder import ReadRotaryEncoder # Class for reading and parsing rotary encoders' data
from motorencoder import ReadMotorEncoder   # Class for reading and parsing motor encoders' data
from endswitch import EndSwitch   # Class for reading and parsing photoelectronic sensors' data
from lqr import LQR                         # Class for Linear Quadratic Regulator (LQR) control
from motorControl import MotorControl            # Class for controlling motor output
import RPi.GPIO as GPIO                     # GPIO library for Raspberry Pi 
import numpy as np                          # Numpy library for numerical operations  
import time                                 # Time library for time operations  

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Pins
switchPin_1 = 4
switchPin_2 = 17
pulsePin = 9
dirPin = 10
encoder_1_A = 21
encoder_1_B = 20
encoder_2_A = 16
encoder_2_B = 12
motor_encoder_A = 7
motor_encoder_B = 8

# Pendulum Parameters
pi = np.pi              # Constant for pi
mc = 0.232              # Mass of the cart
m1 = 0.127              # Mass of the first pendulum
m2 = 0.127              # Mass of the second pendulum
L1 = 0.3                # Length of the first pendulum
L2 = 0.3                # Length of the second pendulum
LC1 = 0.3               # Length to center of mass of the first pendulum
LC2 = 0.15              # Length to center of mass of the second pendulum
I1 = m1 * LC1**2        # Moment of inertia of the first pendulum
I2 = m2 * LC2**2        # Moment of inertia of the second pendulum
g = 9.81                # Gravitational acceleration 

# Intermediate calculations for the system matrices
h1 = mc + m1 + m2
h2 = m1 * LC1 + m2 * L1
h3 = m2 * LC2
h4 = m2 * LC1**2 + m2 * L1**2 + I1
h5 = m2 * LC2 * L1
h6 = m2 * LC2**2 + I2
h7 = m1 * LC1 * g + m2 * L1 * g
h8 = m2 * LC2 * g

# System matrix representation
M = np.array([
    [1, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0],
    [0, 0, 0, h1, h2, h3],
    [0, 0, 0, h2, h4, h5],
    [0, 0, 0, h3, h5, h6],
])
N = np.array([
    [0, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 1],
    [0, 0, 0, 0, 0, 0],
    [0, -h7, 0, 0, 0, 0],
    [0, 0, -h8, 0, 0, 0],
])
F = np.array([[0], [0], [0], [1], [0], [0]])

# Matrices of the state-space system
A = np.linalg.solve(M, N)
B = np.linalg.solve(M, F)
C = np.array([[1, 0, 0, 0, 0, 0]])
D = np.array([[0]])
t_s = 0.02  # Sampling time

# LQR Parameters
Q = np.diag([4000, 50, 50, 100, 10, 10])  # State cost matrix
R = np.array([[100]])  # Control cost matrix

# Other Constants
stepsPerRev = 200  # Steps per revolution for the motor
pulleyRad = 0.0125  # Radius of the pulley
holdingTorque = 2  # Holding torque of the motor
cpr_m = 500  # Counts per revolution for the motor encoder
maxSteps = 625  # Maximum steps for the rotary encoder
cpr = 1250  # Counts per revolution for the rotary encoder

# Instantiating objects
ENCODER = ReadRotaryEncoder(encoder_1_A, encoder_1_B, max_steps=maxSteps, wrap=True)  # Rotary encoder for the pendulum
ENCODER_2 = ReadRotaryEncoder(encoder_2_A, encoder_2_B, max_steps=maxSteps, wrap=True)  # Rotary encoder for the second pendulum
ENCODER_M = ReadMotorEncoder(motor_encoder_A, motor_encoder_B, max_steps=0)  # Motor encoder
#END_SWITCH = EndSwitch(switchPin_1)  # End switch 1
#END_SWITCH_2 = EndSwitch(switchPin_2)  # End switch 2
MOTOR = MotorControl(pulsePin, dirPin, stepsPerRev, pulleyRad, holdingTorque)  # Motor control object
LQR_CONTROLLER = LQR(A, B, C, D, Q, R)  # LQR controller object

# Discrete-time system
sys_C, sys_D = LQR_CONTROLLER.covert_continuous_to_discrete(A, B, C, D, t_s)  # Convert continuous to discrete system
# calculate K_discrete
K_d = LQR_CONTROLLER.compute_K_discrete(Q, R, sys_D)  # Compute the LQR gain for discrete system


# define function to read sensor data
def read_sensors_data(lastTime,lastTime_2,lastTime_m,lastSteps, lastSteps_2,lastSteps_m):
    sensorData = []
    sensorData.append(str("{:.2f}".format(ENCODER_M.read_position(cpr_m))))  # Read motor encoder position
    sensorData.append(str("{:.2f}".format(ENCODER.read_position(cpr))))  # Read first pendulum encoder position
    sensorData.append(str("{:.2f}".format(ENCODER_2.read_position(cpr))))  # Read second pendulum encoder position
    v, lastTime_m, lastSteps_m = ENCODER_M.read_velocity(cpr_m, lastTime, lastSteps_m)  # Read motor encoder velocity
    sensorData.append(str("{:.2f}".format(v)))  # Append motor velocity to sensor data
    omega_1, lastTime, lastSteps = ENCODER.read_velocity(cpr, lastTime, lastSteps)  # Read first pendulum encoder velocity
    sensorData.append(str("{:.2f}".format(omega_1)))  # Append first pendulum velocity to sensor data
    omega_2, lastTime_2, lastSteps_2 = ENCODER_2.read_velocity(cpr, lastTime, lastSteps_2)  # Read second pendulum encoder velocity
    sensorData.append(str("{:.2f}".format(omega_2)))  # Append second pendulum velocity to sensor data
    return sensorData, lastTime,lastTime_2,lastTime_m,lastSteps, lastSteps_2,lastSteps_m  # Return the sensor data

# define call back function for the end switch
def end_switch_callback(channel):
    MotorControl.stop_motor()  # stop the motor
    
# time Costants
lastTime = time.time()  # Last time for the first pendulum
lastTime_2 = time.time()  # Last time for the second pendulum
lastTime_m = time.time()  # Last time for the motor
lastSteps = ENCODER.steps  # Last steps for the first pendulum
lastSteps_2 = ENCODER_2.steps  # Last steps for the second pendulum
lastSteps_m = ENCODER_M.steps  # Last steps for the motor
velocity = 0  # Initial velocity
position = 0  # Initial position

# define event interrupt
GPIO.add_event_detect(switchPin_1, GPIO.FALLING, callback=end_switch_callback, bouncetime=0)  # Event detection for end switch 1
GPIO.add_event_detect(switchPin_2, GPIO.FALLING, callback=end_switch_callback, bouncetime=0)  # Event detection for end switch 2


# Main loop
try:
    while True:
        sensorData, lastTime,lastTime_2,lastTime_m,lastSteps, lastSteps_2,lastSteps_m = read_sensors_data(lastTime,lastTime_2,lastTime_m,lastSteps, lastSteps_2,lastSteps_m)  # Read sensor data
        u = LQR_CONTROLLER.compute_control_output(K_d ,sensorData)  # Compute control output u
        steps, stepPeriod, velocity, position = MOTOR.calculate_steps(u,velocity,position, t_s)  # Calculate motor steps and period
        if np.sign(u)*1 == 1:  # Set motor direction based on control output
            MOTOR.move_stepper(steps, stepPeriod, 1)
        else:
            MOTOR.move_stepper(steps, stepPeriod, -1)
    
except KeyboardInterrupt:
    GPIO.cleanup()  

except Exception as e:
    print('An error occurred:', str(e))