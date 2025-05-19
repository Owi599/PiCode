# Necessary imports
from rotaryencoder import ReadRotaryEncoder # Class for reading and parsing rotary encoders' data
from motorencoder import ReadMotorEncoder   # Class for reading and parsing motor encoders' data
from endswitch import EndSwitch             # Class for reading and parsing photoelectronic sensors' data
from lqr import LQR                         # Class for Linear Quadratic Regulator (LQR) control
from motorControl import MotorControl       # Class for controlling motor output
import RPi.GPIO as GPIO                     # GPIO library for Raspberry Pi 
import numpy as np                          # Numpy library for numerical operations  
import time                                 # Time library for time operations  

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Pins
switchPin_1 = 4
switchPin_2 = 3
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
m_c = 0.232              # Mass of the cart
m_1 = 0.127              # Mass of the first pendulum
m_2 = 0.127              # Mass of the second pendulum
l_1 = 0.3                # Length of the first pendulum
l_2 = 0.3                # Length of the second pendulum
lc_1 = 0.3               # Length to center of mass of the first pendulum
lc_2 = 0.15              # Length to center of mass of the second pendulum
i_1 = m_1 * lc_1**2        # Moment of inertia of the first pendulum
i_2 = m_2 * lc_2**2        # Moment of inertia of the second pendulum
g = 9.81                # Gravitational acceleration 

# i_ntermedi_ate calculati_ons for the system matrices
h_1 = m_c + m_1 + m_2
h_2 = m_1 * lc_1 + m_2 * l_1
h_3 = m_2 * lc_2
h_4 = m_2 * lc_1**2 + m_2 * l_1**2 + i_1
h_5 = m_2 * lc_2 * l_1
h_6 = m_2 * lc_2**2 + i_2
h_7 = m_1 * lc_1 * g + m_2 * l_1 * g
h_8 = m_2 * lc_2 * g

# System matrix representati_on
M = np.array([
    [1, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0],
    [0, 0, 0, h_1, h_2, h_3],
    [0, 0, 0, h_2, h_4, h_5],
    [0, 0, 0, h_3, h_5, h_6],
])
N = np.array([
    [0, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 1],
    [0, 0, 0, 0, 0, 0],
    [0, -h_7, 0, 0, 0, 0],
    [0, 0, -h_8, 0, 0, 0],
])
F = np.array([[0], [0], [0], [1], [0], [0]])

# Matrices of the state-space system
A = np.linalg.solve(M, N)
B = np.linalg.solve(M, F)
C = np.array([[1, 0, 0, 0, 0, 0]])
D = np.array([[0]])
t_s = 0.2  # Sampling time

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
ENCODER.steps = 625  # Set the steps for the rotary encoder
#END_SWITCH = EndSwitch(switchPin_1)  # End switch 1
#END_SWITCH_2 = EndSwitch(switchPin_2)  # End switch 2
MOTOR = MotorControl(pulsePin, dirPin, stepsPerRev, pulleyRad, holdingTorque)  # Motor control object
LQR_CONTROLLER = LQR(A, B, C, D, Q, R)  # LQR controller object

# Discrete-time system
sys_C, sys_D = LQR_CONTROLLER.covert_continuous_to_discrete(A, B, C, D, t_s)  # Convert continuous to discrete system
# calculate K_discrete
K_d = LQR_CONTROLLER.compute_K_discrete(Q, R, sys_D)  # Compute the LQR gain for discrete system
print('K_d:', K_d)  # Print the LQR gain

# define function to read sensor data
def read_sensors_data(lastTime,lastTime_2,lastTime_m,lastSteps, lastSteps_2,lastSteps_m):
    sensorData = []
    sensorData.append(ENCODER_M.read_position(cpr_m))  # Read motor encoder position
    sensorData.append(ENCODER.read_position(cpr))  # Read first pendulum encoder position
    sensorData.append(ENCODER_2.read_position(cpr))  # Read second pendulum encoder position
    v, lastTime_m, lastSteps_m = ENCODER_M.read_velocity(cpr_m, lastTime, lastSteps_m)  # Read motor encoder velocity
    sensorData.append(v)  # Append motor velocity to sensor data
    omega_1, lastTime, lastSteps = ENCODER.read_velocity(cpr, lastTime, lastSteps)  # Read first pendulum encoder velocity
    sensorData.append(omega_1)  # Append first pendulum velocity to sensor data
    omega_2, lastTime_2, lastSteps_2 = ENCODER_2.read_velocity(cpr, lastTime, lastSteps_2)  # Read second pendulum encoder velocity
    sensorData.append(omega_2)  # Append second pendulum velocity to sensor data
    return sensorData, lastTime,lastTime_2,lastTime_m,lastSteps, lastSteps_2,lastSteps_m  # Return the sensor data

# define call back function for the end switch
def end_switch_callback(channel,MotorControl):
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
GPIO.setup(switchPin_1, GPIO.IN)  # Set up end switch 1
GPIO.setup(switchPin_2, GPIO.IN)  # Set up end switch 2
GPIO.add_event_detect(switchPin_1, GPIO.FALLING, callback=lambda x: end_switch_callback(switchPin_1,MOTOR), bouncetime=10)  # Event detection for end switch 1
GPIO.add_event_detect(switchPin_2, GPIO.FALLING, callback=lambda x: end_switch_callback(switchPin_2,MOTOR), bouncetime=10)  # Event detection for end switch 2


# Main loop
try:
    while True:
        sensorData, lastTime,lastTime_2,lastTime_m,lastSteps, lastSteps_2,lastSteps_m = read_sensors_data(lastTime,lastTime_2,lastTime_m,lastSteps, lastSteps_2,lastSteps_m)  # Read sensor data
        print('Sensor Data:',sensorData)
        u = LQR_CONTROLLER.compute_control_output_discrete(K_d ,sensorData)  # Compute control output u
        print('Control Output:', u)
        steps, stepPeriod, velocity, position = MOTOR.calculate_steps(u[0],velocity,position, t_s)  # Calculate motor steps and period
        if np.sign(u)*1 == 1:  # Set motor direction based on control output
            MOTOR.move_stepper(steps, stepPeriod, 1)
        else:
            MOTOR.move_stepper(steps, stepPeriod, -1)
    
except KeyboardInterrupt:
    GPIO.cleanup()  

except Exception as e:
    print('An error occurred:', str(e))