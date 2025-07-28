# Necessary imports
from rotaryencoder import ReadRotaryEncoder # Class for reading and parsing rotary encoders' data
from motorencoder import ReadMotorEncoder   # Class for reading and parsing motor encoders' data
from endswitch import EndSwitch             # Class for reading and parsing photoelectronic sensors' data
from lqr import LQR                         # Class for Linear Quadratic Regulator (LQR) control
from poleplacement import PolePlacement     # Class for Pole Placement control
from motorControl import MotorControl       # Class for controlling motor output
import RPi.GPIO as GPIO                     # GPIO library for Raspberry Pi 
import numpy as np                          # Numpy library for numerical operations  
import time                                 # Time library for time operations  
import timeout_decorator                    # Timeout decorator for function execution time limits
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
pi = np.pi
m_c = 0.6
m_1 = 0.102
m_2 = 0.104
l_1 = 0.28
l_2 = 0.305
lc_1 = 0.17
lc_2 = 0.065
i_1 = m_1 * lc_1**2
i_2 = m_2 * lc_2**2
g = 9.81
b_c = 0.05
b_1 = 0.01
b_2 = 0.01

# Intermediate calculations for the system matrices
h_1 = m_c + m_1 + m_2
h_2 = m_1 * lc_1 + m_2 * l_1
h_3 = m_2 * lc_2
h_4 = m_2 * lc_1**2 + m_2 * l_1**2 + i_1
h_5 = m_2 * lc_2 * l_1
h_6 = m_2 * lc_2**2 + i_2
h_7 = m_1 * lc_1 * g + m_2 * l_1 * g
h_8 = m_2 * lc_2 * g

# System matrix representation
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
    [0, 0, 0, -b_c, 0, 0],
    [0, -h_7, 0, 0, -b_1, 0],
    [0, 0, -h_8, 0, 0, -b_2],
])
F = np.array([[0], [0], [0], [1], [0], [0]])

# Matrices of the state-space system
A = np.linalg.solve(M, N)
B = np.linalg.solve(M, F)
C = np.array([[1, 0, 0, 0, 0, 0]])
D = np.array([[0]])
t_s = 0.02  # Sampling time

# LQR Parameters
# Q = np.diag([1000, 50, 50, 1000, 10, 10])  # State cost matrix
# R = np.array([[100]])  # Control cost matrix

# Other Constants
stepsPerRev = 200  # Steps per revolution for the motor
pulleyRad = 0.0125  # Radius of the pulley
holdingTorque = 2  # Holding torque of the motor
cpr_m = 500  # Counts per revolution for the motor encoder
maxSteps = 625  # Maximum steps for the rotary encoder
cpr = 1250  # Counts per revolution for the rotary encoder
microstep = 4
# Instantiating objects
ENCODER = ReadRotaryEncoder(encoder_1_A, encoder_1_B, max_steps=maxSteps, wrap=True)  # Rotary encoder for the pendulum
ENCODER_2 = ReadRotaryEncoder(encoder_2_A, encoder_2_B, max_steps=maxSteps, wrap=True)  # Rotary encoder for the second pendulum
ENCODER_M = ReadMotorEncoder(motor_encoder_A, motor_encoder_B, max_steps=0)  # Motor encoder
ENCODER.steps = 625  # Set the steps for the rotary encoder
#END_SWITCH = EndSwitch(switchPin_1)  # End switch 1
#END_SWITCH_2 = EndSwitch(switchPin_2)  # End switch 2
MOTOR = MotorControl(pulsePin, dirPin, stepsPerRev, pulleyRad, holdingTorque, t_s)  # Motor control object
# LQR_CONTROLLER = LQR(A, B, C, D, Q, R)  # LQR controller object
PP_CONTROLLER = PolePlacement(A, B, C, D)  # Pole placement controller object
# Discrete-time system
# sys_C, sys_D = LQR_CONTROLLER.covert_continuous_to_discrete(A, B, C, D, t_s)  # Convert continuous to discrete system
# # calculate K_discrete
# K_d = LQR_CONTROLLER.compute_K_discrete(Q, R, sys_D)  # Compute the LQR gain for discrete system
# print('K_d:', K_d)  # Print the LQR gain
# print('Time Constant:',LQR_CONTROLLER.compute_eigenvalues_discrete(Q,R,sys_D,K_d))  # Print the time constant of the system
desired_poles = [-6.66,-6,-5,-5.66,-7.66,-8]
K = PP_CONTROLLER.compute_poles(desired_poles)  # Compute the state feedback gain matrix K

eigenvalues, dominantEigenvalue, timeConstant = PP_CONTROLLER.compute_eigenvalues_and_time_constant(K)  # Compute eigenvalues and time constant
print('Eigenvalues:', eigenvalues)  # Print the eigenvalues of the closed-loop system
print('Dominant Eigenvalue:', dominantEigenvalue)  # Print the dominant eigenvalue
print('Time Constant:', timeConstant)  # Print the time constant of the closed-loop system


# define function to read sensor data
def read_sensors_data(lastTime,lastTime_2,lastTime_m,lastSteps, lastSteps_2,lastSteps_m):
    startTime = time.perf_counter()  # Start time for reading sensor data
    sensorData = []
    sensorData.append(ENCODER_M.read_position(cpr_m,microstep))  # Read motor encoder position
    sensorData.append(ENCODER.read_position(cpr))  # Read first pendulum encoder position
    sensorData.append(ENCODER_2.read_position(cpr))  # Read second pendulum encoder position
    v, lastTime_m, lastSteps_m = ENCODER_M.read_velocity(cpr_m, lastTime, lastSteps_m)  # Read motor encoder velocity
    sensorData.append(v)  # Append motor velocity to sensor data
    omega_1, lastTime, lastSteps = ENCODER.read_velocity(cpr, lastTime, lastSteps)  # Read first pendulum encoder velocity
    sensorData.append(omega_1)  # Append first pendulum velocity to sensor data
    omega_2, lastTime_2, lastSteps_2 = ENCODER_2.read_velocity(cpr, lastTime, lastSteps_2)  # Read second pendulum encoder velocity
    sensorData.append(omega_2)  # Append second pendulum velocity to sensor data
    endTime = time.perf_counter()  # End time for reading sensor data
    sensorTime = endTime - startTime  # Calculate sensor reading time
    return sensorData, lastTime,lastTime_2,lastTime_m,lastSteps, lastSteps_2,lastSteps_m,sensorTime  # Return the sensor data

# define call back function for the end switch
def end_switch_callback(channel,MotorControl):
    MotorControl.stop_motor()  # stop the motor
    
# define event interrupt
GPIO.setup(switchPin_1, GPIO.IN)  # Set up end switch 1
GPIO.setup(switchPin_2, GPIO.IN)  # Set up end switch 2
GPIO.add_event_detect(switchPin_1, GPIO.FALLING, callback=lambda x: end_switch_callback(switchPin_1,MOTOR), bouncetime=10)  # Event detection for end switch 1
GPIO.add_event_detect(switchPin_2, GPIO.FALLING, callback=lambda x: end_switch_callback(switchPin_2,MOTOR), bouncetime=10)  # Event detection for end switch 2

# def home_cart(MotorControl, ReadMotorEncoder):
#     while True:
#         # Move the motor until the end switch is triggered
#         MotorControl.move_stepper(100, 0.01, -1)  # Move the motor in reverse direction
#         time.sleep(0.01)  # Sleep for a short duration to avoid excessive CPU usage
#         if GPIO.input(switchPin_1) == 0 or GPIO.input(switchPin_2) == 0:
#             # If the end switch is triggered, stop the motor
#             MotorControl.stop_motor()
#             ReadMotorEncoder.steps = 0  # Reset the motor encoder steps
#             MotorControl.move_stepper(100,0.01,1)  # Reset the motor position
#             break
#     while GPIO.input(switchPin_1) == 1 or GPIO.input(switchPin_2) == 1:
#         MotorControl.move_stepper(100, 0.01, 1)  # Move the motor in direction until the end switch is triggered
#         time.sleep(0.01)  # Sleep for a short duration to avoid excessive CPU usage
#         if GPIO.input(switchPin_1) == 0 or GPIO.input(switchPin_2) == 0:
#             MotorControl.stop_motor()  # Stop the motor if the end switch is triggered
#             MotorControl.move_stepper(ReadMotorEncoder.steps/2, 0.01, -1)  # Move the motor in reverse direction to reset the position
#             ReadMotorEncoder.steps = 0  # Reset the motor encoder steps
#             break
# home_cart(MOTOR, ENCODER_M)  # Home the cart by moving the motor until the end switch is triggered

# time Costants
lastTime = time.time()  # Last time for the first pendulum
lastTime_2 = time.time()  # Last time for the second pendulum
lastTime_m = time.time()  # Last time for the motor
lastSteps = ENCODER.steps  # Last steps for the first pendulum
lastSteps_2 = ENCODER_2.steps  # Last steps for the second pendulum
lastSteps_m = ENCODER_M.steps  # Last steps for the motor



sensorTimeArray = []  # Array to store sensor data reading times
controlOutputCalculationTimeArray = []  # Array to store control calculation times
stepCalculationTimeArray = []  # Array to store step calculation times
movementTimeArray = []  # Array to store movement times
# Main loop
while True:
    try:
        sensorData, lastTime,lastTime_2,lastTime_m,lastSteps, lastSteps_2,lastSteps_m,sensorTime = read_sensors_data(lastTime,lastTime_2,lastTime_m,lastSteps, lastSteps_2,lastSteps_m)  # Read sensor data
        print('Sensor Data:',sensorData)
        # u, controlOutputCalculationTime = PP_CONTROLLER.compute_contorller_output(K, sensorData)  # Compute control output u
        u = -K@sensorData  # Compute control output u using pole placement
        print(type(u))
        #  #LQR_CONTROLLER.compute_control_output_discrete(K_d ,sensorData)  
        #  Compute control output u
        print('Control Output:', u[0])

        steps, stepPeriod,stepCalculationTime,direction = MOTOR.calculate_steps(u[0])  # Calculate motor steps and period
        print('Steps:', steps, 'Step Period:', stepPeriod, 'Step Calculation Time:', stepCalculationTime,type(steps),type(stepPeriod),type(stepCalculationTime))  # Print calculated steps and period
        print(type(MOTOR.move_stepper))
        if direction == 1:
            MOTOR.move_stepper(steps, stepPeriod, 1)  # Move the motor in the direction of control output
        elif direction == -1:
            MOTOR.move_stepper(steps, stepPeriod, -1)  # Move the motor in the opposite direction of control output
        else:
            movementTime = 0  # If control output is zero, no movement is made
        print('Movement Time:', movementTime)  # Print the time taken for the motor movement
        sensorTimeArray.append(sensorTime)
        # controlOutputCalculationTimeArray.append(controlOutputCalculationTime)
        stepCalculationTimeArray.append(stepCalculationTime)
        movementTimeArray.append(movementTime)

    except TimeoutError:
        print("Function timed out. loop starting over.")
        continue 

    except KeyboardInterrupt:
        GPIO.cleanup()
        np.array(sensorTimeArray).tofile('sensorTimeArray.csv', sep=',')
        np.array(controlOutputCalculationTimeArray).tofile('controlOutputCalculationTimeArray.csv', sep=',')
        np.array(stepCalculationTimeArray).tofile('stepCalculationTimeArray.csv', sep=',')
        np.array(movementTimeArray).tofile('movementTimeArray.csv', sep=',')
        print('Program terminated by user. Data saved to CSV files.')  
        break

    except Exception as e:
        print('An error occurred:', str(e))
        GPIO.cleanup()
        np.array(sensorTimeArray).tofile('sensorTimeArray.csv', sep=',')
        np.array(controlOutputCalculationTimeArray).tofile('controlOutputCalculationTimeArray.csv', sep=',')
        np.array(stepCalculationTimeArray).tofile('stepCalculationTimeArray.csv', sep=',')
        np.array(movementTimeArray).tofile('movementTimeArray.csv', sep=',')
        print('Data saved to CSV files.')
        print('Program terminated due to an error.')
        break
    # finally:
    #     GPIO.cleanup()
    #     np.array(sensorTimeArray).tofile('sensorTimeArray.csv', sep=',')
    #     np.array(controlOutputCalculationTimeArray).tofile('controlOutputCalculationTimeArray.csv', sep=',')
    #     np.array(stepCalculationTimeArray).tofile('stepCalculationTimeArray.csv', sep=',')
    #     np.array(movementTimeArray).tofile('movementTimeArray.csv', sep=',')
    #     print('Program terminated. Data saved to CSV files.')
    #     break