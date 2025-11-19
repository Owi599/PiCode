# Updated Main-loop for pigpio-based encoder and motor control
import pigpio
import numpy as np
import time
from rotaryEncoder import ReadRotaryEncoder  # pigpio-based class for rotary encoders
from motorEncoder import ReadMotorEncoder    # pigpio-based class for motor encoder
from motorControl import MotorControl        # pigpio-based class for stepper control
from lqr import LQR                         # Controller classes as before
from poleplacement import PolePlacement

# GPIO and pigpio daemon setup
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Could not connect to pigpio daemon. Did you run 'sudo pigpiod'?")

# Assign GPIO pins (update as needed)
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

# Pendulum and cart physical parameters (unchanged)
pi_const = np.pi
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

h_1 = m_c + m_1 + m_2
h_2 = m_1 * lc_1 + m_2 * l_1
h_3 = m_2 * lc_2
h_4 = m_2 * lc_1**2 + m_2 * l_1**2 + i_1
h_5 = m_2 * lc_2 * l_1
h_6 = m_2 * lc_2**2 + i_2
h_7 = m_1 * lc_1 * g + m_2 * l_1 * g
h_8 = m_2 * lc_2 * g

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

A = np.linalg.solve(M, N)
B = np.linalg.solve(M, F)
C = np.array([[1, 0, 0, 0, 0, 0]])
D = np.array([[0]])
t_s = 0.02  # Sampling time

# Actuator/encoder configuration
stepsPerRev = 200
pulleyRad = 0.0125
holdingTorque = 2
cpr_m = 500
cpr = 5000
microstep = 4

# Use pigpio-based encoder classes
ENCODER = ReadRotaryEncoder(encoder_1_A, encoder_1_B, pi, id="lower")       # Lower pendulum encoder
ENCODER_2 = ReadRotaryEncoder(encoder_2_A, encoder_2_B, pi, id="upper")     # Upper pendulum encoder
ENCODER_M = ReadMotorEncoder(motor_encoder_A, motor_encoder_B, pi)          # Cart (motor) encoder

ENCODER.calibrate(pi_const)
ENCODER_2.calibrate(0)
ENCODER_M.calibrate()

# Use pigpio-based motor control
MOTOR = MotorControl(pi, pulsePin, dirPin, stepsPerRev, pulleyRad, microstep, t_s)
PP_CONTROLLER = PolePlacement(A, B, C, D)

desired_poles = [-6.66,-6,-5,-5.66,-7.66,-8]
K = PP_CONTROLLER.compute_poles(desired_poles)
print("K", K)
eigenvalues, dominantEigenvalue, timeConstant = PP_CONTROLLER.compute_eigenvalues_and_time_constant(K)
print('Eigenvalues:', eigenvalues)
print('Dominant Eigenvalue:', dominantEigenvalue)
print('Time Constant:', timeConstant)

# Sensor reading function updated for new API
def read_sensors_data(lastTime,lastTime_2,lastTime_m,lastSteps, lastSteps_2,lastSteps_m):
    startTime = time.perf_counter()
    sensorData = []
    sensorData.append(ENCODER_M.read_position(cpr_m))          # Cart position [m]
    sensorData.append(ENCODER.read_position(cpr))              # Lower arm angle [rad]
    sensorData.append(ENCODER_2.read_position(cpr))            # Upper arm angle [rad]
    v, lastTime_m, lastSteps_m = ENCODER_M.read_velocity(cpr_m)# Cart velocity [m/s]
    sensorData.append(v)
    omega_1, lastTime, lastSteps = ENCODER.read_velocity(cpr)  # Lower arm angular velocity [rad/s]
    sensorData.append(omega_1)
    omega_2, lastTime_2, lastSteps_2 = ENCODER_2.read_velocity(cpr)# Upper arm angular velocity [rad/s]
    sensorData.append(omega_2)
    endTime = time.perf_counter()
    sensorTime = endTime - startTime
    return sensorData, lastTime, lastTime_2, lastTime_m, lastSteps, lastSteps_2, lastSteps_m, sensorTime

# END SWITCHES: Convert to pigpio-based (example for one switch)
def end_switch_callback(gpio, level, tick):
    MOTOR.stop_motor()
    print(f"End switch triggered on pin {gpio}. Motor stopped.")

# Set up end switches with pigpio interrupts (using pull-ups if normally open)
for switch_pin in [switchPin_1, switchPin_2]:
    pi.set_mode(switch_pin, pigpio.INPUT)
    pi.set_pull_up_down(switch_pin, pigpio.PUD_UP)
    pi.callback(switch_pin, pigpio.FALLING_EDGE, end_switch_callback)

# Initial time/step snapshots
lastTime = time.time()
lastTime_2 = time.time()
lastTime_m = time.time()
lastSteps = ENCODER.steps
lastSteps_2 = ENCODER_2.steps
lastSteps_m = ENCODER_M.steps

# Data arrays
sensorTimeArray = []
stepCalculationTimeArray = []
movementTimeArray = []

# Main loop
try:
    while True:
        try:
            # 1. Sensor Reading
            sensorData, lastTime, lastTime_2, lastTime_m, lastSteps, lastSteps_2, lastSteps_m, sensorTime = read_sensors_data(
                lastTime, lastTime_2, lastTime_m, lastSteps, lastSteps_2, lastSteps_m)
            print('Sensor Data:', sensorData)

            # 2. Control Law
            u = - K @ sensorData
            print('Control Output:', u)

            # 3. Compute Steps and Frequency (updated for pigpio)
            steps, freq, direction = MOTOR.calculate_steps(float(u))
            print('steps:', steps, 'step freq:', freq)

            # 4. Move Motor (hardware waveform, non-blocking)
            startMovement = time.perf_counter()
            MOTOR.move_stepper(steps, freq, np.sign(direction)*1)
            # Wait for move to end or sample timeout
            move_timeout = t_s
            time_waited = 0
            while MOTOR.is_moving():
                time.sleep(0.001)
                time_waited += 0.001
                if time_waited > move_timeout:
                    print("Move timeout - forcibly stopping motor.")
                    MOTOR.stop_motor()
                    break
            movementTime = time.perf_counter() - startMovement

            # 5. Store timing data
            sensorTimeArray.append(sensorTime)
            stepCalculationTimeArray.append(move_timeout)
            movementTimeArray.append(movementTime)

        except KeyboardInterrupt:
            print("User interrupted - exiting safely.")
            break

        except Exception as e:
            print('An error occurred:', str(e))
            break

finally:
    # Save data and cleanup, always runs when loop exits
    np.array(sensorTimeArray).tofile('sensorTimeArray.csv', sep=',')
    np.array(stepCalculationTimeArray).tofile('stepCalculationTimeArray.csv', sep=',')
    np.array(movementTimeArray).tofile('movementTimeArray.csv', sep=',')
    ENCODER.cleanup()
    ENCODER_2.cleanup()
    ENCODER_M.cleanup()
    MOTOR.stop_motor()
    pi.stop()
    print('All resources cleaned up and data saved.')
