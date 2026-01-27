# Updated Main-loop for pigpio-based encoder and motor control
import pigpio
import numpy as np
import time
from rotaryEncoder import ReadRotaryEncoder   # pigpio-based class for rotary encoders
from motorEncoder import ReadMotorEncoder     # pigpio-based class for motor encoder
from motorControl import MotorControl         # pigpio-based class for stepper control
from lqr import LQR                           # Controller classes as before
from poleplacement import PolePlacement


# ---------------------------------------------------------------------------
# GPIO and pigpio daemon setup
# ---------------------------------------------------------------------------
pi = pigpio.pi()
if not pi.connected:
    # Ensure pigpiod is running before executing this script
    raise RuntimeError("Could not connect to pigpio daemon. Did you run 'sudo pigpiod'?")

# Assign GPIO pins (update as needed for your hardware wiring)
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


# ---------------------------------------------------------------------------
# Pendulum and cart physical parameters
# (double inverted pendulum on a cart – same as in previous models)
# ---------------------------------------------------------------------------
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

# Intermediate terms for compact matrix representation
h_1 = m_c + m_1 + m_2
h_2 = m_1 * lc_1 + m_2 * l_1
h_3 = m_2 * lc_2
h_4 = m_2 * lc_1**2 + m_2 * l_1**2 + i_1
h_5 = m_2 * lc_2 * l_1
h_6 = m_2 * lc_2**2 + i_2
h_7 = m_1 * lc_1 * g + m_2 * l_1 * g
h_8 = m_2 * lc_2 * g

# ---------------------------------------------------------------------------
# Linearized system matrices:  M x_dot = N x + F u  →  x_dot = A x + B u
# State: [x, theta1, theta2, x_dot, theta1_dot, theta2_dot]^T
# ---------------------------------------------------------------------------
M = np.array([
    [1, 0, 0, 0,   0,   0],
    [0, 1, 0, 0,   0,   0],
    [0, 0, 1, 0,   0,   0],
    [0, 0, 0, h_1, h_2, h_3],
    [0, 0, 0, h_2, h_4, h_5],
    [0, 0, 0, h_3, h_5, h_6],
])
N = np.array([
    [0,  0,   0,   1,    0,    0],
    [0,  0,   0,   0,    1,    0],
    [0,  0,   0,   0,    0,    1],
    [0,  0,   0,  -b_c,  0,    0],
    [0, -h_7, 0,   0,   -b_1,  0],
    [0,  0,  -h_8, 0,    0,   -b_2],
])
F = np.array([[0], [0], [0], [1], [0], [0]])  # Input force on cart

# Continuous-time state-space model
A = np.linalg.solve(M, N)
B = np.linalg.solve(M, F)
C = np.array([[1, 0, 0, 0, 0, 0]])            # Measure cart position
D = np.array([[0]])
t_s = 0.02                                    # Sampling time [s]


# ---------------------------------------------------------------------------
# Actuator/encoder configuration
# ---------------------------------------------------------------------------
stepsPerRev = 200      # Full steps per revolution for the stepper
pulleyRad = 0.0125     # Pulley radius [m]
holdingTorque = 2      # Not used directly here (could be for checks)
cpr_m = 500            # Motor encoder counts per rev
cpr = 5000             # Pendulum encoder counts per rev
microstep = 4          # Microstepping factor


# ---------------------------------------------------------------------------
# Instantiate pigpio-based encoder and motor control objects
# ---------------------------------------------------------------------------
ENCODER = ReadRotaryEncoder(encoder_1_A, encoder_1_B, pi, id="lower")   # Lower pendulum
ENCODER_2 = ReadRotaryEncoder(encoder_2_A, encoder_2_B, pi, id="upper") # Upper pendulum
ENCODER_M = ReadMotorEncoder(motor_encoder_A, motor_encoder_B, pi)      # Cart encoder

# Calibrate encoders (e.g. set zero positions / reference orientations)
ENCODER.calibrate(pi_const)   # Lower arm, reference at pi rad (inverted)
ENCODER_2.calibrate(0)        # Upper arm, reference at 0 rad
ENCODER_M.calibrate()         # Cart encoder zeroing


# Motor driver wrapper (uses pigpio waveforms for step generation)
MOTOR = MotorControl(pi, pulsePin, dirPin, stepsPerRev, pulleyRad, microstep, t_s)

# Pole-placement controller instance
PP_CONTROLLER = PolePlacement(A, B, C, D)

desired_poles = [-6.66, -6, -5, -5.66, -7.66, -8]   # Desired closed-loop poles
K = PP_CONTROLLER.compute_poles(desired_poles)
print("K", K)

# Analyze closed-loop dynamics
eigenvalues, dominantEigenvalue, timeConstant = PP_CONTROLLER.compute_eigenvalues_and_time_constant(K)
print('Eigenvalues:', eigenvalues)
print('Dominant Eigenvalue:', dominantEigenvalue)
print('Time Constant:', timeConstant)


# ---------------------------------------------------------------------------
# Sensor reading function for pigpio-based encoders
# ---------------------------------------------------------------------------
def read_sensors_data(lastTime, lastTime_2, lastTime_m,
                      lastSteps, lastSteps_2, lastSteps_m):
    """
    Read all sensor channels and compute velocities.

    Args:
        lastTime, lastTime_2, lastTime_m (float): Previous timestamps for each encoder.
        lastSteps, lastSteps_2, lastSteps_m (int): Previous step counts for each encoder.

    Returns:
        tuple:
            sensorData (list): [x, theta1, theta2, x_dot, theta1_dot, theta2_dot].
            updated time/step variables for next call.
            sensorTime (float): Time spent in this read function.
    """
    startTime = time.perf_counter()
    sensorData = []

    # Cart position [m]
    sensorData.append(ENCODER_M.read_position(cpr_m))
    # Lower arm angle [rad]
    sensorData.append(ENCODER.read_position(cpr))
    # Upper arm angle [rad]
    sensorData.append(ENCODER_2.read_position(cpr))

    # Cart velocity [m/s]
    v, lastTime_m, lastSteps_m = ENCODER_M.read_velocity(cpr_m)
    sensorData.append(v)

    # Lower arm angular velocity [rad/s]
    omega_1, lastTime, lastSteps = ENCODER.read_velocity(cpr)
    sensorData.append(omega_1)

    # Upper arm angular velocity [rad/s]
    omega_2, lastTime_2, lastSteps_2 = ENCODER_2.read_velocity(cpr)
    sensorData.append(omega_2)

    endTime = time.perf_counter()
    sensorTime = endTime - startTime
    return (sensorData,
            lastTime, lastTime_2, lastTime_m,
            lastSteps, lastSteps_2, lastSteps_m,
            sensorTime)


# ---------------------------------------------------------------------------
# End switch handling using pigpio callbacks
# ---------------------------------------------------------------------------
def end_switch_callback(gpio, level, tick):
    """
    Callback executed when an end switch is triggered.

    Args:
        gpio (int): GPIO pin number where event occurred.
        level (int): Pin level at event (0 or 1).
        tick (int): Time stamp from pigpio (in microseconds).
    """
    MOTOR.stop_motor()
    print(f"End switch triggered on pin {gpio}. Motor stopped.")


# Configure end switches as inputs with pull-ups and attach callbacks
for switch_pin in [switchPin_1, switchPin_2]:
    pi.set_mode(switch_pin, pigpio.INPUT)
    pi.set_pull_up_down(switch_pin, pigpio.PUD_UP)   # Assumes normally-open to GND
    pi.callback(switch_pin, pigpio.FALLING_EDGE, end_switch_callback)


# ---------------------------------------------------------------------------
# Initial time/step snapshots for velocity computation
# ---------------------------------------------------------------------------
lastTime = time.time()
lastTime_2 = time.time()
lastTime_m = time.time()
lastSteps = ENCODER.steps
lastSteps_2 = ENCODER_2.steps
lastSteps_m = ENCODER_M.steps


# ---------------------------------------------------------------------------
# Data arrays for timing analysis
# ---------------------------------------------------------------------------
sensorTimeArray = []          # Time spent reading sensors
stepCalculationTimeArray = [] # Placeholder for step calc time (currently move_timeout)
movementTimeArray = []        # Time spent executing motor movement


# ---------------------------------------------------------------------------
# Main real-time control loop
# ---------------------------------------------------------------------------
try:
    while True:
        try:
            # 1. Sensor Reading
            (sensorData,
             lastTime, lastTime_2, lastTime_m,
             lastSteps, lastSteps_2, lastSteps_m,
             sensorTime) = read_sensors_data(
                lastTime, lastTime_2, lastTime_m,
                lastSteps, lastSteps_2, lastSteps_m
            )
            print('Sensor Data:', sensorData)

            # 2. State-feedback control law: u = -K x
            u = - K @ sensorData
            print('Control Output:', u)

            # 3. Convert control input to stepper motion command
            #    (steps, frequency, direction sign)
            steps, freq, direction = MOTOR.calculate_steps(float(u))
            print('steps:', steps, 'step freq:', freq)

            # 4. Move Motor using pigpio waveforms (non-blocking)
            startMovement = time.perf_counter()
            MOTOR.move_stepper(steps, freq, np.sign(direction) * 1)

            # Poll until move finishes or timeout exceeds sample time
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

            # 5. Store timing data for offline analysis
            sensorTimeArray.append(sensorTime)
            stepCalculationTimeArray.append(move_timeout)
            movementTimeArray.append(movementTime)

        except KeyboardInterrupt:
            # Allow user to stop the loop with Ctrl+C
            print("User interrupted - exiting safely.")
            break

        except Exception as e:
            # Catch unexpected runtime errors and exit loop
            print('An error occurred:', str(e))
            break

finally:
    # -----------------------------------------------------------------------
    # Save timing data and clean up resources (always executed on exit)
    # -----------------------------------------------------------------------
    np.array(sensorTimeArray).tofile('sensorTimeArray.csv', sep=',')
    np.array(stepCalculationTimeArray).tofile('stepCalculationTimeArray.csv', sep=',')
    np.array(movementTimeArray).tofile('movementTimeArray.csv', sep=',')

    # Clean up encoders and motor
    ENCODER.cleanup()
    ENCODER_2.cleanup()
    ENCODER_M.cleanup()
    MOTOR.stop_motor()

    # Stop pigpio connection
    pi.stop()
    print('All resources cleaned up and data saved.')
