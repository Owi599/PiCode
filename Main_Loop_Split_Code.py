# Necessary imports
from com import UDP, TCP                     # Communication class for UDP and TCP
from rotaryencoder import ReadRotaryEncoder  # Class for reading rotary encoders' data
from motorencoder import ReadMotorEncoder    # Class for reading motor encoder data
from endswitch import EndSwitch              # Class for reading photoelectronic sensors' data
from motorControl import MotorControl        # Class for controlling motor output
import RPi.GPIO as GPIO                      # GPIO library for Raspberry Pi
import numpy as np                           # Numpy for numerical operations
import time                                  # Time utilities
import string as strg                        # String library (used here for join alias)

# ---------------------------------------------------------------------------
# GPIO setup
# ---------------------------------------------------------------------------
GPIO.setmode(GPIO.BCM)       # Use BCM (GPIO) pin numbering
GPIO.setwarnings(False)      # Suppress GPIO warnings (re-run safe)

# ---------------------------------------------------------------------------
# Network configuration (Ethernet endpoints and ports)
# ---------------------------------------------------------------------------
ethServer = "10.0.8.40"          # IP of PC/host for sensor reception
ethClient = "10.0.8.55"          # IP of Raspberry Pi for control output
ethServerPortSensor = 890        # UDP port for sending sensor data
ethServerPortControl = 5000      # UDP port for receiving control inputs

# ---------------------------------------------------------------------------
# Pin assignments
# ---------------------------------------------------------------------------
switchPin_1 = 4                  # End switch 1 (limit switch, safety)
switchPin_2 = 17                 # End switch 2
pulsePin = 9                     # Step/clock pin for stepper driver
dirPin = 10                      # Direction pin for stepper driver
encoder_1_A = 21                 # Lower pendulum encoder channel A
encoder_1_B = 20                 # Lower pendulum encoder channel B
encoder_2_A = 16                 # Upper pendulum encoder channel A
encoder_2_B = 12                 # Upper pendulum encoder channel B
motor_encoder_A = 7              # Cart/motor encoder channel A
motor_encoder_B = 8              # Cart/motor encoder channel B

# ---------------------------------------------------------------------------
# Mechanical/encoder configuration
# ---------------------------------------------------------------------------
stepsPerRev = 200        # Full steps per revolution for the motor
pulleyRad = 0.0125       # Pulley radius [m]
holdingTorque = 2        # Motor holding torque [Nm] (not used directly here)
cpr_m = 500              # Counts per revolution (cart encoder)
maxSteps = 625           # Max encoder steps for internal wrapping
cpr = 1250               # Counts per revolution (pendulum encoders)

# ---------------------------------------------------------------------------
# Instantiate sensor and actuator objects
# ---------------------------------------------------------------------------
ENCODER = ReadRotaryEncoder(encoder_1_A, encoder_1_B,
                            max_steps=maxSteps, wrap=True)      # Lower pendulum
ENCODER_2 = ReadRotaryEncoder(encoder_2_A, encoder_2_B,
                              max_steps=maxSteps, wrap=True)    # Upper pendulum
ENCODER_M = ReadMotorEncoder(motor_encoder_A, motor_encoder_B,
                             max_steps=0)                       # Motor/cart encoder

# END_SWITCH = EndSwitch(switchPin_1)      # Optional: hardware end switch 1
# END_SWITCH_2 = EndSwitch(switchPin_2)    # Optional: hardware end switch 2

MOTOR = MotorControl(pulsePin, dirPin, stepsPerRev,
                     pulleyRad, holdingTorque)                  # Stepper motor interface

# ---------------------------------------------------------------------------
# Communication (UDP) setup
# ---------------------------------------------------------------------------
UDPSENSOR = UDP(ethServer, ethServerPortSensor)     # Sends sensor data to host
UDPCONTROL = UDP(ethClient, ethServerPortControl)   # Receives control commands
server = UDPSENSOR.create_server()                  # UDP server socket (RPi bind)
client = UDPCONTROL.create_client()                 # UDP client socket (RPi send)

# ---------------------------------------------------------------------------
# Sensor reading function (positions + velocities)
# ---------------------------------------------------------------------------
def read_sensors_data(lastTime, lastTime_2, lastTime_m,
                      lastSteps, lastSteps_2, lastSteps_m):
    """
    Read all encoder channels, compute velocities, and return formatted strings.

    Args:
        lastTime, lastTime_2, lastTime_m (float): Previous timestamps for each encoder.
        lastSteps, lastSteps_2, lastSteps_m (int): Previous steps for each encoder.

    Returns:
        tuple: (sensorData, lastTime, lastTime_2, lastTime_m,
                lastSteps, lastSteps_2, lastSteps_m, sensorTime)
            sensorData (list[str]): [x, theta1, theta2, x_dot, theta1_dot, theta2_dot]
                                     formatted with 2 decimal places.
    """
    startTime = time.perf_counter()
    sensorData = []

    # Motor/cart position [m]
    sensorData.append("{:.2f}".format(ENCODER_M.read_position(cpr_m)))

    # Pendulum 1 angle [rad]
    sensorData.append("{:.2f}".format(ENCODER.read_position(cpr)))

    # Pendulum 2 angle [rad]
    sensorData.append("{:.2f}".format(ENCODER_2.read_position(cpr)))

    # Motor/cart velocity [m/s]
    v, lastTime_m, lastSteps_m = ENCODER_M.read_velocity(cpr_m, lastTime, lastSteps_m)
    sensorData.append("{:.2f}".format(v))

    # Pendulum 1 angular velocity [rad/s]
    omega_1, lastTime, lastSteps = ENCODER.read_velocity(cpr, lastTime, lastSteps)
    sensorData.append("{:.2f}".format(omega_1))

    # Pendulum 2 angular velocity [rad/s]
    omega_2, lastTime_2, lastSteps_2 = ENCODER_2.read_velocity(cpr, lastTime, lastSteps_2)
    sensorData.append("{:.2f}".format(omega_2))

    sensorTime = time.perf_counter() - startTime
    return (sensorData,
            lastTime, lastTime_2, lastTime_m,
            lastSteps, lastSteps_2, lastSteps_m,
            sensorTime)

# ---------------------------------------------------------------------------
# End switch callback: emergency stop
# ---------------------------------------------------------------------------
def end_switch_callback(channel):
    """
    GPIO interrupt callback for end switches.

    Args:
        channel (int): GPIO pin on which the event occurred.
    """
    MotorControl.stop_motor()   # Stop motor immediately (class method or adjust to instance)


# ---------------------------------------------------------------------------
# Timing/loop state initialization
# ---------------------------------------------------------------------------
lastTime = time.time()          # Timestamp for pendulum 1
lastTime_2 = time.time()        # Timestamp for pendulum 2
lastTime_m = time.time()        # Timestamp for motor
lastSteps = ENCODER.steps       # Previous steps for pendulum 1
lastSteps_2 = ENCODER_2.steps   # Previous steps for pendulum 2
lastSteps_m = ENCODER_M.steps   # Previous steps for motor encoder
velocity = 0                    # Initial cart velocity reference
position = 0                    # Initial cart position reference
t_sample = 0.02                 # Sampling time [s]

# ---------------------------------------------------------------------------
# Configure GPIO interrupt for end switches
# ---------------------------------------------------------------------------
GPIO.add_event_detect(switchPin_1, GPIO.FALLING,
                      callback=end_switch_callback, bouncetime=0)
GPIO.add_event_detect(switchPin_2, GPIO.FALLING,
                      callback=end_switch_callback, bouncetime=0)

# ---------------------------------------------------------------------------
# Logging arrays for timing metrics
# ---------------------------------------------------------------------------
data = ''
sensorTimeArray = []
stepCalculationTimeArray = []
movementTimeArray = []
sendTimeArray = []

# ---------------------------------------------------------------------------
# Main real-time loop: sense → send → receive → act
# ---------------------------------------------------------------------------
try:
    while True:
        # 1. Read all sensor channels and compute velocities
        (sensorData,
         lastTime, lastTime_2, lastTime_m,
         lastSteps, lastSteps_2, lastSteps_m,
         sensorTime) = read_sensors_data(
            lastTime, lastTime_2, lastTime_m,
            lastSteps, lastSteps_2, lastSteps_m
        )

        # 2. Serialize sensor data for UDP (space/comma separation if needed)
        data = strg.join(sensorData)

        # 3. Send sensor packet to host and measure send time
        sendTime = UDPSENSOR.send_data(data, client)
        print('Sensor data sent:', sensorData)

        # 4. Receive control output u from host (blocking read)
        u = float(UDPCONTROL.receive_data(server).strip())
        print('Control output received:', u)

        # 5. Map control output to motor steps and period
        steps, stepPeriod, velocity, position, stepCalculationTime = MOTOR.calculate_steps(
            u, velocity, position, t_sample
        )

        # 6. Execute motor movement (direction sign determined by u)
        if np.sign(u) * 1 == 1:
            movementTime = MOTOR.move_stepper(steps, stepPeriod, 1)
        else:
            movementTime = MOTOR.move_stepper(steps, stepPeriod, -1)

        # 7. Log timing data
        sensorTimeArray.append(sensorTime)
        stepCalculationTimeArray.append(stepCalculationTime)
        movementTimeArray.append(movementTime)
        sendTimeArray.append(sendTime)

except KeyboardInterrupt:
    # Graceful termination on user interrupt
    GPIO.cleanup()
    np.array(sensorTimeArray).tofile('sensorTimeArraySplit.csv', sep=',')
    np.array(stepCalculationTimeArray).tofile('stepCalculationTimeArraySplit.csv', sep=',')
    np.array(movementTimeArray).tofile('movementTimeArraySplit.csv', sep=',')
    np.array(sendTimeArray).tofile('sendTimeArray.csv', sep=',')
    print('Program terminated by user. Data saved to CSV files.')

except Exception as e:
    # Catch-all for runtime errors (communication, parsing, etc.)
    print('An error occured:', str(e))
    GPIO.cleanup()
    np.array(sensorTimeArray).tofile('sensorTimeArraySplit.csv', sep=',')
    np.array(stepCalculationTimeArray).tofile('stepCalculationTimeArraySplit.csv', sep=',')
    np.array(movementTimeArray).tofile('movementTimeArraySplit.csv', sep=',')
    np.array(sendTimeArray).tofile('sendTimeArray.csv', sep=',')
    print('Program terminated due to error. Data saved to CSV files.')

finally:
    # Final cleanup (idempotent, in case previous blocks already ran)
    GPIO.cleanup()
    np.array(sensorTimeArray).tofile('sensorTimeArraySplit.csv', sep=',')
    np.array(stepCalculationTimeArray).tofile('stepCalculationTimeArraySplit.csv', sep=',')
    np.array(movementTimeArray).tofile('movementTimeArraySplit.csv', sep=',')
    np.array(sendTimeArray).tofile('sendTimeArray.csv', sep=',')
    print('Program terminated. Data saved to CSV files.')
