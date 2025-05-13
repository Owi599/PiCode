# Necessary imports
from com import UDP, TCP 					#Communication class for UDP and TCP
from rotaryencoder import ReadRotaryEncoder # Class for reading and parsing rotary encoders' data
from motorencoder import ReadMotorEncoder   # Class for reading and parsing motor encoders' data
from endswitch import EndSwitch             # Class for reading and parsing photoelectronic sensors' data
from motorControl import MotorControl       # Class for controlling motor output
import RPi.GPIO as GPIO                     # GPIO library for Raspberry Pi 
import numpy as np                          # Numpy library for numerical operations  
import time                                 # Time library for time operations  
import string as strg                       # String library for string operations
# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#Constant Values
ethServer = "10.0.8.40"
ethClient = "10.0.8.55"
ethServerPortSensor = 890
ethServerPortControl = 5000

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

#Communiaction Server/Clients initiation       
UDPSENSOR    = UDP(ethServer,ethServerPortSensor)
UDPCONTROL   = UDP(ethClient,ethServerPortControl)
server = UDPSENSOR.create_server()  # Create UDP server for sensor data
client = UDPCONTROL.create_client()  # Create UDP client for control data
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
t_sample = 0.02 # sampling time

# define event interrupt
GPIO.add_event_detect(switchPin_1, GPIO.FALLING, callback=end_switch_callback, bouncetime=0)  # Event detection for end switch 1
GPIO.add_event_detect(switchPin_2, GPIO.FALLING, callback=end_switch_callback, bouncetime=0)  # Event detection for end switch 2

data = ''

try:
	while True:
		sensorData, lastTime,lastTime_2,lastTime_m,lastSteps, lastSteps_2,lastSteps_m = read_sensors_data(lastTime,lastTime_2,lastTime_m,lastSteps, lastSteps_2,lastSteps_m)  # Read sensor data
		data = strg.join(sensorData)
		UDPSENSOR.send_data(data,client)
		print('Sensor data sent:', sensorData)
		u = float(UDPCONTROL.receive_data(server).strip())
		print('Control output received:', u)
		steps, stepPeriod, velocity, position = MOTOR.calculate_steps(u,velocity,position, t_sample)  # Calculate motor steps and period
    	
		if np.sign(u)*1 == 1:  # Set motor direction based on control output
			MOTOR.move_stepper(steps, stepPeriod, 1)
		else:
			MOTOR.move_stepper(steps, stepPeriod, -1)

except KeyboardInterrupt:
	GPIO.cleanup()

except Exception as e:
	print('An error occured:', str(e))
	GPIO.cleanup()
