#This is the code for the ethernet interface for communnicating sensor and control data through the ethernet interface of the RPi
from com import UDP, TCP
from rotaryencoder import ReadRotaryEncoder#child class wherer all sensor reading related methods are stored
from motorencoder import ReadMotorEncoder
from PhotoelectricSensor import EndSwitch
from MotorCtrlOutput import CTRL
import RPi.GPIO as GPIO
import numpy as np
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#Constant Values
ETH_SERVER_IP = "10.0.8.40"
ETH_CLIENT = "10.0.8.55"
ETH_SERVER_PORT_SENSOR = 890
ETH_SERVER_PORT_CTRL = 5000

#Creating objects for the endswitch using GPIO pins 4 and 17
# SwitchPin_1 = 4
# EndSwitch = EndSwitch(SwitchPin_1)
# SwitchPin_2 = 17
# EndSwitch_2 = EndSwitch(SwitchPin_2)

#Creating object for the motor control using GPIO pins 09 and 10
PulsePin = 9
DirPin = 10


StepsPerRev = 200
PulleyRad = 2.2
HoldingTorque = 2
Motor = CTRL(PulsePin,DirPin,StepsPerRev,PulleyRad,HoldingTorque)

# Creating encoder object using GPIO pins 7 and 8 in BCM mode
encoder_m     = ReadMotorEncoder(7, 8, max_steps=0)
cpr_m         = 500

# Creating encoder object using GPIO pins 20 and 21
encoder       = ReadRotaryEncoder(21,20,max_steps=625,wrap=True)
encoder_2     = ReadRotaryEncoder(16,12,max_steps=625,wrap=True)
encoder.steps = 625
encoder_2.steps = 0
cpr           = 1250 

# Initialize variables for angular velocity measurement
last_time     = time.time()
last_steps    = encoder.steps
last_steps_m  = encoder_m.steps

#Communiaction Server/Clients initiation       
UDP_SENSOR    = UDP(ETH_SERVER_IP,ETH_SERVER_PORT_SENSOR)
UDP_CTRL      = UDP(ETH_CLIENT,ETH_SERVER_PORT_CTRL)

Data  = [] 
strg = ' '
while True:
	try:
		time.sleep(0.02)
		Data.append(str("{:.2f}".format(encoder_m.readPosition(cpr_m))))
		Data.append(str("{:.2f}".format(encoder.readPosition(cpr))))
		Data.append(str("{:.2f}".format(encoder_2.readPosition(cpr))))
		Data.append(str("{:.2f}".format(encoder_m.readVelocity(cpr_m,last_time,last_steps_m))))
		Data.append(str("{:.2f}".format(encoder.readVelocity(cpr,last_time,last_steps))))
		Data.append(str("{:.2f}".format(encoder_2.readVelocity(cpr,last_time,last_steps))))
		data = strg.join(Data)
		UDP_SENSOR.SendData(data)
		try:
			C = float(UDP_CTRL.RecData().strip())
		except ValueError:
			print('Invalid data received')
			C = 0
		print(C)
		Data = [] 
		if C < 0:
			Motor.Stepper(C,-1,encoder_m.readVelocity(cpr_m,last_time,last_steps_m)[0])
		elif C > 0:
			Motor.Stepper(C,1,encoder_m.readVelocity(cpr_m,last_time,last_steps_m)[0])
		


	except KeyboardInterrupt:
		GPIO.cleanup()
		break
	except Exception as e:
		print('An error occured:', str(e))
		GPIO.cleanup()
		break
