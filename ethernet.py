#This is the code for the ethernet interface for communnicating sensor and control data through the ethernet interface of the RPi
from com import UDP, TCP
from gpiozero import RotaryEncoder #to creat Rotary encoder instance
from rotaryencoder import ReadRotaryEncoder#child class wherer all sensor reading related methods are stored
from motorencoder import ReadMotorEncoder
import numpy as np
import time

#Constant Values
ETH_SERVER_IP = "10.0.8.40"
ETH_CLIENT = "10.0.8.55"
ETH_SERVER_PORT_SENSOR = 890
ETH_SERVER_PORT_CTRL = 5000

# Creating encoder object using GPIO pins 7 and 8 in BCM mode
encoder_m     = ReadMotorEncoder(7, 8, max_steps=0)
cpr_m         = 500

# Creating encoder object using GPIO pins 20 and 21
encoder       = ReadRotaryEncoder(21,20,max_steps=625,wrap=True)
encoder.steps = 625
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
		time.sleep(0.005)
		Data.append(str("{:.2f}".format(encoder_m.readPosition(cpr_m))))
		Data.append(str("{:.2f}".format(encoder.readPosition(cpr))))
		Data.append(str("{:.2f}".format(encoder_m.readVelocity(cpr_m,last_time,last_steps_m))))
		Data.append(str("{:.2f}".format(encoder.readVelocity(cpr,last_time,last_steps))))
		data = strg.join(Data)
		UDP_SENSOR.SendData(data)
		print("{:.3f}".format(float(UDP_CTRL.RecData())))
		Data = [] 
	except KeyboardInterrupt:
		break
	except Exception as e:
		print('An error occured:', str(e))