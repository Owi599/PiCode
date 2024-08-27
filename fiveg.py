#This is the code for the 5G interface for communnicating sensor and control data through the 5G Module interface of the RPi
from com import COM
from gpiozero import RotaryEncoder #to creat Rotary encoder instance
from rotaryencoder import ReadRotaryEncoder
import numpy as np
import time
import serial 
from curses import ascii

# Creating encoder object using GPIO pins 20 and 21
encoder = RotaryEncoder(21,20,max_steps=625,wrap=True)
encoder.steps = 625
cpr= 1250

# Initiating USB Serial port instance
s = serial.Serial("/dev/ttyUSB2",115200) # creates a serial instance through which the communicates with 5G mode through at-commands and different data types
rec_buff= '' # temp variable


def initiateUDP():
	send_at('AT','OK',1)
	send_at('ATE1','OK',1)
	if send_at('AT+NETOPEN?','+NETOPEN: 0',1):
		send_at('AT+NETOPEN','OK',1)
		time.sleep(0.01)
	send_at('AT+CIPOPEN=1,"UDP",,,5000','OK',1)

def send_at(command,back,timeout):
	rec_buff= ''
	s.write((command+ '\r\n').encode())
	time.sleep(timeout)
	if s.inWaiting():
		time.sleep(0.01)
		rec_buff=s.read(s.inWaiting())
	if back not in rec_buff.decode():
		print(command + 'ERROR')
		print(command + ' back:\t' + rec_buff.decode())
		return 0
	else:
		print(rec_buff.decode())
		return back
def readValue(CPR):
		if encoder.wait_for_rotate():			
			sensorFloat = (2*np.pi/ cpr*encoder.steps)
			if sensorFloat >= 0:
				sensorFloat = sensorFloat 
			if sensorFloat < 0:
				sensorFloat = sensorFloat 
			sensor = str(sensorFloat)
			return sensor
def ask_ME(CPR):
			sensor = readValue(CPR)
			AT = 'AT+CIPSEND=1,,"10.0.3.55",664'
			send_at(AT,'',1)
			time.sleep(0.5)
			s.write(sensor.encode())
			ctrl = chr(26)
			s.write(ctrl.encode())
			time.sleep(0.05)
			
def CloseConnection():
	send_at('AT+CIPCLOSE=0','OK',1)
	send_at('AT+NETCLOSE','OK',1)


initiateUDP()
while True:
	try:
		ask_ME(1250)
		time.sleep(0.5)
		s.write(chr(10).encode())
		data = send_at('AT+CIPRXGET=0','',1)
		print(data)
	except KeyboardInterrupt:
		break
CloseConnection()
