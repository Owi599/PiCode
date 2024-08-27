# Import necassry Libraries
import serial # python library to communicate with the 5G_Mod
import time   # python time library	
import keyboard	# python library to work in keyboard input
from pynput.keyboard import Listener #keyboard listener to register keystrokes 
import RPi.GPIO as gpio # innate gpio control library (works only on pi)
import numpy as np #for numerical operations
from gpiozero import RotaryEncoder #to creat Rotary encoder instance

gpio.setwarnings(False) #saves time by reducing unnecessary warnings
gpio.setmode(gpio.BCM) # sets Board numbering mode
modes={-1:"Unset",11:"BCM",10:"BOARD"} # assign strings to mode integers
mode= gpio.getmode() 
print('The pin numbering mode is: '+ modes[mode])


s = serial.Serial("/dev/ttyUSB2",115200) # creates a serial instance through which the communicates with 5G mode through at-commands and different data types
rec_buff= '' # temp variable



	
# Creating encoder object using GPIO pins 20 and 21
encoder = RotaryEncoder(20,21,max_steps=0)
# Assigning parameter values
cpr = 1250 / 4  # Pulses Per Revolution of the encoder

	
#Function Def to initiate Network and TCP port
def initiateUDP():
	#AT-commands to test the Connection
	send_at('AT','OK',1)
	send_at('ATE1','OK',1)
	#AT-command to open the TCP/IP connection to the 5G Network (VERY IMPORTANT TO START WITH THIS COMMMAND)
	if send_at('AT+NETOPEN?','+NETOPEN: 0',1):
		send_at('AT+NETOPEN','OK',1)
		time.sleep(0.01)
	#Start conection between TCP-client (5G MOD) and TCP-Server (Laptop/other device)
	send_at('AT+CIPOPEN=1,"UDP",,,5000','OK',1)
	
	
#Function Definition to cut off TCP connection from client to server
def CloseConnection():
	#cut TCP and Network connection to free the line
	send_at('AT+CIPCLOSE=0','OK',1)
	send_at('AT+NETCLOSE','OK',1)
	#unassign all pins
	gpio.cleanup()
	
	

#Function defintion for sending AT-Commands
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
		return 1	
		
#Get-Function to periodically ask for sensor data
def ask_ME():
	while True:
			time.sleep(0.05)
			sensor1 = str(2*np.pi / cpr*encoder.steps)
			send_at('AT+CIPSEND=1,7,"10.0.3.39",664','',1)
			s.write(sensor1.encode())
			time.sleep(0.05)
			ask_ME()
			
	
initiateUDP()
time.sleep(0.5)
while True: 
	#call ask_ME function
	try:	 	
		ask_ME()
	except KeyboardInterrupt:
		break
time.sleep(.08)
CloseConnection()
s.close()
	
