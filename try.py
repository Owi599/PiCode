#This is the code for the ethernet interface for communnicating sensor and control data through the ethernet interface of the RPi
from com import UDP
from gpiozero import RotaryEncoder #to creat Rotary encoder instance
from rotaryencoder import ReadRotaryEncoder#child class wherer all sensor reading related methods are stored
from motorencoder import ReadMotorEncoder
import numpy as np
import time

#udpCTRL = COM("10.0.8.55",5000,1)
# Creating encoder object using GPIO pins 7 and 8 in BCM mode
encoder_m     = ReadMotorEncoder(7, 8, max_steps=0)
cpr_m         = 500

encoder       = ReadRotaryEncoder(16,12,max_steps = 625, wrap=True)
encoder.steps = 0
cpr           = 1250

last_time = time.time()
last_time_m = time.time()
last_steps    = encoder.steps
last_steps_m  = encoder_m.steps

while True:
	time.sleep(0.05)
	x = encoder_m.readPosition(cpr_m)
	pos = "{:.2f}".format(x)
	a = encoder.readPosition(cpr)
	agl = "{:.2f}".format(a)
	v, last_time_m, last_steps_m = encoder_m.readVelocity(cpr_m,last_time,last_steps_m)
	vel = "{:.3f}".format(v)
	w, last_time, last_steps = encoder.readVelocity(cpr,last_time,last_steps)
	omg =   "{:.3f}".format(w)

	
	print(pos,',',agl,',',vel,',',omg)
	
