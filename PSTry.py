import RPi.GPIO as GPIO
from PhotoelectricSensor import EndSwitch
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

Sensor = EndSwitch(4)

while True:
    print(Sensor.read())
    time.sleep(0.5)

