import RPi.GPIO as GPIO  # GPIO library for Raspberry Pi
import time

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

pulse = 9
dir = 10

GPIO.setup(pulse, GPIO.OUT)
GPIO.setup(dir, GPIO.OUT)
try:
    while True:
        for i in range(1600):  # Send 10 pulses
            GPIO.output(dir, GPIO.HIGH)  # Set direction to HIGH
            GPIO.output(pulse, GPIO.HIGH)  # Set pulse HIGH
            time.sleep(3.33e-6)  # Wait for 1 ms
            GPIO.output(pulse, GPIO.LOW)  # Set pulse LOW
            time.sleep(3.33e-6)
        for i in range(1600):  # Send 10 pulses in the opposite direction
            GPIO.output(dir, GPIO.LOW)
            GPIO.output(pulse, GPIO.HIGH)
            time.sleep(3.33e-6)
            GPIO.output(pulse, GPIO.LOW)
            time.sleep(3.33e-6)
        print("Pulse sent")

        # Add a delay to avoid flooding the output
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Exiting...")
    GPIO.cleanup()  # Clean up GPIO settings