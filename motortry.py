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
        for _ in range(206):  # Send 10 pulses
            GPIO.output(dir, GPIO.LOW)  # Set direction to HIGH
            GPIO.output(pulse, GPIO.HIGH)  # Set pulse HIGH
            time.sleep(0.245e-3)  # Wait for 1 ms
            GPIO.output(pulse, GPIO.LOW)  # Set pulse LOW
            time.sleep(0.245-3)
        for _ in range(108):  # Send 10 pulses in the opposite direction
            GPIO.output(dir, GPIO.LOW)
            GPIO.output(pulse, GPIO.HIGH)
            time.sleep(0.935e-3/2)
            GPIO.output(pulse, GPIO.LOW)
            time.sleep(0.935e-3/2)
        for _ in range(42):
            GPIO.output(dir, GPIO.LOW)
            GPIO.output(pulse, GPIO.HIGH)
            time.sleep(2.3867e-3/2)
            GPIO.output(pulse, GPIO.LOW)
            time.sleep(2.3867e-3/2)
        for _ in range(0):
            GPIO.output(dir, GPIO.HIGH)
            GPIO.output(pulse, GPIO.HIGH)
            time.sleep(0.02/2)
            GPIO.output(pulse, GPIO.LOW)
            time.sleep(0.02/2)
        for _ in range(27):
            GPIO.output(dir, GPIO.HIGH)
            GPIO.output(pulse, GPIO.HIGH)
            time.sleep(3.65e-3/2)
            GPIO.output(pulse, GPIO.LOW)
            time.sleep(3.65e-3/2)         
        for _ in range(43):
            GPIO.output(dir, GPIO.HIGH)
            GPIO.output(pulse, GPIO.HIGH)
            time.sleep(2.313e-3/2)
            GPIO.output(pulse, GPIO.LOW)
            time.sleep(2.313e-3/2)
        for _ in range(52):
            GPIO.output(dir,GPIO.HIGH)
            GPIO.output(pulse,GPIO.HIGH)
            time.sleep(1.936e-3/2)        
            GPIO.output(pulse,GPIO.LOW)
            time.sleep(1.936e-3/2)       
        for _ in range(55):
            GPIO.output(dir,GPIO.HIGH)
            GPIO.output(pulse,GPIO.HIGH)
            time.sleep(1.816e-3/2)        
            GPIO.output(pulse,GPIO.LOW)
            time.sleep(1.816e-3/2)       
        for _ in range(55):
            GPIO.output(dir,GPIO.HIGH)
            GPIO.output(pulse,GPIO.HIGH)
            time.sleep(1.812e-3/2)        
            GPIO.output(pulse,GPIO.LOW)
            time.sleep(1.812e-3/2)       
        for _ in range(53):
            GPIO.output(dir,GPIO.HIGH)
            GPIO.output(pulse,GPIO.HIGH)
            time.sleep(1.883e-3/2)        
            GPIO.output(pulse,GPIO.LOW)
            time.sleep(1.883e-3/2)       
        for _ in range(50):
            GPIO.output(dir,GPIO.HIGH)
            GPIO.output(pulse,GPIO.HIGH)
            time.sleep(2.011e-3/2)        
            GPIO.output(pulse,GPIO.LOW)
            time.sleep(2.011e-3/2)
        for _ in range(46):
            GPIO.output(dir,GPIO.HIGH)
            GPIO.output(pulse,GPIO.HIGH)
            time.sleep(2.197e-3/2)        
            GPIO.output(pulse,GPIO.LOW)
            time.sleep(2.197e-3/2)           
        for _ in range(41):
            GPIO.output(dir,GPIO.HIGH)
            GPIO.output(pulse,GPIO.HIGH)
            time.sleep(2.4476e-3/2)
            GPIO.output(pulse,GPIO.LOW)
            time.sleep(2.4476e-3/2)
except KeyboardInterrupt:
    print("Exiting...")
    GPIO.cleanup()  # Clean up GPIO settings