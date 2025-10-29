import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(9,GPIO.OUT)
GPIO.setup(10,GPIO.OUT)
def move_stepper(steps:int, stepPeriod:float, direction:int):
        DIR = 10
        pulsePin = 9
        if direction == 1:
            GPIO.output(DIR, GPIO.HIGH)
        elif direction == -1:
            GPIO.output(DIR,GPIO.LOW)
        else:
            raise ValueError('Direction must be 1 or -1')
        
            
        for _ in range(steps):
            GPIO.output(pulsePin,GPIO.HIGH)
            time.sleep(stepPeriod/2)
            GPIO.output(pulsePin,GPIO.LOW)
            time.sleep(stepPeriod/2)

try:
    move_stepper(206,9.680295166085153e-05,-1)
    move_stepper(108,0.00018456327791018692,-1)
    move_stepper(42,0.0004711074615600865,-1)
    move_stepper(0,0.02,1)
    move_stepper(27,0.0007)

except Exception as e:
    print('An error occurred:', str(e)) 
    GPIO.cleanup()
except KeyboardInterrupt:
    print('Exiting....')
    GPIO.cleanup()