from motorencoder import ReadMotorEncoder
from rotaryencoder import ReadRotaryEncoder
import time
motor_encoder_A = 20
motor_encoder_B = 21
encoder_a = 7
encoder_b = 8
cpr=1250
microstep = 4
ENCODDER = ReadRotaryEncoder(motor_encoder_A, motor_encoder_B, max_steps=625, wrap=True)
currentTime = 0
ENCODDER.steps = 625

while True:
    position = ENCODDER.read_position(cpr)
    # velocity, currentTime, currentSteps = ENCODDER.read_velocity(cpr, currentTime, currentSteps)
    print(f"Position: {(position*100):.2f} cm")
    time.sleep(0.02)  # Adjust the sleep time as needed