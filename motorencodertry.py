from motorencoder import ReadMotorEncoder
from rotaryencoder import ReadRotaryEncoder
import time
motor_encoder_A = 20
motor_encoder_B = 21
encoder_a = 16
encoder_b = 12
cpr=1250
microstep = 4
ENCODDER = ReadRotaryEncoder(motor_encoder_A, motor_encoder_B, max_steps = 0, wrap=False)
ENCODDER_2 = ReadRotaryEncoder(encoder_a, encoder_b, max_steps=0, wrap=False)
currentTime = 0
ENCODDER.steps =625
ENCODDER_2.steps =0
while True:
    position = ENCODDER.read_position(cpr)
    position_2 = ENCODDER_2.read_position(cpr)
    # velocity, currentTime, currentSteps = ENCODDER.read_velocity(cpr, currentTime, currentSteps)
    print(f"Position: {(position):.2f}, position2 : {(position_2):.2f} ")
    # time.sleep(0.05)