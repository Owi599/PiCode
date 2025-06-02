from motorencoder import ReadMotorEncoder
import time
motor_encoder_A = 7
motor_encoder_B = 8
cpr=500
microstep = 4
ENCODDER = ReadMotorEncoder(motor_encoder_A, motor_encoder_B, max_steps=0)
currentTime = 0
currentSteps = 0
while True:
    position = ENCODDER.read_position(cpr, microstep)
    velocity, currentTime, currentSteps = ENCODDER.read_velocity(cpr, currentTime, currentSteps)
    print(f"Position: {(position*100):.2f} cm, Velocity: {velocity:.4f} m/s")
    time.sleep(0.02)  # Adjust the sleep time as needed