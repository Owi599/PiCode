import pigpio
import time
from motorEncoder import ReadMotorEncoder
from rotaryEncoder import ReadRotaryEncoder

# GPIO pin assignments
motor_encoder_A = 20
motor_encoder_B = 21
encoder_a = 16
encoder_b = 12

# Encoder parameters
PPR = 1250  # Pulses Per Revolution
CPR = PPR * 4  # Counts Per Revolution (4x quadrature decoding)

# Connect to pigpio daemon
pi = pigpio.pi()
if not pi.connected:
    print("Could not connect to pigpio daemon. Did you run 'sudo pigpiod'?")
    exit()

try:
    # Create encoder instances with pigpio
    ENCODER = ReadRotaryEncoder(clk_gpio=motor_encoder_A, dt_gpio=motor_encoder_B, pi=pi, id="arm1")
    ENCODER_2 = ReadRotaryEncoder(clk_gpio=encoder_a, dt_gpio=encoder_b, pi=pi, id="arm2")
    
    # Calibrate the encoders to their initial positions
    # Arm 1: Set to pi radians (hanging down)
    ENCODER.calibrate(cpr=CPR, target_angle=3.14159)  # pi radians
    
    # Arm 2: Set to 0 radians (aligned with arm 1)
    ENCODER_2.calibrate(cpr=CPR, target_angle=0)
    
    print("Encoders calibrated. Starting position readings...")
    time.sleep(1)
    
    # Main loop
    while True:
        # Read positions (already wrapped to [-pi, pi])
        position = ENCODER.read_position(CPR)
        position_2 = ENCODER_2.read_position(CPR)
        
        # Read velocities
        velocity, _, _ = ENCODER.read_velocity(CPR)
        velocity_2, _, _ = ENCODER_2.read_velocity(CPR)
        
        print(f"Arm1 - Pos: {position:+.2f} rad, Vel: {velocity:+.2f} rad/s | "
              f"Arm2 - Pos: {position_2:+.2f} rad, Vel: {velocity_2:+.2f} rad/s")
        
        time.sleep(0.02)  # 20ms sampling period

except KeyboardInterrupt:
    print("\nProgram stopped by user.")
finally:
    # Clean up resources
    if 'ENCODER' in locals():
        ENCODER.cleanup()
    if 'ENCODER_2' in locals():
        ENCODER_2.cleanup()
    if pi.connected:
        pi.stop()
    print("Cleanup complete.")
