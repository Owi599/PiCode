import pigpio
import time
from rotaryEncoder import ReadRotaryEncoder

pi = pigpio.pi()
if not pi.connected:
    print("Could not connect to pigpio daemon.")
    exit()

try:
    # Create encoders
    ENCODER = ReadRotaryEncoder(clk_gpio=20, dt_gpio=21, pi=pi, id="arm1")
    ENCODER_2 = ReadRotaryEncoder(clk_gpio=16, dt_gpio=12, pi=pi, id="arm2")
    
    PPR = 1250
    CPR = PPR * 4
    
    # Calibrate: Arm 1 down = π, Arm 2 aligned = 0
    input("Place Arm 1 hanging DOWN and press Enter...")
    ENCODER.calibrate(cpr=CPR, target_angle=np.pi)
    
    input("Place Arm 2 aligned with Arm 1 and press Enter...")
    ENCODER_2.calibrate(cpr=CPR, target_angle=0)
    
    print("\nCalibration complete! Starting measurements...\n")
    time.sleep(1)
    
    while True:
        pos1 = ENCODER.read_position(CPR)
        pos2 = ENCODER_2.read_position(CPR)
        vel1, _, _ = ENCODER.read_velocity(CPR)
        vel2, _, _ = ENCODER_2.read_velocity(CPR)
        
        print(f"Arm1: {pos1:+.3f} rad ({np.degrees(pos1):+6.1f}°) | "
              f"Arm2: {pos2:+.3f} rad ({np.degrees(pos2):+6.1f}°)")
        
        time.sleep(0.02)

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    ENCODER.cleanup()
    ENCODER_2.cleanup()
    pi.stop()
