import pigpio
import time
import numpy as np
from rotaryEncoder import ReadRotaryEncoder

# Connect to pigpio
pi = pigpio.pi()
if not pi.connected:
    print("ERROR: Could not connect to pigpio daemon.")
    print("Please run: sudo pigpiod")
    exit()

print("Connected to pigpio daemon successfully.\n")

# GPIO pins
ARM1_CLK = 20
ARM1_DT = 21
ARM2_CLK = 16
ARM2_DT = 12

# Encoder parameters
PPR = 1250
CPR = PPR * 4  # 5000 counts per revolution with 4x decoding

print(f"Encoder Configuration:")
print(f"  PPR (Pulses Per Revolution): {PPR}")
print(f"  CPR (Counts Per Revolution with 4x decoding): {CPR}")
print(f"  Resolution: {360/CPR:.3f} degrees per count\n")

try:
    # Create encoder objects
    ENCODER_1 = ReadRotaryEncoder(clk_gpio=ARM1_CLK, dt_gpio=ARM1_DT, pi=pi, id="Arm1_Lower")
    ENCODER_2 = ReadRotaryEncoder(clk_gpio=ARM2_CLK, dt_gpio=ARM2_DT, pi=pi, id="Arm2_Upper")
    
    print("=" * 60)
    print("CALIBRATION PROCEDURE")
    print("=" * 60)
    
    # Calibrate Arm 1 (lower arm)
    print("\n[ARM 1 - Lower Arm]")
    print("Position the lower arm HANGING STRAIGHT DOWN.")
    input("Press Enter when ready...")
    ENCODER_1.calibrate(cpr=CPR, target_angle=np.pi)
    
    # Calibrate Arm 2 (upper arm)
    print("\n[ARM 2 - Upper Arm]")
    print("Position the upper arm ALIGNED WITH the lower arm (both hanging down).")
    input("Press Enter when ready...")
    ENCODER_2.calibrate(cpr=CPR, target_angle=0)
    
    print("\n" + "=" * 60)
    print("CALIBRATION COMPLETE - Starting measurements")
    print("=" * 60)
    print("\nExpected readings:")
    print("  Arm 1 down = +3.14 rad (+180°), up = 0.00 rad (0°)")
    print("  Arm 2 aligned with Arm 1 = 0.00 rad (0°)")
    print("\nPress Ctrl+C to stop\n")
    
    time.sleep(2)
    
    # Main measurement loop
    count = 0
    while True:
        pos1 = ENCODER_1.read_position(CPR)
        pos2 = ENCODER_2.read_position(CPR)
        vel1, _, _ = ENCODER_1.read_velocity(CPR)
        vel2, _, _ = ENCODER_2.read_velocity(CPR)
        
        # Print every 10 cycles (0.2 seconds) to avoid screen clutter
        if count % 10 == 0:
            print(f"Arm1: {pos1:+.3f} rad ({np.degrees(pos1):+7.2f}°) {vel1:+.2f} rad/s | "
                  f"Arm2: {pos2:+.3f} rad ({np.degrees(pos2):+7.2f}°) {vel2:+.2f} rad/s")
        
        count += 1
        time.sleep(0.02)

except KeyboardInterrupt:
    print("\n\nProgram stopped by user.")
    
except Exception as e:
    print(f"\nERROR: {e}")
    import traceback
    traceback.print_exc()
    
finally:
    print("\nCleaning up...")
    if 'ENCODER_1' in locals():
        ENCODER_1.cleanup()
    if 'ENCODER_2' in locals():
        ENCODER_2.cleanup()
    if pi.connected:
        pi.stop()
    print("Cleanup complete.")
