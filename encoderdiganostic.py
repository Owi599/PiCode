import pigpio
import numpy as np
import time
from rotaryEncoder import PigpioQuadratureEncoder
CPR = 625  # As you discovered

pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Start pigpiod: sudo pigpiod")


enc_arm1 = PigpioQuadratureEncoder(pi, gpio_a=20, gpio_b=21, cpr=CPR, name="Arm1")
enc_arm2 = PigpioQuadratureEncoder(pi, gpio_a=16, gpio_b=12, cpr=CPR, name="Arm2")

try:
    print("=" * 60)
    print("DIAGNOSTIC: Check encoder alignment")
    print("=" * 60)
    
    # Calibrate both at current position (should be upright)
    print("\nBoth arms at UPRIGHT position. Calibrating...")
    enc_arm1.calibrate(0.0)
    enc_arm2.calibrate(0.0)
    
    print("\nNow ROTATE BOTH ARMS by the SAME physical amount (e.g., 90° clockwise).")
    print("Check if both encoders show similar angles.\n")
    
    for i in range(50):
        theta1 = enc_arm1.read_position()
        theta2 = enc_arm2.read_position()
        diff = abs(theta1 - theta2)
        
        print(f"Arm1: {theta1:+.3f} rad | Arm2: {theta2:+.3f} rad | Diff: {diff:.3f} rad ({np.degrees(diff):.1f}°)")
        time.sleep(0.1)
    
    print("\n" + "=" * 60)
    print("If difference grows linearly or stays constant: channels may be swapped.")
    print("If Arm2 shows constant offset: different encoder resolution or calibration issue.")
    print("=" * 60)

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    enc_arm1.cleanup()
    enc_arm2.cleanup()
    pi.stop()
