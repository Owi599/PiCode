import pigpio
import numpy as np
import time
from rotaryEncoder import PigpioQuadratureEncoder
CPR = 625  # Cycles per revolution
CPR_2 = 625
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Start pigpiod: sudo pigpiod")

# Create encoders
enc_arm1 = PigpioQuadratureEncoder(pi, gpio_a=21, gpio_b=20, cpr=CPR, name="Arm1")
enc_arm2 = PigpioQuadratureEncoder(pi, gpio_a=16, gpio_b=12, cpr=CPR_2, name="Arm2")

try:
    # Calibration - directly set step counts
    print("Place Arm 1 hanging DOWN (this is π radians).")
    input("Press Enter to calibrate...")
    enc_arm1.calibrate(np.pi)  # Sets steps = 2500
    
    print("\nPlace Arm 2 aligned with Arm 1 (this is 0 radians).")
    input("Press Enter to calibrate...")
    enc_arm2.calibrate(0)  # Sets steps = 0
    
    print("\nStarting measurements (Ctrl+C to stop)...")
    print("Expected: Arm1 down=+3.14 rad, up=0.00 rad\n")
    time.sleep(1)
    
    # Main loop
    while True:
        theta1 = enc_arm1.read_position()
        theta2 = enc_arm2.read_position()
        omega1 = enc_arm1.read_velocity()
        omega2 = enc_arm2.read_velocity()
        
        print(f"Arm1: {theta1:+.3f} rad ({np.degrees(theta1):+7.2f}°), {omega1:+.2f} rad/s | "
              f"Arm2: {theta2:+.3f} rad ({np.degrees(theta2):+7.2f}°), {omega2:+.2f} rad/s", 
              end="\r")
        
        time.sleep(0.02)

except KeyboardInterrupt:
    print("\n\nStopped.")
finally:
    enc_arm1.cleanup()
    enc_arm2.cleanup()
    pi.stop()
