import pigpio
import numpy as np
import time
from rotaryEncoder import PigpioQuadratureEncoder

CPR = 1250  # Cycles per revolution (4x decoding handled internally)

pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Start pigpiod: sudo pigpiod")

# Create encoders
enc_arm1 = PigpioQuadratureEncoder(pi, gpio_a=20, gpio_b=21, cpr=CPR, name="Arm1")
enc_arm2 = PigpioQuadratureEncoder(pi, gpio_a=16, gpio_b=12, cpr=CPR, name="Arm2")

try:
    # Calibration
    print("Place Arm 1 hanging DOWN.")
    input("Press Enter to calibrate Arm1 to π...")
    enc_arm1.calibrate(np.pi)
    
    print("Place Arm 2 aligned with Arm 1.")
    input("Press Enter to calibrate Arm2 to 0...")
    enc_arm2.calibrate(0.0)
    
    print("\nMeasuring (Ctrl+C to stop)...\n")
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
