import pigpio
import numpy as np
import time
from encoder_class import PigpioQuadratureEncoder   # name as you save it

PPR = 1250
CPR = PPR * 4    # 5000

pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("run: sudo pigpiod")

# GPIO assignment
ARM1_A = 20
ARM1_B = 21
ARM2_A = 16
ARM2_B = 12

enc_arm1 = PigpioQuadratureEncoder(pi, ARM1_A, ARM1_B, CPR, name="Arm1")
enc_arm2 = PigpioQuadratureEncoder(pi, ARM2_A, ARM2_B, CPR, name="Arm2")

try:
    # --- Calibration phase ---
    print("Put ARM 1 (lower) hanging DOWN. This should represent +pi.")
    input("Press Enter to calibrate Arm1 to +pi...")

    # Down = +π for arm 1
    enc_arm1.calibrate_to_angle(np.pi)

    print("Put ARM 2 (upper) ALIGNED with ARM 1 (both hanging down).")
    input("Press Enter to calibrate Arm2 to 0...")

    # Aligned with arm1 at down = 0 for arm 2
    enc_arm2.calibrate_to_angle(0.0)

    print("\nCalibrated. Now reading angles in [-pi, pi]. Ctrl+C to stop.\n")

    while True:
        theta1 = enc_arm1.get_angle()    # [-π, π], down ≈ +π, up ≈ 0
        theta2 = enc_arm2.get_angle()    # [-π, π], aligned down ≈ 0

        omega1 = enc_arm1.get_velocity()
        omega2 = enc_arm2.get_velocity()

        print(
            f"Arm1: θ={theta1:+.3f} rad ({np.degrees(theta1):+7.2f}°), "
            f"ω={omega1:+.2f} rad/s | "
            f"Arm2: θ={theta2:+.3f} rad ({np.degrees(theta2):+7.2f}°), "
            f"ω={omega2:+.2f} rad/s",
            end="\r",
        )
        time.sleep(0.02)

except KeyboardInterrupt:
    print("\nStopped by user.")
finally:
    enc_arm1.cleanup()
    enc_arm2.cleanup()
    pi.stop()
