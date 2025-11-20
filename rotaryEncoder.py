import pigpio
import time
import numpy as np


class PigpioQuadratureEncoder:
    def __init__(self, pi, gpio_a, gpio_b, cpr, name="enc"):
        """
        Pigpio-based quadrature encoder.
        :param cpr: Cycles Per Revolution (e.g., 1250). 
                    Internally uses 4x decoding, so effective counts = cpr * 4
        """
        if not pi.connected:
            raise RuntimeError("pigpio not connected")

        self.pi = pi
        self.gpio_a = gpio_a
        self.gpio_b = gpio_b
        self.cpr = cpr * 4  # 4x decoding: 1250 cycles → 5000 counts per revolution
        self.name = name

        self.steps = 0
        self.last_steps = 0
        self.last_time = time.time()

        # Internal quadrature state
        self.lev_a = 0
        self.lev_b = 0
        self.last_gpio = None

        # Configure pins
        self.pi.set_mode(gpio_a, pigpio.INPUT)
        self.pi.set_mode(gpio_b, pigpio.INPUT)
        self.pi.set_pull_up_down(gpio_a, pigpio.PUD_UP)
        self.pi.set_pull_up_down(gpio_b, pigpio.PUD_UP)

        # Setup callbacks
        self.cb_a = self.pi.callback(gpio_a, pigpio.EITHER_EDGE, self._pulse)
        self.cb_b = self.pi.callback(gpio_b, pigpio.EITHER_EDGE, self._pulse)

    def _pulse(self, gpio, level, tick):
        """Quadrature 4x decoding."""
        if gpio == self.gpio_a:
            self.lev_a = level
        else:
            self.lev_b = level

        if gpio != self.last_gpio:
            self.last_gpio = gpio

            if gpio == self.gpio_a and level == 1:
                if self.lev_b == 1:
                    self.steps += 1
                else:
                    self.steps -= 1
            elif gpio == self.gpio_b and level == 1:
                if self.lev_a == 1:
                    self.steps -= 1
                else:
                    self.steps += 1

    def calibrate(self, target_angle):
        """
        Set the step count to represent the target_angle.
        Upright (0 rad) = 0 steps
        Down (π rad) = cpr/2 steps (half revolution)
        
        Example: calibrate(np.pi) sets steps = 2500 (for cpr=5000)
        """
        self.steps = int(target_angle * self.cpr / (2.0 * np.pi))
        self.last_steps = self.steps
        print(f"[{self.name}] Calibrated to {target_angle:.3f} rad ({np.degrees(target_angle):.1f}°)")
        print(f"  Steps set to: {self.steps}")

    def read_position(self):
        """
        Returns angular position in [-π, π] radians.
        Directly converts steps to angle: angle = 2π * steps / cpr
        """
        angle = (2.0 * np.pi * self.steps) / self.cpr
        
        # Wrap to [-π, π]
        angle = np.fmod(angle, 2.0 * np.pi)
        if angle > np.pi:
            angle -= 2.0 * np.pi
        elif angle < -np.pi:
            angle += 2.0 * np.pi
        
        return angle

    def read_velocity(self):
        """
        Returns angular velocity in rad/s.
        """
        now = time.time()
        steps_now = self.steps

        dt = now - self.last_time
        dsteps = steps_now - self.last_steps

        if dt <= 0.0:
            return 0.0

        vel = (dsteps / dt) * (2.0 * np.pi / self.cpr)

        self.last_time = now
        self.last_steps = steps_now

        return vel

    def cleanup(self):
        """Cancel callbacks."""
        self.cb_a.cancel()
        self.cb_b.cancel()
