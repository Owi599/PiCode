import pigpio
import time
import numpy as np

class PigpioQuadratureEncoder:
    def __init__(self, pi, gpio_a, gpio_b, cpr, name="enc"):
        if not pi.connected:
            raise RuntimeError("pigpio not connected")

        self.pi = pi
        self.gpio_a = gpio_a
        self.gpio_b = gpio_b
        self.cpr = cpr              # counts per mechanical revolution (5000)
        self.name = name

        self.steps = 0              # raw count
        self.ref_steps = 0          # reference at calibration
        self.last_steps = 0
        self.last_time = time.time()

        # internal state for decoding
        self.lev_a = 0
        self.lev_b = 0
        self.last_gpio = None

        # configure pins
        self.pi.set_mode(gpio_a, pigpio.INPUT)
        self.pi.set_mode(gpio_b, pigpio.INPUT)
        self.pi.set_pull_up_down(gpio_a, pigpio.PUD_UP)
        self.pi.set_pull_up_down(gpio_b, pigpio.PUD_UP)

        # callbacks: both edges, both channels
        self.cb_a = self.pi.callback(gpio_a, pigpio.EITHER_EDGE, self._pulse)
        self.cb_b = self.pi.callback(gpio_b, pigpio.EITHER_EDGE, self._pulse)

    def _pulse(self, gpio, level, tick):
        # quadrature 4x decode
        if gpio == self.gpio_a:
            self.lev_a = level
        else:
            self.lev_b = level

        if gpio != self.last_gpio:    # simple debounce
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

    # --- calibration and reading API ---

    def calibrate_to_angle(self, target_angle_rad):
        """
        Set current physical position to correspond to target_angle_rad.
        For this encoder object, all future angles will be relative to this calibration.
        """
        # we want: theta = 2π*(steps - ref_steps)/cpr
        # at calibration: target_angle = 2π*(steps_now - ref_steps)/cpr
        # => ref_steps = steps_now - target_angle*cpr/(2π)
        self.ref_steps = self.steps - int(target_angle_rad * self.cpr / (2.0 * np.pi))
        self.last_steps = self.steps
        print(f"[{self.name}] calibrated: target_angle = {target_angle_rad:.3f} rad")

    def get_angle(self):
        """
        Return wrapped angle in [-π, π] relative to the calibration.
        """
        rel_steps = self.steps - self.ref_steps
        angle = 2.0 * np.pi * rel_steps / self.cpr

        # wrap to [-π, π]
        angle = np.fmod(angle, 2.0 * np.pi)
        if angle > np.pi:
            angle -= 2.0 * np.pi
        elif angle < -np.pi:
            angle += 2.0 * np.pi

        return angle

    def get_velocity(self):
        """
        Angular velocity in rad/s, based on raw steps (no wrapping).
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
        self.cb_a.cancel()
        self.cb_b.cancel()
