import pigpio
import time
import numpy as np

class ReadRotaryEncoder:
    def __init__(self, clk_gpio, dt_gpio, pi, id="encoder"):
        """
        Initializes the encoder for measuring angular motion using direct pigpio callbacks.
        """
        if not pi.connected:
            raise RuntimeError("pigpio not connected")
            
        self.pi = pi
        self.clk_gpio = clk_gpio
        self.dt_gpio = dt_gpio
        self.steps = 0
        self.calibration_offset = 0  # NEW: Store calibration offset in steps
        self.last_time = time.time()
        self.last_steps = 0
        self.id = id
        
        self.levA = 0
        self.levB = 0
        self.lastGpio = None

        # Set up pins as inputs with pull-ups
        self.pi.set_mode(clk_gpio, pigpio.INPUT)
        self.pi.set_mode(dt_gpio, pigpio.INPUT)
        self.pi.set_pull_up_down(clk_gpio, pigpio.PUD_UP)
        self.pi.set_pull_up_down(dt_gpio, pigpio.PUD_UP)

        # Set up callbacks for both edges on both pins
        self.cbA = self.pi.callback(clk_gpio, pigpio.EITHER_EDGE, self._pulse)
        self.cbB = self.pi.callback(dt_gpio, pigpio.EITHER_EDGE, self._pulse)

    def _pulse(self, gpio, level, tick):
        """
        Callback for quadrature decoding.
        This implements 4x decoding (counts on all edges of both channels).
        """
        if gpio == self.clk_gpio:
            self.levA = level
        else:
            self.levB = level

        if gpio != self.lastGpio:  # Debounce
            self.lastGpio = gpio

            if gpio == self.clk_gpio and level == 1:
                if self.levB == 1:
                    self.steps += 1
                else:
                    self.steps -= 1
            elif gpio == self.dt_gpio and level == 1:
                if self.levA == 1:
                    self.steps -= 1
                else:
                    self.steps += 1

    def calibrate(self, cpr, target_angle):
        """
        Calibrates the encoder by storing an offset.
        The current physical position will correspond to target_angle.
        
        This works by: angle = (steps - offset) * (2π / cpr)
        We want: target_angle = (current_steps - offset) * (2π / cpr)
        Solving for offset: offset = current_steps - (target_angle * cpr) / (2π)
        """
        self.calibration_offset = self.steps - int((target_angle * cpr) / (2 * np.pi))
        self.last_steps = self.steps
        print(f"Encoder '{self.id}' calibrated. Current position set to {target_angle:.2f} rad.")
        print(f"  Raw steps: {self.steps}, Calibration offset: {self.calibration_offset}")

    def read_position(self, cpr):
        """
        Calculates the angular position and wraps it to the [-pi, pi] range.
        Uses the calibration offset so the position is relative to the calibrated zero.
        """
        # Calculate angle using offset
        calibrated_steps = self.steps - self.calibration_offset
        raw_angle = (2 * np.pi / cpr) * calibrated_steps
        
        # Wrap to [-pi, pi] using proper modulo arithmetic
        # This ensures the angle stays in the correct range after multiple revolutions
        wrapped_angle = np.arctan2(np.sin(raw_angle), np.cos(raw_angle))
        
        return wrapped_angle

    def read_velocity(self, cpr):
        """
        Calculates angular velocity from the raw, unwrapped step count.
        Velocity doesn't need wrapping - it's a rate of change.
        """
        current_time = time.time()
        current_steps = self.steps

        time_diff = current_time - self.last_time
        step_diff = current_steps - self.last_steps

        if time_diff < 0.0001:
            return 0, current_time, current_steps

        angular_velocity = (step_diff / time_diff) * (2 * np.pi / cpr)

        self.last_time = current_time
        self.last_steps = current_steps

        return angular_velocity, current_time, current_steps

    def cleanup(self):
        """Cleans up the callbacks."""
        self.cbA.cancel()
        self.cbB.cancel()
