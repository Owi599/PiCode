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
        self.zero_steps = 0  # Steps at calibration point
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
        Sets the zero reference point.
        Current position will be interpreted as target_angle.
        """
        # Store current steps as the reference for target_angle
        self.zero_steps = self.steps - int((target_angle * cpr) / (2 * np.pi))
        self.last_steps = self.steps
        print(f"Encoder '{self.id}' calibrated.")
        print(f"  Current physical position = {target_angle:.2f} rad")
        print(f"  Raw steps = {self.steps}, Zero reference = {self.zero_steps}")

    def read_position(self, cpr):
        """
        Returns angular position in [-pi, pi] range.
        """
        # Steps relative to zero reference
        relative_steps = self.steps - self.zero_steps
        
        # Convert to radians
        angle = (2.0 * np.pi * relative_steps) / cpr
        
        # Wrap to [-pi, pi] using atan2 method (most reliable)
        angle_wrapped = np.arctan2(np.sin(angle), np.cos(angle))
        
        return angle_wrapped

    def read_velocity(self, cpr):
        """
        Calculates angular velocity in rad/s.
        """
        current_time = time.time()
        current_steps = self.steps

        time_diff = current_time - self.last_time
        step_diff = current_steps - self.last_steps

        if time_diff < 0.0001:
            return 0, current_time, current_steps

        angular_velocity = (step_diff / time_diff) * (2.0 * np.pi / cpr)

        self.last_time = current_time
        self.last_steps = current_steps

        return angular_velocity, current_time, current_steps

    def cleanup(self):
        """Cleans up the callbacks."""
        self.cbA.cancel()
        self.cbB.cancel()
