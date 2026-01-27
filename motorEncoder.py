import pigpio
import time
import numpy as np


class ReadRotaryEncoder:
    def __init__(self, clk_gpio, dt_gpio, pi, id="encoder"):
        """
        Initialize a quadrature rotary encoder using pigpio callbacks.

        Args:
            clk_gpio (int): GPIO pin for channel A (CLK).
            dt_gpio (int): GPIO pin for channel B (DT).
            pi (pigpio.pi): Active pigpio instance.
            id (str): Optional identifier for logging/debugging.
        """
        if not pi.connected:
            raise RuntimeError("pigpio not connected")

        self.pi = pi
        self.clk_gpio = clk_gpio
        self.dt_gpio = dt_gpio
        self.steps = 0                      # Raw quadrature step count
        self.calibration_offset = 0        # Offset in steps for calibrated zero
        self.last_time = time.time()       # Last timestamp for velocity computation
        self.last_steps = 0                # Last step count for velocity computation
        self.id = id

        # Internal decoder state
        self.levA = 0
        self.levB = 0
        self.lastGpio = None

        # Configure pins as inputs with pull-ups
        self.pi.set_mode(clk_gpio, pigpio.INPUT)
        self.pi.set_mode(dt_gpio, pigpio.INPUT)
        self.pi.set_pull_up_down(clk_gpio, pigpio.PUD_UP)
        self.pi.set_pull_up_down(dt_gpio, pigpio.PUD_UP)

        # Register callbacks on both edges of both channels for 4x decoding
        self.cbA = self.pi.callback(clk_gpio, pigpio.EITHER_EDGE, self._pulse)
        self.cbB = self.pi.callback(dt_gpio, pigpio.EITHER_EDGE, self._pulse)

    def _pulse(self, gpio, level, tick):
        """
        Quadrature decoder callback.

        This is called by pigpio on every edge of both channels.
        Implements 4x decoding (increments or decrements on all rising edges),
        using the phase relationship between A and B to determine direction.
        """
        # Update level of the channel that changed
        if gpio == self.clk_gpio:
            self.levA = level
        else:
            self.levB = level

        # Simple debounce: ignore repeated callbacks from same GPIO
        if gpio != self.lastGpio:
            self.lastGpio = gpio

            # Rising edge on A
            if gpio == self.clk_gpio and level == 1:
                if self.levB == 1:
                    self.steps += 1
                else:
                    self.steps -= 1

            # Rising edge on B
            elif gpio == self.dt_gpio and level == 1:
                if self.levA == 1:
                    self.steps -= 1
                else:
                    self.steps += 1

    def calibrate(self, cpr, target_angle):
        """
        Calibrate the encoder zero based on the current mechanical position.

        After calibration, the current physical angle is mapped to target_angle.

        Angle mapping:
            angle = (steps - offset) * (2π / cpr)

        We enforce:
            target_angle = (current_steps - offset) * (2π / cpr)
        →  offset = current_steps - (target_angle * cpr) / (2π)

        Args:
            cpr (int): Counts per mechanical revolution (full quadrature counts).
            target_angle (float): Desired angle [rad] at the current position.
        """
        self.calibration_offset = self.steps - int((target_angle * cpr) / (2 * np.pi))
        self.last_steps = self.steps
        print(f"Encoder '{self.id}' calibrated. Current position set to {target_angle:.2f} rad.")
        print(f"  Raw steps: {self.steps}, Calibration offset: {self.calibration_offset}")

    def read_position(self, cpr):
        """
        Return the calibrated angular position wrapped to [-pi, pi].

        The internal step count is shifted by calibration_offset so that
        the calibrated position corresponds to the chosen zero.

        Args:
            cpr (int): Counts per revolution.

        Returns:
            float: Angle in radians, wrapped to [-pi, pi].
        """
        # Apply calibration offset
        calibrated_steps = self.steps - self.calibration_offset
        raw_angle = (2 * np.pi / cpr) * calibrated_steps

        # Wrap to [-pi, pi] using atan2 of sine and cosine (robust modulo 2π)
        wrapped_angle = np.arctan2(np.sin(raw_angle), np.cos(raw_angle))
        return wrapped_angle

    def read_velocity(self, cpr):
        """
        Compute angular velocity from step difference over time.

        The velocity is based on the raw (unwrapped) step count; no wrapping is
        required because velocity is a rate of change, not an absolute angle.

        Args:
            cpr (int): Counts per revolution.

        Returns:
            tuple:
                angular_velocity (float): Angular velocity [rad/s].
                current_time (float): Current timestamp.
                current_steps (int): Current step count.
        """
        current_time = time.time()
        current_steps = self.steps

        time_diff = current_time - self.last_time
        step_diff = current_steps - self.last_steps

        # Avoid division by very small time intervals
        if time_diff < 0.0001:
            return 0, current_time, current_steps

        angular_velocity = (step_diff / time_diff) * (2 * np.pi / cpr)

        # Update history for next call
        self.last_time = current_time
        self.last_steps = current_steps

        return angular_velocity, current_time, current_steps

    def cleanup(self):
        """
        Cancel pigpio callbacks associated with this encoder.

        Should be called during shutdown to avoid resource leaks.
        """
        self.cbA.cancel()
        self.cbB.cancel()
