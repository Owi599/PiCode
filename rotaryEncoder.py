import pigpio
import time
import numpy as np


class PigpioQuadratureEncoder:
    def __init__(self, pi, gpio_a, gpio_b, cpr, name="enc"):
        """
        Pigpio-based quadrature encoder with 4x decoding.

        Args:
            pi (pigpio.pi): Active pigpio instance.
            gpio_a (int): GPIO pin for channel A.
            gpio_b (int): GPIO pin for channel B.
            cpr (int): Cycles per revolution of encoder (e.g., 1250).
                       Internally uses 4x decoding → effective counts = cpr * 4.
            name (str): Identifier for logging.
        """
        if not pi.connected:
            raise RuntimeError("pigpio not connected")

        self.pi = pi
        self.gpio_a = gpio_a
        self.gpio_b = gpio_b
        self.cpr = cpr * 4          # 4x quadrature decoding (all edges)
        self.name = name

        # Encoder state tracking
        self.steps = 0              # Raw step count (unwrapped)
        self.last_steps = 0         # Previous steps for velocity
        self.last_time = time.time() # Previous timestamp for velocity

        # Quadrature decoder state
        self.lev_a = 0              # Current level of channel A
        self.lev_b = 0              # Current level of channel B
        self.last_gpio = None       # Debounce: last GPIO that changed
        self.state = 0              # Current quadrature state (2 bits)

        # Configure pins as inputs with pull-ups
        self.pi.set_mode(gpio_a, pigpio.INPUT)
        self.pi.set_mode(gpio_b, pigpio.INPUT)
        self.pi.set_pull_up_down(gpio_a, pigpio.PUD_UP)
        self.pi.set_pull_up_down(gpio_b, pigpio.PUD_UP)
        
        # Initialize state from current pin levels
        self.state = (self.pi.read(gpio_a) << 1) | self.pi.read(gpio_b)

        # State transition table for 4x quadrature decoding
        # Maps (old_state, new_state) → step increment (+1 or -1)
        self._tt = {
            (0b00, 0b01): +1, (0b01, 0b11): +1, (0b11, 0b10): +1, (0b10, 0b00): +1,
            (0b00, 0b10): -1, (0b10, 0b11): -1, (0b11, 0b01): -1, (0b01, 0b00): -1,
        }

        # Register callbacks for both edges of both channels
        self.cb_a = self.pi.callback(gpio_a, pigpio.EITHER_EDGE, self._pulse)
        self.cb_b = self.pi.callback(gpio_b, pigpio.EITHER_EDGE, self._pulse)
        
        self.pi.set_glitch_filter(gpio_a,1)
        self.pi.set_glitch_filter(gpio_b,1)
        

    def _pulse(self, gpio, level, tick):
        """
        Quadrature decoding callback.

        Triggered on every edge transition of either channel.
        Uses state machine lookup table for robust direction detection.

        Args:
            gpio (int): GPIO pin that changed.
            level (int): New logic level (0 or 1).
            tick (int): Timestamp from pigpio [microseconds].
        """
        # Update channel levels
        a = self.lev_a = level if gpio == self.gpio_a else self.lev_a
        b = self.lev_b = level if gpio == self.gpio_b else self.lev_b
        
        # Current quadrature state (2 bits: A | B)
        new_state = (a << 1) | b
        
        # Look up step increment from state transition table
        self.steps += self._tt.get((self.state, new_state), 0)
        self.state = new_state
        

    def calibrate(self, target_angle):
        """
        Set the current step count to represent the given target angle.

        This is a direct calibration: the current physical position becomes target_angle.

        Examples:
            - calibrate(0): Upright position → 0 steps
            - calibrate(np.pi): Down position → cpr/2 steps (half revolution)

        Args:
            target_angle (float): Desired angle [rad] at current position.
        """
        self.steps = int(target_angle * self.cpr / (2.0 * np.pi))
        self.last_steps = self.steps
        print(f"[{self.name}] Calibrated to {target_angle:.3f} rad ({np.degrees(target_angle):.1f}°)")
        print(f"  Steps set to: {self.steps}")

    def read_position(self):
        """
        Return angular position wrapped to [-π, π] radians.

        Uses raw (unwrapped) step count → angle, then wraps using modulo arithmetic.

        Returns:
            float: Angle in radians, continuously wrapped to [-π, π].
        """
        # Convert steps to angle (full revolutions allowed)
        angle = (2.0 * np.pi * self.steps) / self.cpr
        
        # Wrap to [-π, π] using fmod (handles negative angles correctly)
        angle = np.fmod(angle, 2.0 * np.pi)
        if angle > np.pi:
            angle -= 2.0 * np.pi
        elif angle < -np.pi:
            angle += 2.0 * np.pi
        
        return angle

    def read_velocity(self):
        """
        Compute angular velocity [rad/s] from step difference over elapsed time.

        Uses raw unwrapped step count (velocity doesn't need wrapping).

        Returns:
            float: Angular velocity [rad/s].
        """
        now = time.time()
        steps_now = self.steps

        dt = now - self.last_time
        dsteps = steps_now - self.last_steps

        if dt <= 0.0:
            return 0.0

        # Velocity = (step rate) * (rad/step)
        vel = (dsteps / dt) * (2.0 * np.pi / self.cpr)

        # Update history for next call
        self.last_time = now
        self.last_steps = steps_now

        return vel

    def cleanup(self):
        """
        Cancel pigpio callbacks to free resources.

        Call this during shutdown to prevent resource leaks.
        """
        self.cb_a.cancel()
        self.cb_b.cancel()
