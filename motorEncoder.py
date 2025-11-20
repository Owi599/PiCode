import pigpio
import time
import numpy as np

class ReadMotorEncoder:
    def __init__(self, clk_gpio, dt_gpio, pi, wheel_radius=0.0125):
        """
        Initializes the encoder for measuring linear motion using direct pigpio callbacks.
        """
        if not pi.connected:
            raise RuntimeError("pigpio not connected")
            
        self.pi = pi
        self.clk_gpio = clk_gpio
        self.dt_gpio = dt_gpio
        self.steps = 0
        self.wheel_radius = wheel_radius
        self.last_time = time.time()
        self.last_steps = 0
        
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

    def calibrate(self):
        """
        Calibrates the encoder, setting the current physical position to be zero meters.
        """
        self.steps = 0
        self.last_steps = 0
        print("Cart position calibrated to 0 meters.")

    def read_position(self, cpr):
        """
        Calculates the linear position of the cart in meters.
        """
        circumference = 2 * np.pi * self.wheel_radius
        distance_per_step = circumference / cpr
        position = self.steps * distance_per_step
        return position

    def read_velocity(self, cpr):
        """
        Calculates the linear velocity of the cart in meters per second.
        """
        current_time = time.time()
        current_steps = self.steps

        time_diff = current_time - self.last_time
        step_diff = current_steps - self.last_steps

        if time_diff < 0.0001:
            return 0, current_time, current_steps

        circumference = 2 * np.pi * self.wheel_radius
        distance_per_step = circumference / cpr
        velocity = (step_diff / time_diff) * distance_per_step

        self.last_time = current_time
        self.last_steps = current_steps

        return velocity, current_time, current_steps

    def cleanup(self):
        """Cleans up the callbacks."""
        self.cbA.cancel()
        self.cbB.cancel()
