import pigpio
import time
import numpy as np


# Class for controlling motor output via pigpio waveforms
class MotorControl():
    def __init__(self, pi, pulsePin, directionPin, stepsPerRev, pulleyRad, microresFactor, t_sample):
        """
        Initialize the motor control using the pigpio library.

        Args:
            pi (pigpio.pi): Active pigpio instance.
            pulsePin (int): GPIO pin used as step/clock output.
            directionPin (int): GPIO pin used as direction output.
            stepsPerRev (int): Full steps per mechanical revolution of the motor.
            pulleyRad (float): Pulley radius in meters.
            microresFactor (int): Microstepping factor (must match driver setting).
            t_sample (float): Control loop sampling time [s].
        """
        if not pi.connected:
            raise RuntimeError("Could not connect to pigpio daemon.")

        self.pi = pi
        self.pulsePin = pulsePin
        self.dirPin = directionPin

        # --- Pin setup ---
        self.pi.set_mode(self.pulsePin, pigpio.OUTPUT)
        self.pi.set_mode(self.dirPin, pigpio.OUTPUT)
        self.pi.write(self.pulsePin, 0)
        self.pi.write(self.dirPin, 0)

        # --- Physical and control parameters ---
        self.stepsPerRev = stepsPerRev
        self.pulleyRad = pulleyRad
        self.t_sample = t_sample
        self.microresolution = microresFactor      # Microstep resolution (driver DIP setting)
        self.m_total = 0.6 + 0.104 + 0.102        # Total moving mass [kg] (cart + links)
        # Max cart velocity at 2000 rpm (converted to rad/s then to linear velocity)
        self.velocity_max = (2000 / 60) * (2 * np.pi) * self.pulleyRad

        # Integration states for simple 1D motion model
        self.velocity_integral = 0                # Integrated velocity [m/s]
        self.position_integral = 0                # Integrated position [m]
        self.dt = self.t_sample

    def calculate_steps(self, force):
        """
        Integrate a simple point-mass model and convert to stepper movement.

        Args:
            force (float): Desired cart force [N] (from controller output).

        Returns:
            tuple:
                steps_to_move (int): Number of steps to move from current state.
                step_freq (float): Step frequency [Hz] for the move.
                direction (int): Direction sign (+1 forward, -1 backward, 0 idle).
        """
        # --- Physics integration: F = m * a → a = F / m_total ---
        acceleration = force / self.m_total
        self.velocity_integral += acceleration * self.dt

        # Clip velocity to physical motor limit
        if abs(self.velocity_integral) > self.velocity_max:
            self.velocity_integral = np.sign(self.velocity_integral) * self.velocity_max

        # Integrate position from velocity
        self.position_integral += self.velocity_integral * self.dt

        # --- Convert desired position to steps from origin ---
        # Total steps corresponding to the integrated position
        total_target_steps = int(
            (self.position_integral * self.stepsPerRev * self.microresolution)
            / (2 * np.pi * self.pulleyRad)
        )

        # For now, just use absolute value as movement amount;
        # more advanced versions can track previous target to get delta-steps.
        steps_to_move = abs(total_target_steps)

        # --- Compute step frequency from desired linear velocity ---
        if self.velocity_integral == 0:
            step_freq = 0
        else:
            # Distance per microstep [m]
            distance_per_step = (2 * np.pi * self.pulleyRad) / (self.stepsPerRev * self.microresolution)
            # step_freq [Hz] = v [m/s] / distance_per_step [m]
            step_freq = abs(self.velocity_integral) / distance_per_step

        # Direction based on sign of cart velocity
        direction = int(np.sign(self.velocity_integral))

        return steps_to_move, step_freq, direction

    def move_stepper(self, steps: int, frequency: float, direction: int):
        """
        Start a hardware-timed stepper move using pigpio waveforms (non-blocking).

        Args:
            steps (int): Number of steps to execute.
            frequency (float): Step frequency [Hz].
            direction (int): 1 for forward, -1 for reverse.
        """
        # Set direction pin according to desired sign
        if direction == 1:
            self.pi.write(self.dirPin, 1)
        elif direction == -1:
            self.pi.write(self.dirPin, 0)

        # If no movement is requested, return immediately
        if steps == 0 or frequency == 0:
            return

        # --- Build hardware-timed waveform ---
        # 50% duty-cycle: high and low each take delay_micros microseconds
        delay_micros = int(500000 / frequency)  # half period in microseconds

        # One step pulse: high for delay, then low for delay
        pulse = [
            pigpio.pulse(1 << self.pulsePin, 0, delay_micros),
            pigpio.pulse(0, 1 << self.pulsePin, delay_micros),
        ]

        # Repeat the pulse 'steps' times
        wave = pulse * steps

        # Create and transmit the waveform
        self.pi.wave_add_generic(wave)
        wave_id = self.pi.wave_create()

        if wave_id >= 0:
            # One-shot execution: run once then stop
            self.pi.wave_tx_send(wave_id, pigpio.WAVE_MODE_ONE_SHOT)
            # Remove wave definition from pigpio to free memory
            self.pi.wave_delete(wave_id)

    def is_moving(self):
        """
        Check whether a waveform (step sequence) is still being played.

        Returns:
            bool: True if motor is currently executing a move, False otherwise.
        """
        return self.pi.wave_tx_busy()

    def stop_motor(self):
        """
        Immediately stop any ongoing motor movement and reset pins.
        """
        print('Motor stopping')
        # Stop any active waveform transmission
        self.pi.wave_tx_stop()
        # Set outputs low (no steps, no direction)
        self.pi.write(self.pulsePin, 0)
        self.pi.write(self.dirPin, 0)
        print('Motor stopped')
