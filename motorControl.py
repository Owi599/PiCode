import pigpio
import time
import numpy as np



# Class for controlling motor output
class MotorControl():
    def __init__(self, pi, pulsePin, directionPin, stepsPerRev, pulleyRad,microresFactor, t_sample):
        """
        Initializes the motor control using the pigpio library.
        :param pi: An active pigpio instance.
        """
        if not pi.connected:
            raise RuntimeError("Could not connect to pigpio daemon.")
        
        self.pi = pi
        self.pulsePin = pulsePin
        self.dirPin = directionPin
        
        # --- Pin Setup ---
        self.pi.set_mode(self.pulsePin, pigpio.OUTPUT)
        self.pi.set_mode(self.dirPin, pigpio.OUTPUT)
        self.pi.write(self.pulsePin, 0)
        self.pi.write(self.dirPin, 0)
        
        # --- Physical and Control Parameters ---
        self.stepsPerRev = stepsPerRev
        self.pulleyRad = pulleyRad
        self.t_sample = t_sample
        self.microresolution = microresFactor # This should match your driver's setting
        self.m_total = 0.6 + 0.104 + 0.102 # Total mass kg
        self.velocity_max = (2000 / 60) * (2 * np.pi) * self.pulleyRad # Max vel for 2000 RPM in m/s

        # Integration terms for velocity and position
        self.velocity_integral = 0
        self.position_integral = 0
        self.dt = self.t_sample
        
    def calculate_steps(self, force):
       
        # --- Physics Integration ---
        acceleration = force / self.m_total
        self.velocity_integral += acceleration * self.dt
        
        # Clip velocity to the motor's physical maximum
        if abs(self.velocity_integral) > self.velocity_max:
            self.velocity_integral = np.sign(self.velocity_integral) * self.velocity_max

        self.position_integral += self.velocity_integral * self.dt
        
        # --- Convert desired position to steps ---
        # Total steps from origin to desired new position
        total_target_steps = int((self.position_integral * self.stepsPerRev * self.microresolution) / (2 * np.pi * self.pulleyRad))
        
        steps_to_move = abs(total_target_steps) # Simplified for this example

        # Calculate the frequency (Hz) needed to achieve the target velocity
        if self.velocity_integral == 0:
            step_freq = 0
        else:
            # stepFreq (Hz) = velocity (m/s) / distance_per_step (m)
            distance_per_step = (2 * np.pi * self.pulleyRad) / (self.stepsPerRev * self.microresolution)
            step_freq = abs(self.velocity_integral) / distance_per_step

        direction = int(np.sign(self.velocity_integral))
        
        return steps_to_move, step_freq, direction

    def move_stepper(self, steps: int, frequency: float, direction: int):
        """
        Moves the stepper motor using hardware-timed pulses (waveforms).
        This function is now NON-BLOCKING. It starts the move and returns immediately.
        """
        # Set direction
        if direction == 1:
            self.pi.write(self.dirPin, 1)
        elif direction == -1:
            self.pi.write(self.dirPin, 0)
        
        # If no movement is needed, do nothing
        if steps == 0 or frequency == 0:
            return

        # --- Create Hardware-Timed Waveform ---
        # Calculate the delay for a 50% duty cycle pulse in microseconds
        delay_micros = int(500000 / frequency)
        
        # Define one pulse: pin high, delay, pin low, delay
        pulse = [
            pigpio.pulse(1 << self.pulsePin, 0, delay_micros),
            pigpio.pulse(0, 1 << self.pulsePin, delay_micros)
        ]
        
        # Create a wave from the required number of pulses
        wave = pulse * steps
        
        self.pi.wave_add_generic(wave)
        wave_id = self.pi.wave_create()
        
        if wave_id >= 0:
            # Send the wave. WAVE_MODE_ONE_SHOT executes it once.
            self.pi.wave_tx_send(wave_id, pigpio.WAVE_MODE_ONE_SHOT)
            self.pi.wave_delete(wave_id) # Clean up the wave from pigpio memory

    def is_moving(self):
        return self.pi.wave_tx_busy()

    def stop_motor(self):
        """Stops any current motor movement immediately."""
        print('Motor stopping')
        # Stop any active waveforms
        self.pi.wave_tx_stop()
        # Set pins to low
        self.pi.write(self.pulsePin, 0)
        self.pi.write(self.dirPin, 0)
        print('Motor stopped')
