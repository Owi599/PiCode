import RPi.GPIO as GPIO
import time

# Use BCM numbering (GPIO numbers, not physical pin numbers)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


class EndSwitch:
    """
    Simple wrapper for a digital end-stop / limit switch connected to a GPIO pin.
    """

    def __init__(self, pin):
        """
        Initialize the end switch on the given GPIO pin.

        Args:
            pin (int): BCM pin number the switch is connected to.
        """
        self.pin = pin
        # Configure pin as digital input; you may want pull_up_down=GPIO.PUD_UP / PUD_DOWN
        GPIO.setup(self.pin, GPIO.IN)

    def read_sensor(self):
        """
        Read the current logic level of the switch.

        Returns:
            int: 1 if the input is HIGH, 0 if the input is LOW.
        """
        # Single read; no need for an infinite loop here
        if GPIO.input(self.pin):
            return 1
        else:
            return 0

    def is_object_detected(self):
        """
        Convenience method to interpret the sensor reading as an object detection flag.

        Returns:
            bool: True if object is detected, False otherwise.
        """
        # Call the local read function (not GPIO.read) and map value to boolean
        return self.read_sensor() == 0  # or == 1 depending on your wiring (NC/NO)
