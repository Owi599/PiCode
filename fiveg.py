# This script implements a 5G-based UDP-like interface for sending sensor
# (rotary encoder) data and receiving control data via AT commands over UART.

from com import COM                               # Custom communication helper (not directly used here)
from gpiozero import RotaryEncoder                # High-level Rotary Encoder interface
from rotaryencoder import ReadRotaryEncoder       # Custom encoder utilities (currently unused)
import numpy as np
import time
import serial
from curses import ascii

# ---------------------------------------------------------------------------
# Rotary encoder configuration
# ---------------------------------------------------------------------------
# Create encoder object using GPIO pins 21 (A) and 20 (B)
encoder = RotaryEncoder(21, 20, max_steps=625, wrap=True)
encoder.steps = 625           # Initialize encoder step counter
cpr = 1250                    # Counts per revolution of the encoder


# ---------------------------------------------------------------------------
# Serial interface to 5G modem (AT command channel)
# ---------------------------------------------------------------------------
# Open UART/USB serial port to the 5G module at 115200 baud
s = serial.Serial("/dev/ttyUSB2", 115200)
rec_buff = ''                 # Temporary receive buffer for AT command responses


def initiateUDP():
    """
    Configure the 5G module and open a UDP socket/session via AT commands.

    Sequence:
      - Basic AT connectivity check
      - Enable echo
      - Open IP network if not already open
      - Open a UDP connection on port 5000 (connection id = 1)
    """
    send_at('AT', 'OK', 1)
    send_at('ATE1', 'OK', 1)
    # Check if network is already open; if not, open it
    if send_at('AT+NETOPEN?', '+NETOPEN: 0', 1):
        send_at('AT+NETOPEN', 'OK', 1)
        time.sleep(0.01)
    # Open UDP channel (ID 1) on local port 5000, remote address/port set later
    send_at('AT+CIPOPEN=1,"UDP",,,5000', 'OK', 1)


def send_at(command, back, timeout):
    """
    Send an AT command to the modem and wait for its response.

    Args:
        command (str): AT command string without CR/LF at the end.
        back (str): Expected substring in the modem response (can be empty).
        timeout (float): Time in seconds to wait for the response.

    Returns:
        str|int: The 'back' string on success, or 0 on error.
    """
    rec_buff = ''
    # Write command terminated by CRLF as required by AT syntax
    s.write((command + '\r\n').encode())
    time.sleep(timeout)

    # If data is available in RX buffer, read it
    if s.inWaiting():
        time.sleep(0.01)
        rec_buff = s.read(s.inWaiting())

    # Check if expected response substring is present
    if back and back not in rec_buff.decode():
        print(command + ' ERROR')
        print(command + ' back:\t' + rec_buff.decode())
        return 0
    else:
        # Print full modem response for debugging/monitoring
        print(rec_buff.decode())
        return back


def readValue(CPR):
    """
    Read the current mechanical position from the rotary encoder.

    Args:
        CPR (int): Counts per revolution (used to scale steps to angle).

    Returns:
        str: Sensor value as string (angle in radians).
    """
    # Block until there is a rotation event
    if encoder.wait_for_rotate():
        # Convert encoder steps to angle in radians
        sensorFloat = (2 * np.pi / CPR * encoder.steps)

        # These if-statements do nothing numerically but keep structure explicit
        if sensorFloat >= 0:
            sensorFloat = sensorFloat
        if sensorFloat < 0:
            sensorFloat = sensorFloat

        sensor = str(sensorFloat)
        return sensor


def ask_ME(CPR):
    """
    Read encoder value and send it as a UDP payload via the 5G modem.

    Args:
        CPR (int): Counts per revolution for scaling encoder steps.
    """
    sensor = readValue(CPR)

    # Build CIPSEND command: connection ID=1, remote IP=10.0.3.55, remote port=664
    AT = 'AT+CIPSEND=1,,"10.0.3.55",664'
    # Initiate send; for some modems, an empty expected response is allowed
    send_at(AT, '', 1)

    # Wait for '>' prompt or readiness (timing is modem-dependent)
    time.sleep(0.5)

    # Send the sensor payload over the established UDP connection
    s.write(sensor.encode())

    # Send Ctrl+Z (ASCII 26) to terminate data input for CIPSEND
    ctrl = chr(26)
    s.write(ctrl.encode())
    time.sleep(0.05)


def CloseConnection():
    """
    Gracefully close UDP connection and network context on the modem.
    """
    send_at('AT+CIPCLOSE=0', 'OK', 1)   # Close connection (ID may differ, check module docs)
    send_at('AT+NETCLOSE', 'OK', 1)     # Close IP network context


# ---------------------------------------------------------------------------
# Main loop:
#  - Initialize UDP over 5G
#  - Periodically send encoder value
#  - Periodically issue receive command to read back any UDP data
# ---------------------------------------------------------------------------
initiateUDP()

while True:
    try:
        # Measure encoder position and send over 5G
        ask_ME(1250)
        time.sleep(0.5)

        # Send newline for readability/log separation (optional)
        s.write(chr(10).encode())

        # Poll modem for received UDP data on active connection
        data = send_at('AT+CIPRXGET=0', '', 1)
        print(data)
    except KeyboardInterrupt:
        # Allow clean exit with Ctrl+C
        break

# Close all modem connections before exiting
CloseConnection()
