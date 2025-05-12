from lqr import LQR
import numpy as np
import time
from rotaryencoder import ReadRotaryEncoder
from motorencoder import ReadMotorEncoder
from endswitch import EndSwitch
from motorControl import CTRL
import RPi.GPIO as GPIO
import threading
from collections import deque

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Pins
SwitchPin_1 = 4
SwitchPin_2 = 17
PulsePin = 9
DirPin = 10
Encoder1A = 21
Encoder1B = 20
Encoder2A = 16
Encoder2B = 12
MotorEncoderA = 7
MotorEncoderB = 8

# Pendulum Parameters
pi = np.pi
mc = 0.232
m1 = 0.127
m2 = 0.127
L1 = 0.3
L2 = 0.3
LC1 = 0.3
LC2 = 0.15
I1 = m1 * LC1**2
I2 = m2 * LC2**2
g = 9.81

h1 = mc + m1 + m2
h2 = m1 * LC1 + m2 * L1
h3 = m2 * LC2
h4 = m2 * LC1**2 + m2 * L1**2 + I1
h5 = m2 * LC2 * L1
h6 = m2 * LC2**2 + I2
h7 = m1 * LC1 * g + m2 * L1 * g
h8 = m2 * LC2 * g

M = np.array([
    [1, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0],
    [0, 0, 0, h1, h2, h3],
    [0, 0, 0, h2, h4, h5],
    [0, 0, 0, h3, h5, h6],
])
N = np.array([
    [0, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 1],
    [0, 0, 0, 0, 0, 0],
    [0, -h7, 0, 0, 0, 0],
    [0, 0, -h8, 0, 0, 0],
])
F = np.array([[0], [0], [0], [1], [0], [0]])

A = np.linalg.solve(M, N)
B = np.linalg.solve(M, F)
C = np.array([[1, 0, 0, 0, 0, 0]])
D = np.array([[0]])
T_s = 0.02

Q = np.diag([4000, 50, 50, 100, 10, 10])
R = np.array([[100]])

StepsPerRev = 200
PulleyRad = 0.0125
HoldingTorque = 2
cpr_m = 500
Max_steps = 625
cpr = 1250

# Instantiate hardware components
encoder = ReadRotaryEncoder(Encoder1A, Encoder1B, max_steps=Max_steps, wrap=True)
encoder_2 = ReadRotaryEncoder(Encoder2A, Encoder2B, max_steps=Max_steps, wrap=True)
encoder_m = ReadMotorEncoder(MotorEncoderA, MotorEncoderB, max_steps=0)
Motor = CTRL(PulsePin, DirPin, StepsPerRev, PulleyRad, HoldingTorque)
Controller = LQR(A, B, C, D, Q, R)
sys_C, sys_D = Controller.C2D(A, B, C, D, T_s)
K_d = Controller.LQR_discrete(Q, R, sys_D)

# Variables for velocity
last_steps = encoder.steps
last_steps_2 = encoder_2.steps
last_steps_m = encoder_m.steps
last_time = time.time()
last_time_2 = time.time()
last_time_m = time.time()

# Shared deques and lock
read_queue = deque(maxlen=3)
control_queue = deque(maxlen=3)
lock = threading.Lock()

# Thread 1: Read Sensors
def readSensors():
    global last_time, last_time_2, last_time_m
    global last_steps, last_steps_2, last_steps_m
    while True:
        try:
            X0 = []
            X0.append(encoder_m.readPosition(cpr_m))
            X0.append(encoder.readPosition(cpr))
            X0.append(encoder_2.readPosition(cpr))
            V, last_time_m, last_steps_m = encoder_m.readVelocity(cpr_m, last_time_m, last_steps_m)
            X0.append(V)
            W1, last_time, last_steps = encoder.readVelocity(cpr, last_time, last_steps)
            X0.append(W1)
            W2, last_time_2, last_steps_2 = encoder_2.readVelocity(cpr, last_time_2, last_steps_2)
            X0.append(W2)
            with lock:
                read_queue.appendleft(X0)
                print("Read:", X0)
            time.sleep(T_s)
        except Exception as e:
            print("Read Error:", e)

# Thread 2: Compute Control
def computeControl():
    while True:
        try:
            with lock:
                if len(read_queue) >= 2:
                    x_k_1 = read_queue[1]
                    u_k_1 = Controller.control_output(x_k_1, Q, R, sys_D)
                    control_queue.appendleft(u_k_1)
                    print("Control:", u_k_1)
            time.sleep(T_s)
        except Exception as e:
            print("Compute Error:", e)

# Thread 3: Actuate Motor
def actuateMotor():
    while True:
        try:
            with lock:
                if len(control_queue) >= 2:
                    u_k_2 = control_queue[1]
                    Motor.rotate(u_k_2)  # Replace with actual actuation logic
                    print("Actuation:", u_k_2)
            time.sleep(T_s)
        except Exception as e:
            print("Actuation Error:", e)

# Launch threads
t1 = threading.Thread(target=readSensors)
t2 = threading.Thread(target=computeControl)
t3 = threading.Thread(target=actuateMotor)

t1.start()
t2.start()
t3.start()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    GPIO.cleanup()
    print("Stopped by user.")
