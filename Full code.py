from LQR import lqr
import numpy as np
import time
from rotaryencoder import ReadRotaryEncoder
from motorencoder import ReadMotorEncoder
from PhotoelectricSensor import EndSwitch
from MotorCtrlOutput import CTRL
import RPi.GPIO as GPIO
import threading

GPIO.setmode(GPIO.BCM)  # allow GPIO pin numbering
GPIO.setwarnings(False)

#constant values

#shared variables
data_lock = threading.Lock()
data = {
    'read': None,
    'calculate': None,
    'control': None,
}

#Pins
SwitchPin_1= 4
SwitchPin_2= 17
PulsePin= 9
DirPin= 10
Encoder1A=21
Encoder1B=20
Encoder2A=16
Encoder2B=12
MotorEncoderA= 7
MotorEncoderB = 8

# Parameter defintion for pendel
pi = np.pi
mc = 0.232 # cart mass in Kg
m1 = 0.127 # mass of pendulum arm 1 in Kg
m2 = 0.127 # mass of pendulum arm 2 in Kg
L1 = 0.3   # length of first arm in m
L2 = 0.3   # length of second arm in m
LC1 = 0.3 # in m
LC2 = 0.15 # in m
I1 = m1 * LC1**2 # moment of inertia 1 in kg.m^2
I2 = m2 * LC2**2 # moment of inertia 2 in kg.m^2
g = 9.81 # in m/s^2

# Intermediates
h1 = mc + m1 + m2
h2 = m1 * LC1 + m2 * L1
h3 = m2 * LC2
h4 = m2 * LC1**2 + m2 * L1**2 + I1
h5 = m2 * LC2 * L1
h6 = m2 * LC2**2 + I2
h7 = m1 * LC1 * g + m2 * L1 * g
h8 = m2 * LC2 * g

# Dynamics
M = np.array(
    [
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, h1, h2, h3],
        [0, 0, 0, h2, h4, h5],
        [0, 0, 0, h3, h5, h6],
    ]
)
N = np.array(
    [
        [0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0],
        [0, -h7, 0, 0, 0, 0],
        [0, 0, -h8, 0, 0, 0],
    ]
)
F = np.array([[0], [0], [0], [1], [0], [0]])

# linearized System Matrices
A = np.linalg.solve(M, N)
B = np.linalg.solve(M, F)
C = np.array([[1, 0, 0, 0, 0, 0]])
D = np.array([[0]])
T_s = 0.02  # Sampling time in seconds

#LQR Matrices
Q = np.array(
    [
        [4000, 0, 0, 0, 0, 0],
        [0, 50, 0, 0, 0, 0],
        [0, 0, 50, 0, 0, 0],
        [0, 0, 0, 100, 0, 0],
        [0, 0, 0, 0, 10, 0],
        [0, 0, 0, 0, 0, 10],
    ]
)
R = np.array([[100]])

# Motor Parameters
StepsPerRev = 200
PulleyRad = 0.0125
HoldingTorque = 2
cpr_m = 500

#Encoder Parameters
Max_steps = 625
cpr = 1250

# Object instantiation

# #Endswitch
# EndSwitch = EndSwitch(SwitchPin_1)
# EndSwitch_2 = EndSwitch(SwitchPin_2)

# Encoders
encoder = ReadRotaryEncoder(Encoder1A, Encoder1B, max_steps=Max_steps, wrap=True)
encoder_2 = ReadRotaryEncoder(Encoder2A, Encoder2B, max_steps=Max_steps, wrap=True)
encoder.steps = 625
encoder_2.steps = 0

# Motor
Motor = CTRL(PulsePin, DirPin, StepsPerRev, PulleyRad, HoldingTorque)

# Motor Encoder
encoder_m = ReadMotorEncoder(MotorEncoderA, MotorEncoderB, max_steps=0)

# Controller
Controller = lqr(A, B, C, D, Q, R)
sys_C, sys_D = Controller.C2D(A, B, C, D, T_s)
K_d = Controller.LQR_discrete(Q, R, sys_D)

# main loopp function
def readSenesors():
    while True:
        try:
            with data_lock:
                X0.append(encoder_m.readPosition(cpr_m))
                X0.append(encoder.readPosition(cpr))
                X0.append(encoder_2.readPosition(cpr))
                V, last_time_m, last_steps_m = encoder_m.readVelocity(cpr_m, last_time_m, last_steps_m)
                X0.append(V)
                W1, last_time, last_steps = encoder.readVelocity(cpr, last_time, last_steps)
                X0.append(W1)
                W2, last_time_2, last_steps_2 = encoder_2.readVelocity(cpr, last_time_2, last_steps_2)
                X0.append(W2)
                print('States:', X0)
                data['read'] = X0
                X0 = []  # Reset state vector for next iteration
            time.sleep(0.02)

        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(f"Error: {e}")


def ControlOutput(lqr):
    while True:
        try:
            with data_lock:
                if data['read'] is not None:
                    X0 = data['read']
                # Control output calculation
                control_output = Controller.control_output(X0, Q, R, sys_D)
                print('Control Output:', control_output)
            time.sleep(0.02)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(f"Control output error: {e}")


#Variables

# last step count for velocity calculation
last_steps = encoder.steps
last_steps_2	  = encoder_2.steps
last_steps_m  = encoder_m.steps

# last time for velocity calculation
last_time = time.time()
last_time_2 = time.time()
last_time_m = time.time()

# sampling time
t_sample = 0.02

#empty State vector
X0 = []


#t0 for time measurement
t0 = time.time()
time_array = []


while True: 

    tf = time.time()
    dt = tf-t0
    time_array.append(dt)
    print('Time:', dt)
    t0 = tf
    X0 = []  # Reset state vector for next iteration
    if KeyboardInterrupt:
        break  

time_array = np.array(time_array)
mean = np.mean(time_array)
print(mean)
GPIO.cleanup()
