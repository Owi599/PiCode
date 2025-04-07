from LQR import lqr
import numpy as np
from mat4py import loadmat
import time
import threading

shared_value = None
shared_value2 = None
lock = threading.Lock()  # Create a lock for synchronization

class LQR_thread(lqr):
    def control_output_d(self, Q, R, sys):
        global shared_value
        global shared_value2
        x = shared_value
        # Calculate the control output using the LQR controller
        u = np.clip(-self.LQR_discrete(Q, R, sys) @ x, -8.5, +8.5)
        with lock:  # Acquire the lock before modifying shared_value2
            shared_value2 = u
        print('Control Output:', u)
        return u

# Parameter definition for pendulum
pi = np.pi
mc = 0.232  # cart mass in Kg
m1 = 0.127  # mass of pendulum arm 1 in Kg
m2 = 0.127  # mass of pendulum arm 2 in Kg
L1 = 0.3    # length of first arm in m
L2 = 0.3    # length of second arm in m
LC1 = 0.3   # in m
LC2 = 0.15  # in m
I1 = m1 * LC1**2  # moment of inertia 1 in kg.m^2
I2 = m2 * LC2**2  # moment of inertia 2 in kg.m^2
g = 9.81  # in m/s^2

def readValues(x, n):
    global shared_value
    s = [x[n][0], x[n][1], x[n][2], x[n][3], x[n][4], x[n][5]]
    with lock:  # Acquire the lock before modifying shared_value
        shared_value = s
    return s

def Move():
    global shared_value2
    for i in range(int(shared_value2)):
        print('Moving')
        time.sleep(0.01)

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

# Linearized System Matrices
A = np.linalg.solve(M, N)
B = np.linalg.solve(M, F)
C = np.array([[1, 0, 0, 0, 0, 0]])
D = np.array([[0]])
T_s = 0.02  # Sampling time in seconds

# LQR Matrices
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

Controller = lqr(A, B, C, D, Q, R)
sys_C, sys_D = Controller.C2D(A, B, C, D, T_s)
K_d = Controller.LQR_discrete(Q, R, sys_D)

# Load force input from MATLAB file
x = loadmat('x.mat')
x = np.array(x['x'])  # Assuming 'u' is the force vector
n = 0
t0 = time.time()
time_array = []

# Create threads
control_thread = threading.Thread(target=Controller.contol_output_d, args=(Q, R, sys_D))
move_thread = threading.Thread(target=Move)

# Start threads
control_thread.start()
move_thread.start()

while n < 1500:
    print(readValues(x, n))
    time.sleep(0.01)
    n += 1
    tf = time.time()
    dt = tf - t0
    time_array.append(dt)
    print('Time:', dt)
    t0 = tf

# Wait for threads to complete
control_thread.join()
move_thread.join()

time_array = np.array(time_array)
mean = np.mean(time_array)
print(mean)
