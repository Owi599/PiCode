from LQR import lqr
import numpy as np
from mat4py import loadmat
import time

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

Controller = lqr(A, B, C, D, Q, R)
sys_C, sys_D = Controller.C2D(A, B, C, D, T_s)
#K = Controller.LQR(Q, R, sys_C)
K_d = Controller.LQR_discrete(Q, R, sys_D)
# dominant_eigenvalue, time_constant = Controller.compute_eigenvalues(Q, R, sys_C)
# dominant_eigenvalue_d, time_constant_d = Controller.compute_eigenvalues_discrete(Q, R, sys_D)
# print('Discrete Eigenvalue:', dominant_eigenvalue_d)
# print('Discrete Time Constant:', time_constant_d)
# print('Continuous Eigenvalue:', dominant_eigenvalue)
# print('Continuous Time Constant:', time_constant)

# x = np.array([[0], [pi], [0], [0], [0], [0]])

# print('Controller Output:', controller_output)

# Load force input from MATLAB file
x = loadmat('x.mat')

x = np.array(x['x'])  # Assuming 'u' is the force vector
n = 0
t0 = time.time()
time_array = []
while n < 1500:
    s = [x[n][0], x[n][1], x[n][2], x[n][3], x[n][4], x[n][5]]
    time.sleep(0.02)
    print('State:', s)
    controller_output = Controller.contol_output_d(s, Q, R, sys_D)
    n +=1
    print('Controller Output:', controller_output)
    tf = time.time()
    dt = tf-t0
    time_array.append(dt)
    print('Time:', dt)
    t0 = tf

time_array = np.array(time_array)
mean = np.mean(time_array)
print(mean)
