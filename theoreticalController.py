from lqr import LQR                         # Class for Linear Quadratic Regulator (LQR) control
import numpy as np
import control as ctrl                  # Control system library for Python

# Pendulum Parameters
pi = np.pi              # Constant for pi
m_c = 0.232              # Mass of the cart
m_1 = 0.127              # Mass of the first pendulum
m_2 = 0.127              # Mass of the second pendulum
l_1 = 0.3                # Length of the first pendulum
l_2 = 0.3                # Length of the second pendulum
lc_1 = 0.3               # Length to center of mass of the first pendulum
lc_2 = 0.15              # Length to center of mass of the second pendulum
i_1 = m_1 * lc_1**2        # Moment of inertia of the first pendulum
i_2 = m_2 * lc_2**2        # Moment of inertia of the second pendulum
g = 9.81                # Gravitational acceleration 

# i_ntermedi_ate calculati_ons for the system matrices
h_1 = m_c + m_1 + m_2
h_2 = m_1 * lc_1 + m_2 * l_1
h_3 = m_2 * lc_2
h_4 = m_2 * lc_1**2 + m_2 * l_1**2 + i_1
h_5 = m_2 * lc_2 * l_1
h_6 = m_2 * lc_2**2 + i_2
h_7 = m_1 * lc_1 * g + m_2 * l_1 * g
h_8 = m_2 * lc_2 * g

# System matrix representation
M = np.array([
    [1, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0],
    [0, 0, 0, h_1, h_2, h_3],
    [0, 0, 0, h_2, h_4, h_5],
    [0, 0, 0, h_3, h_5, h_6],
])
N = np.array([
    [0, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 1],
    [0, 0, 0, 0, 0, 0],
    [0, -h_7, 0, 0, 0, 0],
    [0, 0, -h_8, 0, 0, 0],
])
F = np.array([[0], [0], [0], [1], [0], [0]])

# Matrices of the state-space system
A = np.linalg.solve(M, N)
B = np.linalg.solve(M, F)
C = np.array([[1, 0, 0, 0, 0, 0]])
D = np.array([[0]])
t_s = 0.01  # Sampling time
print('A:', A)  # Print the state matrix A
print('B:', B)  # Print the input matrix B
print('C:', C)  # Print the output matrix C
print('D:', D)  # Print the feedthrough matrix D

# Controllablity matrix
Ct = ctrl.ctrb(A, B)  # Compute the controllability matrix
if np.linalg.matrix_rank(Ct) < A.shape[0]:
    raise ValueError("The system is not controllable.")  # Raise an error if the system is not controllable


# LQR Parameters
Q = np.diag([1000, 50, 50, 1000, 10, 10])  # State cost matrix
R = np.array([[100]])  # Control cost matrix
LQR_CONTROLLER = LQR(A, B, C, D, Q, R)  # LQR controller object

# Discrete-time system
sys_C, sys_D = LQR_CONTROLLER.covert_continuous_to_discrete(A, B, C, D, t_s)  # Convert continuous to discrete system
# calculate K_discrete
K_d = LQR_CONTROLLER.compute_K_discrete(Q, R, sys_D)  # Compute the LQR gain for discrete system
print('K_d:', K_d)  # Print the LQR gain
print('Time Constant:',LQR_CONTROLLER.compute_eigenvalues_discrete(Q,R,sys_D,K_d))  # Print the time constant of the system