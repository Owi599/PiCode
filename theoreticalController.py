from lqr import LQR                   # Class for Linear Quadratic Regulator (LQR) control [web:1]
import numpy as np
import control as ctrl                # Control system library for Python [web:6]
import matplotlib.pyplot as plt       # Importing the plotting library
import random as rnd
import math as m
from motorControl import MotorControl

# Pendulum Parameters [web:4]
pi = np.pi
m_c = 0.6          # Cart mass (kg)
m_1 = 0.102        # Mass of first pendulum (kg)
m_2 = 0.104        # Mass of second pendulum (kg)
l_1 = 0.28         # Length of first pendulum (m)
l_2 = 0.305        # Length of second pendulum (m)
lc_1 = 0.17        # Center of mass distance for first pendulum (m)
lc_2 = 0.065       # Center of mass distance for second pendulum (m)
i_1 = m_1 * lc_1**2  # Moment of inertia for first pendulum (kg*m^2)
i_2 = m_2 * lc_2**2  # Moment of inertia for second pendulum (kg*m^2)
g = 9.81           # Gravitational acceleration (m/s^2)
b_c = 0.05         # Cart damping coefficient
b_1 = 0.01         # First pendulum damping coefficient
b_2 = 0.01         # Second pendulum damping coefficient

# Intermediate calculations for the system matrices (derived from linearized equations of motion using Lagrangian mechanics) [web:16][web:4]
h_1 = m_c + m_1 + m_2
h_2 = m_1 * lc_1 + m_2 * l_1
h_3 = m_2 * lc_2
h_4 = m_2 * lc_1**2 + m_2 * l_1**2 + i_1
h_5 = m_2 * lc_2 * l_1
h_6 = m_2 * lc_2**2 + i_2
h_7 = m_1 * lc_1 * g + m_2 * l_1 * g
h_8 = m_2 * lc_2 * g

# System matrix representation: M * \ddot{q} = N * q + F * u, where q = [x, theta1, theta2, \dot{x}, \dot{theta1}, \dot{theta2}]^T
# M: mass/inertia matrix, N: damping and gravity terms, F: input (force on cart) matrix [web:16]
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
    [0, 0, 0, -b_c, 0, 0],
    [0, -h_7, 0, 0, -b_1, 0],
    [0, 0, -h_8, 0, 0, -b_2],
])
F = np.array([[0], [0], [0], [1], [0], [0]])

# Matrices of the state-space system: \dot{x} = A x + B u, solved as A = M^{-1} N, B = M^{-1} F [web:16]
A = np.linalg.solve(M, N)
B = np.linalg.solve(M, F)
C = np.array([[1, 0, 0, 0, 0, 0]])  # Output: cart position
D = np.array([[0]])
# t_s = 0.01  # Sampling time (commented out)

# Controllability matrix: verifies if the system can be controlled with full state feedback [web:6]
Ct = ctrl.ctrb(A, B)
if np.linalg.matrix_rank(Ct) < A.shape[0]:
    raise ValueError("The system is not controllable.")
print('Controllability matrix rank:', np.linalg.matrix_rank(Ct))

# Hardware setup for stepper motor controlling the cart
pulsePin = 9
dirPin = 10
stepsPerRev = 200
pulleyRad = 0.0125     # Pulley radius (m)
holdingTorque = 2      # Motor holding torque (Nm)
t_s = 0.02             # Sampling time (s)
MOTOR = MotorControl(pulsePin, dirPin, stepsPerRev, pulleyRad, holdingTorque, t_s)  # Motor control object

# Commented out: LQR sweep and selection (brute-force tuning of Q/R weights for optimal time constant) [web:1][web:7]
# Variables to store the best configuration
# min_time_const = float('inf')
# best_q_scale, best_r_scale = None, None
# best_Kd, best_Q, best_R, best_sys_D = None, None, None, None
#
# # LQR sweep and selection
# for q_scale in [1e3, 2e3, 5e3, 1e4, 2e4, 5e4, 1e5, 1e6, 1e7, 1e8]:
#     for r_scale in [1e3, 2e3, 5e3, 1e4, 2e4, 5e4, 1e5, 1e6, 1e7, 1e8]:
#         Q = np.diag([q_scale, 1, 1, q_scale, 1, 1])
#         R = np.array([[r_scale]])
#         LQR_CONTROLLER = LQR(A, B, C, D, Q, R)
#         sys_C, sys_D = LQR_CONTROLLER.covert_continuous_to_discrete(A, B, C, D, t_s)
#         K_d = LQR_CONTROLLER.compute_K_discrete(Q, R, sys_D)
#         eig, t_const = LQR_CONTROLLER.compute_eigenvalues_discrete(Q, R, sys_D, K_d)
#         print(f"Q_scale: {q_scale}, R_scale: {r_scale} -> T = {round(t_const*1000)}ms")
#         if t_const < min_time_const:
#             min_time_const = t_const
#             best_q_scale = q_scale
#             best_r_scale = r_scale
#             best_Kd = K_d
#             best_Q = Q
#             best_R = R
#             best_sys_D = sys_D
#
# print('-------------------')
# print(f"Best Q_scale: {best_q_scale}, R_scale: {best_r_scale}, Time constant: {min_time_const}")

# # Best controller configuration (for LQR discrete)
# A_cl = best_sys_D.A - best_sys_D.B @ best_Kd
# sys_C = ctrl.ss(A, B, C, D)
# sys_D = ctrl.c2d(sys_C, t_s)  # Convert to discrete system
# A_d,B_d,C_d,D_d = sys_D.A, sys_D.B, sys_D.C, sys_D.D

# Pole placement using Ackermann's formula: computes state feedback gain K such that eigenvalues of A - B K are at desired_poles [web:6][web:20][web:23]
desired_poles = [-6.66,-6,-5,-5.66,-7.66,-8]  # Desired closed-loop poles (all stable in left half-plane)
K = ctrl.place(A, B, desired_poles)
print(K)

# Closed-loop system matrix
A_cl = A - B @ K
Eigs = np.linalg.eigvals(A_cl)
dominantEigenvalue = min(Eigs, key=lambda ev: abs(ev.real))  # Pole closest to imaginary axis determines settling time
timeConstant = 1 / abs(dominantEigenvalue.real)
print(f"Dominant Eigenvalue: {dominantEigenvalue}, Time Constant: {timeConstant:.2f} seconds")

# Closed-loop state-space model (no input for simulation)
sys_cl = ctrl.ss(A_cl, B * 0, np.eye(A.shape[0]), np.zeros((A.shape[0], 1)))

# Initial state and time for simulation: pendulum 1 starts inverted (pi rad), others zero
x0 = np.array([0, pi, 0, 0, 0, 0])
T_s = 1                    # Note: Large step size (1s), typically smaller for accuracy
t = np.arange(0,10,T_s)
t_start = 0
t_end = t_start + T_s
_, y = ctrl.forced_response(sys_cl, T=t, U=np.zeros_like(t), X0=x0)  # Simulate open-loop response of closed-loop dynamics? Wait, U=0 but with state feedback, should compute U=K(y) iteratively, but here approximate full response
U = np.matmul(K,y)         # Compute control input from state feedback (for reference)

print(U.size)

# # cart_position = y[0,:]
# threshold = 0.001
# # rect_pulse = np.where(np.abs(cart_position)>threshold,1,0)

# Convert continuous control signal U to motor steps for hardware implementation
stepset = [] 
i = 0 
while i < U.size:
    stepset.append(MOTOR.calculate_steps(U[0,i]))  # Calculate steps needed for each control value
    i+=1     
print(stepset)

# # Interpolation for discrete states at specific times (commented)
# y_discrete_0 = np.interp(0,t,y[0])
# y_discrete_1 = np.interp(0,t,y[1])
# y_discrete_2 = np.interp(0,t,y[2])
# y_discrete_3 = np.interp(0,t,y[3])
# y_discrete_4 = np.interp(0,t,y[4])
# y_discrete_5 = np.interp(0,t,y[5])

# Plot motor steps (control effort)
plt.figure(figsize=(10, 6))
plt.plot(t,stepset)
# plt.step(t, cart_position, where='post', label='Rectangular Pulse (thresholded)')
# plt.plot(t, y_discrete_0, label='Cart Position (m)')
# plt.plot(t_discrete, y_discrete_3, label='Cart Velocity (m/s)')
# plt.plot(t_discrete, y_discrete_1, label='Pendulum 1 Angle (rad)')
# plt.plot(t_discrete, y_discrete_4, label='Pendulum 1 Angular Velocity (rad/s)')
# plt.plot(t_discrete, y_discrete_2, label='Pendulum 2 Angle (rad)')
# plt.plot(t_discrete, y_discrete_5, label='Pendulum 2 Angular Velocity (rad/s)')
# plt.xlabel('Time (s)')
# plt.ylabel('States')
# plt.title('System Response with State Feedback Control ')
plt.legend()
plt.grid()
plt.show()
