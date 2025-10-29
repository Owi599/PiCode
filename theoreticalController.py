from lqr import LQR                   # Class for Linear Quadratic Regulator (LQR) control
import numpy as np
import control as ctrl                # Control system library for Python
import matplotlib.pyplot as plt       # Importing the plotting library
import random as rnd
import math as m
# Pendulum Parameters
pi = np.pi
m_c = 0.6
m_1 = 0.102
m_2 = 0.104
l_1 = 0.28
l_2 = 0.305
lc_1 = 0.17
lc_2 = 0.065
i_1 = m_1 * lc_1**2
i_2 = m_2 * lc_2**2
g = 9.81
b_c = 0.05
b_1 = 0.01
b_2 = 0.01

# Intermediate calculations for the system matrices
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
    [0, 0, 0, -b_c, 0, 0],
    [0, -h_7, 0, 0, -b_1, 0],
    [0, 0, -h_8, 0, 0, -b_2],
])
F = np.array([[0], [0], [0], [1], [0], [0]])

# Matrices of the state-space system
A = np.linalg.solve(M, N)
B = np.linalg.solve(M, F)
C = np.array([[1, 0, 0, 0, 0, 0]])
D = np.array([[0]])
#t_s = 0.01  # Sampling time

# Controllability matrix
Ct = ctrl.ctrb(A, B)
if np.linalg.matrix_rank(Ct) < A.shape[0]:
    raise ValueError("The system is not controllable.")
print('Controllability matrix rank:', np.linalg.matrix_rank(Ct))

def calculate_steps(force):
        #velocity and Speed in RPM
        print('force:', force)
        velocity_integral = 0
        position_integral = 0
        acceleration = force/(0.6 + 0.104 + 0.102)
        print('Acceleration:',acceleration)
        velocity_integral += acceleration*0.02
        print('Velocity before clipping',velocity_integral)
        #Check for maximum speed
        if abs(velocity_integral) > (0.1047*0.0125*2000):
            velocity_integral = np.sign(velocity_integral) * (0.1047*0.0125*2000)
            print('Warning:Speed cannot be greater than 2000 rpm')
        print('Velocity after clipping:',velocity_integral)
        position_integral += velocity_integral*0.02
        print('End position',position_integral)
        steps_int = m.floor(abs((position_integral*200*4)/(2*np.pi*0.0125)))  # Ensures at least 1 step
        print(steps_int)
        stepFreq = (abs(velocity_integral) * 4 * 200 ) / (0.0125*(2*np.pi))   # Frequency in Hz
        # print(stepFreq)
        if stepFreq == 0:
            stepPeriod = 0  # Motor is not moving
        else:
            stepPeriod = 1 / stepFreq
        
        # # Set maximum delay to prevent out-of-bound values
        maxDelay = 0.02   # maximum delay
        #print(maxDelay)
        stepPeriod = float(min(stepPeriod, maxDelay))
        print(stepPeriod)
        
        direction = int(1*np.sign(velocity_integral))  # Determine direction based on velocity sign
        #print(direction)
        return steps_int, stepPeriod,  direction
    
# # Variables to store the best configuration
# min_time_const = float('inf')
# best_q_scale, best_r_scale = None, None
# best_Kd, best_Q, best_R, best_sys_D = None, None, None, None

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

# print('-------------------')
# print(f"Best Q_scale: {best_q_scale}, R_scale: {best_r_scale}, Time constant: {min_time_const}")

# # Best controller configuration
# A_cl = best_sys_D.A - best_sys_D.B @ best_Kd
# sys_C = ctrl.ss(A, B, C, D)
# sys_D = ctrl.c2d(sys_C, t_s)  # Convert to discrete system
# A_d,B_d,C_d,D_d = sys_D.A, sys_D.B, sys_D.C, sys_D.D

# Variables to store the best configuration

desired_poles = [-6.66,-6,-5,-5.66,-7.66,-8]  # Desired poles for the system
K = ctrl.place(A, B, desired_poles)
print(K)
# Best controller configuration
A_cl = A - B @ K
Eigs = np.linalg.eigvals(A_cl)
dominantEigenvalue = min(Eigs, key=lambda ev: abs(ev.real))
timeConstant = 1 / abs(dominantEigenvalue.real)
print(f"Dominant Eigenvalue: {dominantEigenvalue}, Time Constant: {timeConstant:.2f} seconds")

# sys_cl = ctrl.ss(A_cl, best_sys_D.B * 0, np.eye(best_sys_D.A.shape[0]), np.zeros((best_sys_D.A.shape[0], 1)))
sys_cl = ctrl.ss(A_cl, B * 0, np.eye(A.shape[0]), np.zeros((A.shape[0], 1)))

# Initial state and time for simulation
x0 = np.array([0, pi, 0, 0, 0, 0])
T_s = 0.02
t = np.arange(0,10,T_s)
t_start = 0
t_end = t_start + T_s
_, y = ctrl.forced_response(sys_cl, T=t, U=np.zeros_like(t), X0=x0)
U = np.matmul(K,y)

print(U.size)


# # cart_position = y[0,:]
# threshold = 0.001
# # rect_pulse = np.where(np.abs(cart_position)>threshold,1,0)
stepset = [] 
i = 0 
while i < U.size:
    stepset.append(calculate_steps(U[0,i]))
    # stepset[1,i].append(calculate_steps(U[0,i])(1))
    # stepset[2,i].append(calculate_steps(U[0,i])(2))
    i+=1    
print(stepset)


# y_discrete_0 = np.interp(0,t,y[0])
# y_discrete_1 = np.interp(0,t,y[1])
# y_discrete_2 = np.interp(0,t,y[2])
# y_discrete_3 = np.interp(0,t,y[3])
# y_discrete_4 = np.interp(0,t,y[4])
# y_discrete_5 = np.interp(0,t,y[5])
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

