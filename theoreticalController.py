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
b_c = 0.05
b_1 = 0.0001
b_2 = 0.0001
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
t_s = 0.01  # Sampling time
print('A:', A)  # Print the state matrix A
print('B:', B)  # Print the input matrix B
print('C:', C)  # Print the output matrix C
print('D:', D)  # Print the feedthrough matrix D

# Controllablity matrix
Ct = ctrl.ctrb(A, B)  # Compute the controllability matrix
if np.linalg.matrix_rank(Ct) < A.shape[0]:
    raise ValueError("The system is not controllable.")  # Raise an error if the system is not controllable
print(np.linalg.matrix_rank(Ct))  # Print the rank of the controllability matrix
# for q_scale in [1e3,2e3,5e3,1e4,2e4,5e4,1e5, 1e6, 1e7, 1e8]:
#     for r_scale in [1e3, 2e3, 5e3, 1e4, 2e4, 5e4, 1e5, 1e6, 1e7, 1e8]:
#         Q = np.diag([q_scale, 1,1 , q_scale,1, 1])  # State cost matrix
#         R = np.array([[r_scale]])  # Control cost matrix
#         LQR_CONTROLLER = LQR(A, B, C, D, Q, R)  # LQR controller object
#         sys_C, sys_D = LQR_CONTROLLER.covert_continuous_to_discrete(A, B, C, D, t_s)  # Convert continuous to discrete system
#         K_d = LQR_CONTROLLER.compute_K_discrete(Q, R, sys_D)  # Compute the LQR gain for discrete system
#         eig, t_const = LQR_CONTROLLER.compute_eigenvalues_discrete(Q, R, sys_D, K_d)
#         print(f"Q_scale: {q_scale}, R_scale: {r_scale} -> T = {round(t_const*1000)}ms")  # Print the scales of Q and R
#         if t_const < 1:
#             print('Target achieved')
#         print('-------------------')
#print(np.linalg.eigvals(A))  # Print the eigenvalues of the state matrix A
desired_poles= [-5,-10.5,-11,-11.5,-12,-12.5]  # Desired poles for the system
K = ctrl.place(A, B, desired_poles)  # Compute the state feedback gain matrix K
print('K:', K)  # Print the state feedback gain matrix K
A_cl = A - B @ K  # Closed-loop system matrix
Eigs = np.linalg.eigvals(A_cl)
print(Eigs)  # Print the eigenvalues of the closed-loop system matrix
dominantEigenvalue = min(Eigs, key=lambda ev: abs(ev.real))
timeConstant = 1 / abs(dominantEigenvalue.real)
print(f"Dominant Eigenvalue: {dominantEigenvalue}, Time Constant: {timeConstant:.2f} seconds")  # Print the dominant eigenvalue and time constant
# sys_cl  = ctrl.ss(A_cl,B*0,np.eye(A.shape[0]),np.zeros((A.shape[0],1)))  # Create a state-space system for the closed-loop system

x0 = np.array([-0.48, pi, pi, 0, 0, 0])  # Initial state of the system
# t = np.linspace(0, 10, 1000)  # Time vector for simulation
# _,y = ctrl.forced_response(sys_cl, T=t, U=np.zeros_like(t), X0=x0)  # Simulate the system response
# import matplotlib.pyplot as plt  # Importing the plotting library
# plt.figure(figsize=(10, 6))  # Create a figure for plotting 
# plt.plot(t, y[0], label='Cart Position (m)')  # Plot the cart position
# plt.plot(t, y[1], label='Pendulum 1 Angle (rad)')  # Plot the first pendulum angle
# plt.plot(t, y[2], label='Pendulum 2 Angle (rad)')  # Plot the second pendulum angle
# plt.xlabel('Time (s)')  # Label for the x-axis
# plt.ylabel('States')  # Label for the y-axis
# plt.title('System Response with State Feedback Control')  # Title of the plot
# plt.legend()  # Show the legend
# plt.grid()  # Add a grid to the plot
# plt.show()  # Display the plot
dt = t_s
steps = int(10 / dt)
x = x0.copy().flatten()
x_history = [x.copy()]
t_sim = [0]
u_history = []

x_min = -0.48
x_max =  0.48

# Limit control logic
limit_triggered = False
limit_direction = 0  # -1 for left, +1 for right

for i in range(steps):
    # Compute control input
    u = float(np.clip(-K @ x, -10.0, 10.0))

    if limit_triggered:
        # If u tries to push further in the same direction → freeze
        if np.sign(u) == limit_direction:
            u = 0.0  # disable control
        else:
            limit_triggered = False  # u reversed direction → resume control

    # Check for hitting limits
    if not limit_triggered:
        if x[0] < x_min:
            x[0] = x_min
            limit_triggered = True
            limit_direction = np.sign(u)  # direction trying to go past left
            u = 0.0
        elif x[0] > x_max:
            x[0] = x_max
            limit_triggered = True
            limit_direction = np.sign(u)  # direction trying to go past right
            u = 0.0

    # System update
    dx = A @ x + (B * u).flatten()
    x = x + dx * dt

    # Log
    x_history.append(x.copy())
    u_history.append(u)
    t_sim.append((i + 1) * dt)

x_history = np.array(x_history)
t_sim = np.array(t_sim)
u_history = np.array(u_history)


import matplotlib.pyplot as plt

plt.figure(figsize=(10, 6))
plt.plot(t_sim, x_history[:, 0], label='Cart Position (m)')
plt.plot(t_sim, x_history[:, 1], label='Pendulum 1 Angle (rad)')
plt.plot(t_sim, x_history[:, 2], label='Pendulum 2 Angle (rad)')
plt.axhline(y=x_min, color='gray', linestyle='--', label='Cart Limit')
plt.axhline(y=x_max, color='gray', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('States')
plt.title('System Response with Direction-Aware Limit Clamping')
plt.legend()
plt.grid(True)
plt.show()

# Optional: Control input plot
plt.figure()
plt.plot(t_sim[:-1], u_history)
plt.title("Control Input u(t)")
plt.xlabel("Time (s)")
plt.ylabel("Force (N)")
plt.grid(True)
plt.show()
