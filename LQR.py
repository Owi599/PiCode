import numpy as np  # library for nummeric operations
import control as ct # control library 
import matplotlib.pyplot as plt # visualization library 
from scipy.linalg import solve_continuous_are, solve_discrete_are # linearization library 
import time # time library


class lqr: 
    def __init__(self,A,B,C,D,Q,R):
        self.A = A
        self.B = B
        self.C = C
        self.D = D
        self.Q = Q
        self.R = R



    def C2D(self,A,B,C,D,T_s):
        sys_C = ct.StateSpace(A, B, C, D)
        sys_D = ct.c2d(sys_C, T_s)  # Convert to discrete system
        
        return sys_C ,sys_D  # Extract matrices

    def LQR(self,Q,R,sys):
        # Compute the continuous-time algebraic Riccati equation
        P = solve_continuous_are(sys.A, sys.B, Q,R)
        # Compute the LQR gain
        K = np.linalg.inv(R) @ sys.B.T @ P
        return K
    
    def LQR_discrete(self,Q,R,sys):
        # Compute the discrete-time algebraic Riccati equation
        P_d = solve_discrete_are(sys.A, sys.B, Q, R)
        # Compute the LQR gain
        K_d = np.linalg.inv(R + sys.B.T @ P_d @ sys.B) @ (sys.B.T @ P_d @ sys.A)
        return K_d

    def compute_eigenvalues(self,Q,R,sys):
        # Compute the eigenvalues of A_cl
        A_cl = sys.A - sys.B @ self.LQR(Q,R,sys)
        eigenvalues = np.linalg.eigvals(A_cl)
        # Find the dominant eigenvalue (the one with the smallest real part magnitude)
        dominant_eigenvalue = min(eigenvalues, key=lambda ev: abs(ev.real))
        # Calculate the time constant
        time_constant = 1 / abs(dominant_eigenvalue.real)
        return dominant_eigenvalue, time_constant
    
    def compute_eigenvalues_discrete(self,Q,R,sys):
        A_cl_d = sys.A - sys.B @ self.LQR_discrete(Q,R,sys)
        eigenvalues_d = np.linalg.eigvals(A_cl_d)
        # Find the dominant eigenvalue (the one with the smallest real part magnitude)
        dominant_eigenvalue_d = min(eigenvalues_d, key=lambda ev: abs(ev.real))
        # Calculate the time constant
        time_constant_d = 1 / abs(dominant_eigenvalue_d.real)
        return dominant_eigenvalue_d, time_constant_d

    def control_output(self,x,Q,R,sys):
        
        return np.clip(-self.LQR(Q,R,sys) @ x, -8.5, +8.5)

    def control_output_d(self,x_k,Q,R,sys):
            
        return np.clip(-self.LQR_discrete(Q,R,sys) @ x_k, -8.5, +8.5)

