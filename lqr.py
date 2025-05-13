import numpy as np  # library for nummeric operations
import control as ct # control library 
from scipy.linalg import solve_continuous_are, solve_discrete_are # linearization library 

# Class defining the LQR controller
class LQR: 
    # Constructor for the LQR class
    def __init__(self,A,B,C,D,Q,R):
        self.A = A
        self.B = B
        self.C = C
        self.D = D
        self.Q = Q
        self.R = R


    # Method to convert continuous-time system to discrete-time system
    def covert_continuous_to_discrete(self,A,B,C,D,T_s):
        sys_C = ct.StateSpace(A, B, C, D)
        sys_D = ct.c2d(sys_C, T_s)  # Convert to discrete system
        
        return sys_C ,sys_D  # Extract matrices

    # Method to compute the LQR gain for continuous-time system
    def compute_K_continuous(self,Q,R,sys):
        # Compute the continuous-time algebraic Riccati equation
        P = solve_continuous_are(sys.A, sys.B, Q,R)
        # Compute the LQR gain
        K = np.linalg.inv(R) @ sys.B.T @ P
        return K
    
    # Method to compute the LQR gain for discrete-time system
    def compute_K_discrete(self,Q,R,sys):
        # Compute the discrete-time algebraic Riccati equation
        P_d = solve_discrete_are(sys.A, sys.B, Q, R)
        # Compute the LQR gain
        K_d = np.linalg.inv(R + sys.B.T @ P_d @ sys.B) @ (sys.B.T @ P_d @ sys.A)
        return K_d
   
    # Method to compute the eigenvalues of the continuous closed-loop system
    def compute_eigenvalues(self,Q,R,sys):
        # Compute the eigenvalues of A_cl
        A_cl = sys.A - sys.B @ self.LQR(Q,R,sys)
        eigenvalues = np.linalg.eigvals(A_cl)
        # Find the dominant eigenvalue (the one with the smallest real part magnitude)
        dominantEigenvalue = min(eigenvalues, key=lambda ev: abs(ev.real))
        # Calculate the time constant
        timeConstant = 1 / abs(dominantEigenvalue.real)
        return dominantEigenvalue, timeConstant
    
    # Method to compute the eigenvalues of the discrete closed-loop system
    def compute_eigenvalues_discrete(self,Q,R,sys):
        A_cl_d = sys.A - sys.B @ self.compute_K_discrete(Q,R,sys)
        eigenvalues_d = np.linalg.eigvals(A_cl_d)
        # Find the dominant eigenvalue (the one with the smallest real part magnitude)
        dominantEigenvalue_d = min(eigenvalues_d, key=lambda ev: abs(ev.real))
        # Calculate the time constant
        timeConstant_d = 1 / abs(dominantEigenvalue_d.real)
        return dominantEigenvalue_d, timeConstant_d
    
    # Method to compute the control output for continuous-time system
    def compute_control_output_continuous(self, K, x):
        
        return np.clip(-K @ x, -8.5, +8.5)
   
    # Method to compute the control output for discrete-time system
    def compute_control_output_discrete(self,K_d,x_k):
            
        return np.clip(-K_d @ x_k, -8.5, +8.5)

