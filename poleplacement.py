import numpy as np
import control as ctrl  # Control system library for Python
import matplotlib.pyplot as plt  # Importing the plotting library
import time  # Importing the time library for performance measurement

class PolePlacement:
    
    def __init__(self, A, B, C, D):
        self.A = A
        self.B = B
        self.C = C
        self.D = D

    def compute_poles(self, desired_poles):
       
        # Check if the system is controllable
        Ct = ctrl.ctrb(self.A, self.B)
        if np.linalg.matrix_rank(Ct) < self.A.shape[0]:
            raise ValueError("The system is not controllable.")
        
        # Compute the state feedback gain matrix K
        K = ctrl.place(self.A, self.B, desired_poles)
        return K

    def compute_eigenvalues_and_time_constant(self, K):
        # Compute the closed-loop system matrix
        A_cl = self.A - self.B @ K
        # Calculate the eigenvalues of the closed-loop system
        eigenvalues = np.linalg.eigvals(A_cl)
        dominantEigenvalue = min(eigenvalues, key=lambda ev: abs(ev.real))
        timeConstant = 1 / abs(dominantEigenvalue.real)
        return eigenvalues, dominantEigenvalue, timeConstant
    # controller not contorller 
    def compute_controller_output(self, K, x):
        # Compute the control input
        startTime = time.perf_counter()
        u = -K @ x
        endTime = time.perf_counter()
        controlOutputCalculationTime = endTime - startTime
        return float(u),controlOutputCalculationTime