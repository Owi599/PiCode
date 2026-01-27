import numpy as np
import control as ctrl  # Control system library for Python
import matplotlib.pyplot as plt  # Importing the plotting library
import time  # Importing the time library for performance measurement


class PolePlacement:
    
    def __init__(self, A, B, C, D):
        """
        Initialize a pole-placement controller wrapper around a state-space model.

        Args:
            A (ndarray): State matrix.
            B (ndarray): Input matrix.
            C (ndarray): Output matrix.
            D (ndarray): Feedthrough matrix.
        """
        self.A = A
        self.B = B
        self.C = C
        self.D = D

    def compute_poles(self, desired_poles):
        """
        Compute state-feedback gain K such that A - B K has the desired poles.

        Args:
            desired_poles (array-like): Desired closed-loop eigenvalues.

        Returns:
            ndarray: State-feedback gain matrix K.

        Raises:
            ValueError: If (A, B) is not controllable.
        """
        # Controllability matrix
        Ct = ctrl.ctrb(self.A, self.B)
        # Check if the rank equals the number of states
        if np.linalg.matrix_rank(Ct) < self.A.shape[0]:
            raise ValueError("The system is not controllable.")
        
        # Pole placement to compute K
        K = ctrl.place(self.A, self.B, desired_poles)
        return K

    def compute_eigenvalues_and_time_constant(self, K):
        """
        Compute closed-loop eigenvalues and dominant time constant.

        Args:
            K (ndarray): State-feedback gain matrix.

        Returns:
            tuple:
                eigenvalues (ndarray): Eigenvalues of A_cl = A - B K.
                dominantEigenvalue (complex): Pole with smallest |Re(λ)|.
                timeConstant (float): Approximate time constant 1/|Re(λ_dom)|.
        """
        # Closed-loop system matrix
        A_cl = self.A - self.B @ K
        # Closed-loop eigenvalues
        eigenvalues = np.linalg.eigvals(A_cl)
        # Dominant eigenvalue (slowest mode in real-part sense)
        dominantEigenvalue = min(eigenvalues, key=lambda ev: abs(ev.real))
        # Time constant from dominant pole
        timeConstant = 1 / abs(dominantEigenvalue.real)
        return eigenvalues, dominantEigenvalue, timeConstant

    # controller not contorller
    def compute_controller_output(self, K, x):
        """
        Compute control law u = -K x and measure computation time.

        Args:
            K (ndarray): State-feedback gain matrix.
            x (array-like): Current state vector.

        Returns:
            tuple:
                u (float): Scalar control input (assumes SISO, first element of K x).
                controlOutputCalculationTime (float): Computation time in seconds.
        """
        startTime = time.perf_counter()
        # State-feedback control computation
        u = -K @ x
        endTime = time.perf_counter()
        controlOutputCalculationTime = endTime - startTime
        # Return scalar (for single-input systems) plus timing
        return float(u), controlOutputCalculationTime
