import numpy as np  # Library for numeric operations
import control as ct  # Control systems library
from scipy.linalg import solve_continuous_are, solve_discrete_are  # Riccati solvers
import time  # Time library for performance measurement


# Class defining the LQR controller
class LQR:
    # Constructor for the LQR class
    def __init__(self, A, B, C, D, Q, R):
        """
        Initialize the LQR object with system and cost matrices.

        Args:
            A, B, C, D (ndarray): State-space matrices of the plant.
            Q (ndarray): State weighting matrix in the LQR cost.
            R (ndarray): Input weighting matrix in the LQR cost.
        """
        self.A = A
        self.B = B
        self.C = C
        self.D = D
        self.Q = Q
        self.R = R

    # Method to convert continuous-time system to discrete-time system
    def covert_continuous_to_discrete(self, A, B, C, D, T_s):
        """
        Convert a continuous-time state-space model to a discrete-time one.

        Args:
            A, B, C, D (ndarray): Continuous-time state-space matrices.
            T_s (float): Sampling time.

        Returns:
            tuple: (sys_C, sys_D) continuous-time and discrete-time systems.
        """
        sys_C = ct.StateSpace(A, B, C, D)       # Build continuous-time model
        sys_D = ct.c2d(sys_C, T_s)              # Zero-order hold discretization
        return sys_C, sys_D

    # Method to compute the LQR gain for continuous-time system
    def compute_K_continuous(self, Q, R, sys):
        """
        Compute continuous-time LQR gain matrix K.

        Args:
            Q (ndarray): State weighting matrix.
            R (ndarray): Input weighting matrix.
            sys (ct.StateSpace): Continuous-time state-space model.

        Returns:
            ndarray: State-feedback gain K.
        """
        # Solve continuous-time algebraic Riccati equation
        P = solve_continuous_are(sys.A, sys.B, Q, R)
        # Compute the LQR gain K = R^{-1} B^T P
        K = np.linalg.inv(R) @ sys.B.T @ P
        return K

    # Method to compute the LQR gain for discrete-time system
    def compute_K_discrete(self, Q, R, sys):
        """
        Compute discrete-time LQR gain matrix K_d.

        Args:
            Q (ndarray): State weighting matrix.
            R (ndarray): Input weighting matrix.
            sys (ct.StateSpace): Discrete-time state-space model.

        Returns:
            ndarray: Discrete-time state-feedback gain K_d.
        """
        # Solve discrete-time algebraic Riccati equation
        P_d = solve_discrete_are(sys.A, sys.B, Q, R)
        # Compute discrete-time LQR gain
        K_d = np.linalg.inv(R + sys.B.T @ P_d @ sys.B) @ (sys.B.T @ P_d @ sys.A)
        return K_d

    # Method to compute the eigenvalues of the continuous closed-loop system
    def compute_eigenvalues(self, Q, R, sys):
        """
        Compute dominant eigenvalue and time constant for continuous closed-loop system.

        Args:
            Q (ndarray): State weighting matrix.
            R (ndarray): Input weighting matrix.
            sys (ct.StateSpace): Continuous-time state-space model.

        Returns:
            tuple: (dominantEigenvalue, timeConstant)
        """
        # Compute LQR gain for the given system
        K = self.compute_K_continuous(Q, R, sys)
        # Closed-loop matrix A_cl = A - B K
        A_cl = sys.A - sys.B @ K
        eigenvalues = np.linalg.eigvals(A_cl)
        # Dominant eigenvalue (smallest |Re(λ)| → slowest mode)
        dominantEigenvalue = min(eigenvalues, key=lambda ev: abs(ev.real))
        # Time constant τ ≈ 1 / |Re(λ_dom)|
        timeConstant = 1 / abs(dominantEigenvalue.real)
        return dominantEigenvalue, timeConstant

    # Method to compute the eigenvalues of the discrete closed-loop system
    def compute_eigenvalues_discrete(self, Q, R, sys, K):
        """
        Compute dominant eigenvalue and time constant for discrete closed-loop system.

        Args:
            Q (ndarray): State weighting matrix.
            R (ndarray): Input weighting matrix.
            sys (ct.StateSpace): Discrete-time state-space model.
            K (ndarray): Discrete-time state-feedback gain.

        Returns:
            tuple: (dominantEigenvalue_d, timeConstant_d)
        """
        # Closed-loop matrix for discrete-time system
        A_cl_d = sys.A - sys.B @ K
        eigenvalues_d = np.linalg.eigvals(A_cl_d)
        # Dominant eigenvalue (closest to unit circle in terms of real part magnitude)
        dominantEigenvalue_d = min(eigenvalues_d, key=lambda ev: abs(ev.real))
        # Approximate time constant (interpreting real part as continuous mapping)
        timeConstant_d = 1 / abs(dominantEigenvalue_d.real)
        return dominantEigenvalue_d, timeConstant_d

    # Method to compute the control output for continuous-time system
    def compute_control_output_continuous(self, K, x):
        """
        Compute saturated continuous-time LQR control and its computation time.

        Args:
            K (ndarray): Continuous-time LQR gain.
            x (array-like): Current state vector.

        Returns:
            tuple:
                u (ndarray): Saturated control input vector.
                controlOutputCalculationTime (float): Computation time in seconds.
        """
        startTime = time.perf_counter()
        # State feedback with saturation to actuator limits
        u = np.clip(-K @ x, -8.5, +8.5)
        endTime = time.perf_counter()
        controlOutputCalculationTime = endTime - startTime
        return u, controlOutputCalculationTime

    # Method to compute the control output for discrete-time system
    def compute_control_output_discrete(self, K_d, x_k):
        """
        Compute saturated discrete-time LQR control and its computation time.

        Args:
            K_d (ndarray): Discrete-time LQR gain.
            x_k (array-like): Current discrete-time state vector.

        Returns:
            tuple:
                u (ndarray): Saturated control input vector.
                controlOutputCalculationTime (float): Computation time in seconds.
        """
        startTime = time.perf_counter()
        # Discrete state feedback with saturation
        u = np.clip(-K_d @ x_k, -8.5, +8.5)
        endTime = time.perf_counter()
        controlOutputCalculationTime = endTime - startTime
        return u, controlOutputCalculationTime
