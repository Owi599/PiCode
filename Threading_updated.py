from LQR import lqr
import numpy as np
from mat4py import loadmat
import time
import threading
from threading import Event, Lock
import queue

# Use events for synchronization
data_ready = Event()
control_ready = Event()

# Use a queue for thread-safe data sharing
data_queue = queue.Queue()
control_queue = queue.Queue()

class LQR_thread(lqr):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)  # Properly initialize parent class
        
    def contol_output_d(self, Q, R, sys):  # Fixed spelling to match original
        try:
            while True:
                # Wait for data to be ready
                data_ready.wait()
                data_ready.clear()
                
                # Get data safely
                x = data_queue.get_nowait()
                
                # Calculate control output
                u = np.clip(-self.LQR_discrete(Q, R, sys) @ x, -8.5, +8.5)
                
                # Share result safely
                control_queue.put(u)
                control_ready.set()
                print(f"Control output calculated: {u}")  # Debugging statement
                
        except queue.Empty:
            pass
        except Exception as e:
            print(f"Control thread error: {e}")

# Parameter definition for pendulum
pi = np.pi
mc = 0.232  # cart mass in Kg
m1 = 0.127  # mass of pendulum arm 1 in Kg
m2 = 0.127  # mass of pendulum arm 2 in Kg
L1 = 0.3    # length of first arm in m
L2 = 0.3    # length of second arm in m
LC1 = 0.3   # in m
LC2 = 0.15  # in m
I1 = m1 * LC1**2  # moment of inertia 1 in kg.m^2
I2 = m2 * LC2**2  # moment of inertia 2 in kg.m^2
g = 9.81  # in m/s^2

def readValues(x):
    time_array = []  # Define before use
    t0 = time.time()
    
    try:
        for n in range(len(x)):
            s = [x[n][0], x[n][1], x[n][2], x[n][3], x[n][4], x[n][5]]
            
            # Share data safely
            data_queue.put(s)
            data_ready.set()
            print(f"Data sent to control thread: {s}")  # Debugging statement
            
            # Wait for control output
            control_ready.wait()
            control_ready.clear()
            
            time.sleep(0.01)
            
            tf = time.time()
            dt = tf - t0
            time_array.append(dt)
            t0 = tf
            
    except Exception as e:
        print(f"Read values error: {e}")
    finally:
        return np.mean(time_array) if time_array else 0

def Move():
    try:
        while True:
            # Wait for control output
            control_ready.wait()
            control_ready.clear()
            
            # Get control value safely
            control_value = control_queue.get_nowait()
            
            # Ensure control_value is a scalar before using it
            control_value = control_value.item() if isinstance(control_value, np.ndarray) else control_value
            
            for i in range(int(control_value)):
                print('Moving')
                time.sleep(0.01)
                
            data_ready.set()
            print("Data ready for next iteration.")  # Debugging statement
            
    except queue.Empty:
        pass
    except Exception as e:
        print(f"Move thread error: {e}")

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

# Linearized System Matrices
A = np.linalg.solve(M, N)
B = np.linalg.solve(M, F)
C = np.array([[1, 0, 0, 0, 0, 0]])
D = np.array([[0]])
T_s = 0.02  # Sampling time in seconds

# LQR Matrices
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

# Create Controller instance
Controller = LQR_thread(A, B, C, D, Q, R)  # Changed to LQR_thread
sys_C, sys_D = Controller.C2D(A, B, C, D, T_s)
K_d = Controller.LQR_discrete(Q, R, sys_D)

# Load force input from MATLAB file
x = loadmat('x.mat')
x = np.array(x['x'])  # Assuming 'x' is the state vector

control_thread = None
move_thread = None

try:
    # Create threads with bound method
    control_thread = threading.Thread(target=Controller.contol_output_d, args=(Q, R, sys_D))
    move_thread = threading.Thread(target=Move)
    
    # Start threads
    control_thread.start()
    move_thread.start()
    
    # Main process
    readValues(x)
    
finally:
    # Cleanup - only join if threads were created
    if control_thread:
        control_thread.join(timeout=1)
    if move_thread:
        move_thread.join(timeout=1)
