import scipy as sp
import numpy as np
import control
from matplotlib import pyplot as plt

# Small kalman filter for angle estimation

sample_rate_hz = 50
dt = 1/sample_rate_hz

# Discrete omega decay coefficient
bc = 8E-4            # Continuous

V_S = 23.4
SPEED_NL = 500 * np.pi/30  # RPM to RadsPS
J = 0.004
R = 7
k = V_S / SPEED_NL

Ad = np.array([
    [1, dt],
    [0, 1 - bc*dt/J - k**2*dt / (J*R)]
])

B = np.array([
    [0],
    [k*dt / (J*R)]
])

C = np.array([[1, 0]])
C_full = np.array([[1, 0], [0, 1]])

D = np.array([[0]])
D_full = np.array([[0], [0]])

# Full-state measurement model, for simulation including both theta and omega
full_state_dss = sp.signal.StateSpace(Ad, B, C_full, D_full, dt=dt)

# Process noise system entry
G = np.array([
    [0],     # Process noise theta
    [1]     # Process noise omega
])

# Process noise covariance
QN = np.array([[100]])

# Sensor noise covariance
RN = np.array([[0.001]])

# Estimator gain matrix from solving the A.R.E.
L, P, E = control.dlqe(Ad, G, C, QN, RN)


print("Estimator gains for theta-only model:", *L.ravel())

# Simulate a voltage step
plt.grid()
tf = 5
t = np.linspace(0, tf, int(1 + tf/dt))
u = np.zeros(t.shape)
u[int(t.size/2):] = 24
_, y, _ = sp.signal.dlsim(full_state_dss, u = u, t = t)
plt.plot(t, y[:, 0])
plt.plot(t, y[:, 1])
plt.show()