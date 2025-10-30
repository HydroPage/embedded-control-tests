import scipy as sp
import numpy as np
import control
from matplotlib import pyplot as plt
from scipy.signal import StateSpace
import pyperclip

# Small kalman filter for angle estimation

sample_rate_hz = 50
dt = 1/sample_rate_hz

# Discrete omega decay coefficient
bc = 0  # Continuous, approximated

V_S = 24.5
SPEED_NL = 538.58 * np.pi/30  # RPM to RadsPS

G = 18.8        # motor turns per output turn (gear ratio)
J = 0.00075     # effective moment of inertia, including gear ratio multiplication
                # (approximate base J, usually order of e-6 to e-5)

R_wires = 0.8
R = 8 + R_wires    # The resistance seen by the driver (motor + wires going to motor)
k = V_S / SPEED_NL

A = np.array([
    [ 0 ,         1          ],
    [ 0 , -bc/J - k**2/(J*R) ]
])

B = np.array([
    [     0     ],
    [ k / (J*R) ]
])

C = np.array([
    [ 1 , 0 ]
])

D = np.array([
    [0]
])

# Full-state measurement model, for simulation including both theta and omega
dss = StateSpace(A, B, C, D).to_discrete(dt, method='bilinear')
Ad = dss.A
Bd = dss.B

# Process noise system entry
G = np.eye(2)

# Process noise/uncertainty covariance
QN = np.array([
    [np.deg2rad(0.1),  0  ],
    [     0,          0.1 ]
])

# Sensor noise covariance (quantization noise variance in radians)
# Variance of uniformly distributed with width d: d^2 / 12
RN = np.array([
    [ (2*np.pi/600)**2 / 12 ]
])

# Estimator gain matrix from solving the A.R.E.
L, P, E = control.dlqe(Ad, G, C, QN, RN)


print("Estimator gains for theta-only model:", *L.ravel())
print(f"constexpr float L[2][1] = \n{{\n\t{{{L[0,0]}}},\n\t{{{L[1,0]}}}\n}};")
pyperclip.copy(f"constexpr float L[2][1] = \n{{\n\t{{{L[0,0]}}},\n\t{{{L[1,0]}}}\n}};")
print("(Above copied to clipboard)")

print("Eigenvalues of discrete closed loop observer:", *np.linalg.eigvals(Ad - L@C).ravel())
print("Discrete A and B:")
print(Ad)
print(Bd)

# Simulate a voltage step
plt.grid()
tf = 5
t = np.linspace(0, tf, int(1 + tf/dt))
u = np.zeros(t.shape)
u[int(t.size/3):] = V_S
u[int(2*t.size/3):] = 0
tout, yout, xout = sp.signal.dlsim(dss, u = u, t = t)
plt.plot(tout, xout[:, 0], label="Angular position")
plt.plot(tout, xout[:, 1], label="Angular velocity")
plt.legend()
plt.show()