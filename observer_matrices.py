from ulab import numpy as np

# Generated from physical parameters:
#   r=0.035m, w=0.140m, tau=0.05s, Km=3.49 rad/V/s, Ts=0.02s
# L taken directly from MATLAB pole placement (image from HW0x03)
# Observer eigenvalues of (A - L*C): [-0.027, -0.255, -19.99, -20.21]
# NOTE: observer poles are close to plant poles — consider re-tuning L
#       for faster convergence (place poles 3-5x further left than plant)

Ts = 0.02

Ad = np.array([
    [0.999708, 0.000000,  0.000288,  0.000288],
    [0.000000, 0.994911, -0.004098,  0.004098],
    [-0.140110, 0.000000,  0.668880,  0.001394],
    [-0.140110, 0.000000,  0.001394,  0.668880],
])

# Bd is 4x6: columns are [uL, uR, sL_vel, sR_vel, psi, psiDot]
Bd = np.array([
    [0.000215,  0.000215,  0.000146,  0.000146,  0.000000,  0.000000],
    [-0.003055,  0.003055, -0.000353,  0.000353,  0.005039,  0.000057],
    [1.149435,  0.001125,  0.070055,  0.070055, -0.000000, -0.006970],
    [0.001125,  1.149435,  0.070055,  0.070055, -0.000000,  0.006970],
])