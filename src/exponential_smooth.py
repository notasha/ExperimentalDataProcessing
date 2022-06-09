import numpy as np

def estimate_math_expectations(traj_noise, measure_noise):
    N = traj_noise.shape[0]
    # define v and ro
    v = np.empty(N)
    for i in range(1, N):
        v[i] = traj_noise[i] + measure_noise[i] - measure_noise[i-1]

    ro = np.empty(N)
    for i in range(2, N):
        ro[i] = traj_noise[i] + traj_noise[i-1] + measure_noise[i] - measure_noise[i-2]

    # define E[v] and E[ro]
    E_v = np.sum(v**2) / (N - 1)
    E_ro = np.sum(ro**2) / (N - 2)

    return E_ro - E_v, (2*E_v - E_ro) / 2

def get_optimal_coefficient(traj_sigma2, measure_sigma2):
    Xi = traj_sigma2 / measure_sigma2
    return (-Xi + np.sqrt(abs(Xi**2 + 4 * Xi))) / 2

def exponential_smoothing(initial_state, measurements, smooth_coefficient):
    points_num = measurements.shape[0]
    XSmooth = np.empty(points_num)
    XSmooth[0] = initial_state
    for i in range(points_num):
        XSmooth[i] = XSmooth[i-1] + smooth_coefficient * (measurements[i] - XSmooth[i-1])
    return XSmooth