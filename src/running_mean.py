import numpy as np

def running_mean(measurements, window_size):
    N = measurements.shape[0]
    # fill first (M-1)/2 elements
    m = round((window_size - 1) / 2)
    first_M = np.mean(measurements[:m])
    last_M = np.mean(measurements[-m:])

    Xrm = np.empty(N)
    Xrm[:3] = np.ones(3) * first_M
    Xrm[-3:] = np.ones(3) * last_M
    for i in range(3, N-3):
        Xrm[i] = np.sum(measurements[i-3:i+3]) / window_size

    return Xrm