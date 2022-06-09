import numpy as np

def get_true_trajectory(init_position, points_num, sigma_squared):
    """
    Generating the true trajectory X_i using the random walk model
    """
    #
    SigmaW = np.sqrt(sigma_squared)
    # Create 1xN array of normally distributed random noise with zero math.expectation and variance SigmaSqW
    w = np.random.normal(0, SigmaW, points_num)
    # Create an array for coordinates of the true trajectory
    X = np.empty(points_num)
    X[0] = init_position  # initial coordinate
    for i in range(1, points_num):
        X[i] = X[i-1] + w[i]
    return X, w

def get_measurements(trajectory, sigma_squared):
    """
    Generating measurements Z_i of the process X_i
    """
    N = trajectory.shape[0]
    SigmaH = np.sqrt(sigma_squared)
    # Create 1xN array of normally distributed random noise with zero math. expectation and variance SigmaSqH
    h = np.random.normal(0, SigmaH, N)
    # Create an array of measurements of measurements
    Z = np.empty(N)
    for i in range(N):
        Z[i] = trajectory[i-1] + h[i]
    return Z, h

