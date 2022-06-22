import numpy as np

def get_random_walk_true_trajectory(init_position, points_num, sigma2):
    """
    Generating the true trajectory X_i using the random walk model
    """
    SigmaW = np.sqrt(sigma2)
    # Create 1xN array of normally distributed random noise with zero math.expectation and variance SigmaSqW
    w = np.random.normal(0, SigmaW, points_num)
    # Create an array for coordinates of the true trajectory
    X = np.empty(points_num)
    X[0] = init_position  # initial coordinate
    for i in range(1, points_num):
        X[i] = X[i-1] + w[i]
    return X, w

def get_measurements(trajectory, sigma2):
    """
    Generating measurements Z_i of the process X_i
    """
    N = trajectory.shape[0]
    SigmaH = np.sqrt(sigma2)
    # Create 1xN array of normally distributed random noise with zero math. expectation and variance SigmaSqH
    h = np.random.normal(0, SigmaH, N)
    # Create an array of measurements
    Z = trajectory + h
    return Z, h

def get_true_trajectory_with_random_acceleration(init_position, init_velocity, time_step, points_num,
                                                 acceleration_sigma2):
    # Acceleration
    SigmaA = np.sqrt(acceleration_sigma2)  # variance of noise a_i
    a = np.random.normal(0, SigmaA, points_num)  # normally distributed random acceleration
    # Velocity
    V = np.empty(points_num)
    V[0] = init_velocity  # initial velocity
    for i in range(1, points_num):
        V[i] = V[i - 1] + a[i - 1] * time_step
    # Coordinate
    X = np.empty(points_num)
    X[0] = init_position  # initial coordinate
    for i in range(1, points_num):
        X[i] = X[i-1] + V[i-1] * time_step + (a[i-1] * time_step**2) / 2
    return X
