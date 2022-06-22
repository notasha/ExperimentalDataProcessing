from data_generation import *

def kalman_filter (N, X_0, T, measurements, P_00, sigma_a2, sigma_measure2):
    # parameters
    F = np.array([[1, T], [0, 1]]) # matrix 2x2
    G = np.array([T**2/2, T]) # matrix 2x1
    H = np.array([1, 0]) # matrix 1x2
    Q = np.matmul(G, G.T) * sigma_a2 # matrix 2x2
    R = sigma_measure2
    K = np.zeros([2, N]) # matrix 2x200

    # initial values
    # prediction
    x_pred = np.zeros([2, N]) # matrix of estimates 2x200
    x_pred[:,0] = np.dot(F, X_0)  # the first prediction
    P_pred = np.zeros([2,2,N]) # matrix of errors 2x2x200
    P_pred[:,:,0] = P_00  # error of the first prediction
    K[:,0] = np.dot(P_pred[:,:,0], H.T) / (np.dot(H, np.dot(P_pred[:,:,0], H.T)) + R)
    # filtration
    x_filt = np.zeros([2,N])  # matrix of estimates 2x200
    x_filt[:,0] = x_pred[:,0]
    P_filt = np.zeros([2,2,N]) # matrix of errors 2x2x200
    P_filt[:,:,0] = P_00
    sigma_k = np.zeros(N)
    sigma_k[0] = np.sqrt(P_filt[0,0,0])

    for i in range(1, N):
        # prediction
        x_pred[:,i] = np.dot(F, x_filt[:,i-1])
        P_pred[:,:,i] = np.dot(F, np.dot(P_filt[:,:,i-1], F.T)) + Q
        # filtration
        K[:, i] = np.dot(P_pred[:,:,i],H) / (np.dot(H, np.dot(P_pred[:,:,i],H.T)) + R)
        x_filt[:,i] = x_pred[:,i] + K[:,i]*(measurements[i] - np.dot(H, x_pred[:,i]))
        P_filt[:,:,i] = np.dot((np.eye(2) - np.outer(K[:,i], H)), P_pred[:,:,i])
        sigma_k[i] = np.sqrt(P_filt[0,0,i])

    return x_filt, K, sigma_k

# Function for m-step extrapolation
def extrapolation (m, T, data):
    F = np.array([[1, T], [0, 1]])
    F_m = np.linalg.matrix_power(F, m)
    extr = np.zeros(data.shape)
    for i in range(data.shape[1]-m):
        extr[:,i+m-1] = np.dot(F_m, data[:,i])
    return extr

# Function for calculation errors on M runs of Kalman filter
def mean_errors (runs, size, time_step, sigma_a, sigma_measure, X_0, P_00, m):
    filt_err = np.zeros([runs, size])
    final_filt_err = np.zeros(size)
    extr_err = np.zeros([runs, size])
    final_extr_err = np.zeros(size)
    for i in range(runs):
        # generate true trajectory
        x = trajectory(size, time_step, initial_v=1, initial_x=5, sigma_a=sigma_a)
        # generate measurements
        z = measurements(x, sigma_measure)
        # perform Kalman filter
        x_filt, K, sigma_k = kalman_filter(size, X_0, time_step, z, P_00, sigma_a, sigma_measure)
        # perform extrapolation
        x_extr = extrapolation(m, time_step, x_filt)
        # mean-squared error of estimation
        filt_err[i,:] = (x - x_filt[0,:])**2
        extr_err[i,:] = (x - x_extr[0,:])**2
    # mean errors starting from 3 step
    for i in range(2, size):
        final_filt_err[i] = np.sqrt(np.mean(filt_err[:,i]))
        final_extr_err[i] = np.sqrt(np.mean(extr_err[:,i]))
    return final_filt_err, final_extr_err

# Function for m-step extrapolation
def extrapolation (m, T, data):
    F = np.array([[1, T], [0, 1]])
    F_m = np.linalg.matrix_power(F, m)
    extr = np.zeros(data.shape)
    for i in range(data.shape[1]-m):
        extr[:,i+m-1] = np.dot(F_m, data[:,i])
    return extr

# Function for calculation errors on M runs of Kalman filter
def mean_errors (runs, size, time_step, acc_sigma2, measure_sigma2, X_0, P_00, m):
    filt_err = np.zeros([runs, size])
    final_filt_err = np.zeros(size)
    extr_err = np.zeros([runs, size])
    final_extr_err = np.zeros(size)
    for i in range(runs):
        # generate true trajectory
        X = get_true_trajectory_with_random_acceleration(init_position=5, init_velocity=1, time_step=time_step,
                                                         points_num=size, acceleration_sigma2=acc_sigma2)
        # generate measurements
        Z, _ = get_measurements(trajectory=X, sigma2=measure_sigma2)
        # perform Kalman filter
        x_filt, K, sigma_k = kalman_filter(size, X_0, time_step, Z, P_00, acc_sigma2, measure_sigma2)
        # perform extrapolation
        x_extr = extrapolation(m, time_step, x_filt)
        # mean-squared error of estimation
        filt_err[i,:] = (X - x_filt[0,:])**2
        extr_err[i,:] = (X - x_extr[0,:])**2
    # mean errors starting from 3 step
    for i in range(2, size):
        final_filt_err[i] = np.sqrt(np.mean(filt_err[:,i]))
        final_extr_err[i] = np.sqrt(np.mean(extr_err[:,i]))
    return final_filt_err, final_extr_err