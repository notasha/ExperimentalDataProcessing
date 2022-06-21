import numpy as np

# Function for calculating deviation indicator
def deviation(measurements, estimations):
    return np.sum((measurements - estimations)**2)

# Function for calculating variability indicator
def variability(array):
    I_v = np.zeros(array.shape)
    for i in range(array.shape[0]-2):
        I_v[i] = (array[i+2] - 2 * array[i+1] + array[i])**2
    return np.sum(I_v)