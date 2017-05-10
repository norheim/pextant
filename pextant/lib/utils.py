import numpy as np
def gridpoints_list(array):
    X, Y = np.mgrid[0:array.shape[0], 0:array.shape[1]]
    positions = np.column_stack((X.flatten(), Y.flatten()))
    return positions