import numpy as np
import scipy.linalg
import math

from File import SaveFile

def Fit(data, GridPoint_dist = 0.1): #1.5
    # THIS IS NOT PROPER FOR LAYERING, STILL NEEDS MODIFICATION, SO SURFACE FITTING IS NOT USED NOW.

    # print('Distance of each Grid Point =', GridPoint_dist)
    data = np.asarray(data)

    # regular grid covering the domain of the data
    mn = np.min(data, axis=0)   #Set minimum in X, Y, Z
    mx = np.max(data, axis=0)

    # Get number of X grid points and Y grid points
    No_of_GridpointsX = math.ceil((mx[0] - mn[0]) / GridPoint_dist) #20
    No_of_GridpointsY = math.ceil((mx[1] - mn[1]) / GridPoint_dist)
    X, Y = np.meshgrid(np.linspace(mn[0], mx[0], No_of_GridpointsX), np.linspace(mn[1], mx[1], No_of_GridpointsY))

    # Make X and Y array become 1 array
    XX = X.flatten()
    YY = Y.flatten()

    # Best-fit linear plane (2nd-order)
    # A = np.c_[np.ones(data.shape[0]), data[:,:2], np.prod(data[:,:2], axis=1), data[:,:2]**2]
    # C,_,_,_ = scipy.linalg.lstsq(A, data[:,2])  # coefficients

    A = np.c_[np.ones(data.shape[0]), data[:,:2], np.prod(data[:,:2], axis=1), data[:,:2]**2, data[:,:2]**3]
    C,_,_,_ = scipy.linalg.lstsq(A, data[:,2])

    # Get the z value of each grid points
    # Z = np.dot(np.c_[np.ones(XX.shape), XX, YY, XX*YY, XX**2, YY**2], C).reshape(X.shape)
    Z = np.dot(np.c_[np.ones(XX.shape), XX, YY, XX*YY, XX**2, YY**2, XX**3, YY**3], C).reshape(X.shape)
    ZZ = Z.flatten()


    Grid = []
    for pointNo in range(0, len(XX)):
        Grid.append([XX[pointNo], YY[pointNo], ZZ[pointNo]])

    # print('\nGrid:\n', Grid)

    # SaveFile('Grid', Grid)
    # SaveFile('Data', data)

    return Grid

