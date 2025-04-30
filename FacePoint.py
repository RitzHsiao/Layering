import math as math
import numpy as np

# This code for generating the point clouds in the triangle of triangular mesh.
# We can say this is for subsampling the mesh to point clouds.

def NewPoint(RefVertice, RefNormal):
    NewPoint = []
    NewPointIdx = []
    NewNormal = []

    for faceNo in range(0, len(RefVertice)):
        NormalSum = math.sqrt(np.power(RefNormal[faceNo][0], 2) + np.power(RefNormal[faceNo][1], 2) + np.power(RefNormal[faceNo][2], 2))
        RefNormal[faceNo] = [RefNormal[faceNo][0]/NormalSum, RefNormal[faceNo][1]/NormalSum, RefNormal[faceNo][2]/NormalSum]
        midTriX = 0
        midTriY = 0
        midTriZ = 0
        for vertexNo in range(0, 3):
            midTriX = midTriX + RefVertice[faceNo][vertexNo][0]
            midTriY = midTriY + RefVertice[faceNo][vertexNo][1]
            midTriZ = midTriZ + RefVertice[faceNo][vertexNo][2]
        midTriX = midTriX / 3
        midTriY = midTriY / 3
        midTriZ = midTriZ / 3
        NewPoint.append([midTriX, midTriY, midTriZ])
        NewPointIdx.append(False)
        NewNormal.append(RefNormal[faceNo])

        for vertexNo in range(0, 3):
            newX = (midTriX + RefVertice[faceNo][vertexNo][0]) / 2
            newY = (midTriY + RefVertice[faceNo][vertexNo][1]) / 2
            newZ = (midTriZ + RefVertice[faceNo][vertexNo][2]) / 2

            NewPoint.append([newX, newY, newZ]) #3
            NewPointIdx.append('False')
            NewNormal.append(RefNormal[faceNo])

    # SaveFile('Output File/New Point', NewPoint)
    print('Total new point:', len(NewPoint))
    print('Total new point idx:', len(NewPointIdx))

    return NewPoint, NewPointIdx, NewNormal