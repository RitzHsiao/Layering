import shutil as shutil
import numpy as np
import os as os
from sklearn.cluster import DBSCAN
import open3d as o3d

from File import SaveFile
# import trajectory as Detraj

def points_in_cylinder(pt1, pt2, r, q):
    # If the point is inside, then it will be True
    Point_is_inside = False

    vec = np.subtract(pt2, pt1)
    const = r*np.linalg.norm(vec)

    top = np.subtract(q, pt1)       # Raw Point is at the top of LastPoint
    down = np.subtract(q, pt2)      # Raw Point is at the bottom of NextPoint

    if np.linalg.norm(np.cross(top, vec)) <= const :
        if np.dot(top, vec) >= 0 and np.dot(down, vec)<=0:
            Point_is_inside = True

    return Point_is_inside

def Cluster(Point, SampleDistance = 2, min_samples=2):
    data = np.asarray(Point)
    # Start Clustering (but not sorted yet in this part)
    model = DBSCAN(eps=SampleDistance, min_samples=min_samples)
    model.fit_predict(data)

    # Prepare the list, prepare the list inside clusterlist.
    ClusterList = [[] for _ in range(len(set(model.labels_)))]

    # In clustering, we maybe will find points which are noise, so if found noise, noise status will become True.
    # The noise points will be grouped in one cluster (-1) and will be removed.
    noise = False
    print('Total Found ClusterList :', len(ClusterList))

    # Start sorting the data based on index number of clustering [after clustering step]
    for data_no in range(0, len(data)):
        # Check the point belongs to which cluster (cluster 1, cluster 2, cluster 3?)
        clusterIdx = model.labels_[data_no]

        # index = -1 means it is noise point
        if clusterIdx != -1:
            ClusterList[clusterIdx].append(Point[data_no])

    return ClusterList

def Layering(RefPointCloud, PointCloudIdx, Normal, tree, RawPointCloud, LayerDepth):


    # Show the Layer Height to the user.
    print('Layer Height:', LayerDepth, 'mm')

    # Parameter for layering.
    KnnRadius = 1.5  # radius for tube (Propeller = 1, golfclubhead = 0.6)  2.2
    LayerFolder = 'Layer'
    print('The result of layering is saved at:', LayerFolder)

    # Deleting and Creating again New "Layer" Folder.
    if os.path.exists(LayerFolder):
        print('Note: If it is error, Make sure to close the "Layer" folder first and run it again.')
        shutil.rmtree(LayerFolder)
    os.makedirs(LayerFolder)

    # LAYERING PART.
    NoOfPointCloud = len(RefPointCloud)
    layer = 0

    # We want all reference points are labeled. If NoOfPointCloud == 0 , layering will be stopped.
    while NoOfPointCloud != 0:
        print('\nRemaining Points:', NoOfPointCloud, '/', len(RefPointCloud))
        print('Layer:', layer)

        # NewPoint is used for creating new point in each layer,
        # this point is from ref points and the direction of going up is based on normals direction.
        BasePoint = []
        BasePointNormal = []
        NewPoint = []
        NewPointNormal = []

        # Check if surrounding of NewPoint there're raw points or not
        for PCDNo in range(0, len(RefPointCloud)):  # Remember: These Point Clouds are from triangle mesh.

            if PointCloudIdx[PCDNo] == False:  # If PointCloud hasn't been labeled.

                LastTempPoint = []
                NextTempPoint = []

                BaseCloud = RefPointCloud[PCDNo]
                BaseNormal = Normal[PCDNo]

                # This is for setting the position of starting layer in each level.
                # LastTempConst = LayerDepth * (layer - 1)

                # If we want the depth of layer 1 is different with others
                if layer == 1:
                    LastTempConst = 0
                    NextTempConst = LayerDepth

                elif layer == 2:
                    LastTempConst = LayerDepth
                    NextTempConst = LayerDepth + (LayerDepth * (layer - 1))

                elif layer > 2:
                    LastTempConst = LayerDepth + (LayerDepth * (layer - 2))  # Jangan lupa atur disini
                    NextTempConst = LayerDepth + (LayerDepth * (layer - 1))



                # Creating the top and bottom range of tube.
                # Note: top(NextTempPoint) = [x,y,z] bottom(LastTempPoint) = [x,y,z]
                if layer == 0:  # Layer 0
                    for coord in range(0, 3):  # Create Last and New Point with XYZ
                        LastTempPoint.append(BaseCloud[coord] + (-2) * BaseNormal[coord]) # Propeller= -2
                        NextTempPoint.append(BaseCloud[coord] + 0.01 * BaseNormal[coord]) # Propeller = 0.2 ScannedPropeller=0.01
                        # print(BaseCloud[])


                else:  # if Layer is not layer 0
                    for coord in range(0, 3):  # Create Last and New Point with XYZ
                        LastTempPoint.append(BaseCloud[coord] + LastTempConst * BaseNormal[coord])
                        NextTempPoint.append(BaseCloud[coord] + NextTempConst * BaseNormal[coord])

                # Now we have new created point, now we need to np.asarray so it can be used for KNN searching.
                PointInput = np.asarray([NextTempPoint])
                # This tree is for checking nearby point.
                dist, ind = tree.query(PointInput, k=4)  # it will find the 4 points which are near PointInput
                # print('ind = ', ind)
                # print('RawPointCloud = ', RawPointCloud)

                GrowingStatus = True
                for index in range(0, len(ind[0])):
                    # Check surrounding of this new point , there're raw points or not.
                    PointChecked = points_in_cylinder(LastTempPoint, NextTempPoint, KnnRadius, RawPointCloud[ind[0][index]])

                    # If there's point is inside the tube, then this NewPoint stopped growing up.
                    if PointChecked == True:

                        # This NewPoint is labeled True (means: it stops growing up)
                        PointCloudIdx[PCDNo] = True

                        # Because already passed the raw point, so we don't need to save this point in the next NewPoint
                        GrowingStatus = False

                        # This is just for telling user how many NewPoint are still remained.
                        NoOfPointCloud = NoOfPointCloud - 1

                        # Stop finding the raw point which is inside the tube.
                        break

                # If after checking, we still need to grow up this point, then we will save that new point in next layer
                if GrowingStatus == True and layer > 0:
                    NewPoint.append(NextTempPoint)
                    NewPointNormal.append(BaseNormal)


                # This Layer 0 just show us the base point
                if layer == 0:
                    NewPoint.append(BaseCloud)
                    NewPointNormal.append(BaseNormal)


                if layer == 0 and GrowingStatus == True:
                    BasePoint.append(BaseCloud)
                    BasePointNormal.append(BaseNormal) #aku ubah sini


        # Save the new points per layer
        if layer == 0:
            SaveFile('checkresult/Layer %d' % (layer), BasePoint)
            # SaveFile('Layer/Layer %d normal' % (layer), BasePointNormal)
            layer = layer + 1
        else:
            if len(NewPoint) != 0:
                SaveFile('checkresult/Layer %d' % (layer), NewPoint)
                # SaveFile('Layer/Layer %d normal' % (layer), NewPointNormal)

                layer = layer + 1

            if len(NewPoint) == 0:
                print('Layer', layer, 'is canceled. (There are no points higher than this layer)')
                layer = layer - 1
                break

    print('total layer = ', layer)
    rest = layer * LayerDepth
    print('rest depth = ', rest, 'mm')

    return







