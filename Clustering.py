import numpy as np
import os as os
import random
import shutil as shutil
import math

from sklearn.cluster import DBSCAN

from File import SaveFile
from File import ReadLayerFile
# from Visualize import Bar

def LayerCluster(SampleDistance = 5, ActiveTheColoredCluster = False):
    # This def is used in Main.py

    # This part is for telling the program where is the Layer File
    LayerFolderName = 'Layer/'
    path, dirs, files = next(os.walk("Layer"))
    file_count = int(len(files)/2)   # Divided by 2 because the Folder contains Layer and Normal files


    # Empty and create the Colored Cluster Folder
    Folder = 'Output File/Colored Cluster'
    if os.path.exists(Folder):
        shutil.rmtree(Folder)
        os.makedirs(Folder)
    else:
        os.makedirs(Folder)


    # Showing the information of clustering
    print('Total Layer Files\t\t:', file_count, '[Layer 0 to Layer {}]'.format(file_count-1))
    print('Distance in 1 cluster\t:', SampleDistance, 'mm')

    # User decide if the cluster points need to use color or not (just for showing the cluster is good by using software)
    if ActiveTheColoredCluster == True:  # If you want Colored Cluster File
        print('Colored Cluster File\t: ON (Color clustering file will be generated)\n')
    elif ActiveTheColoredCluster == False:
        print('Colored Cluster File\t: OFF\n')


    # Set the number of array layer for storing points.
    # In propeller case, we want to separate top and bottom wing.
    TopWingPointList = [[] for _ in range(file_count)]
    TopWingPointNormalList = [[] for _ in range(file_count)]
    BottomWingPointList = [[] for _ in range(file_count)]
    BottomWingPointNormalList = [[] for _ in range(file_count)]



    # Start Clustering
    for LayerNo in range(0, file_count):  # Layer starts from layer 0

        # Read Layer File and Normal Layer File
        Data = ReadLayerFile(LayerFolderName + 'Layer {}.xyz'.format(LayerNo))
        DataNormal = ReadLayerFile(LayerFolderName + 'Layer {} normal.xyz'.format(LayerNo))

        # Now we have point clouds and their each normals.

        # Do clustering
        # ClusterList, ColoredClusterList, ClusterNormalList = cluster(Data, DataNormal, SampleDistance, ActiveTheColoredCluster)
        TopWingPointList[LayerNo], TopWingPointNormalList[LayerNo], BottomWingPointList[LayerNo], BottomWingPointNormalList[LayerNo], ColoredClusterList = cluster(Data, DataNormal, SampleDistance, ActiveTheColoredCluster)
        # AllClusterList[LayerNo] = ClusterList
        # AllClusterNormalList[LayerNo] = ClusterNormalList

        # This is for clustering points using color and save it
        if ActiveTheColoredCluster == True:
            SaveFile('Output File/Colored Cluster/ClusteringLayer {}'.format(LayerNo), ColoredClusterList)

        TopWingSave = []
        for ClusterNo in range(0, len(TopWingPointList[LayerNo])):
            for PointNo in range(0, len(TopWingPointList[LayerNo][ClusterNo])):
                TopWingSave.append(TopWingPointList[LayerNo][ClusterNo][PointNo])

        BotWingSave = []
        for ClusterNo in range(0, len(BottomWingPointList[LayerNo])):
            for PointNo in range(0, len(BottomWingPointList[LayerNo][ClusterNo])):
                BotWingSave.append(BottomWingPointList[LayerNo][ClusterNo][PointNo])


        # Save the top point of wing per layer
        if not os.path.exists('Output File/Part Layer'.format()):
            print('Note: If it is error, Make sure to close the "Layer" folder first and run it again.')
            os.makedirs('Output File/Part Layer')
        SaveFile('Output File/Part Layer/Top_Layer_{}'.format(LayerNo), TopWingSave)
        SaveFile('Output File/Part Layer/Bot_Layer_{}'.format(LayerNo), BotWingSave)
        print('\n')


    # TopWIngPointList = [ Layer [ Cluster [ Point ] ] ]
    return TopWingPointList, TopWingPointNormalList, BottomWingPointList, BottomWingPointNormalList


def clusterCheck(u, v):
    # Set the degree difference between two normals
    degree_threshold = 30

    # Magnitude of normals
    u1 = math.sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2])
    v1 = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])

    # Do dot product for 2 vectors
    uv = np.dot(u, v)

    # cos theta = U.V / ||U|| ||V||
    degree = math.degrees(math.acos(round((uv / (u1 * v1)), 2)))
    # print('degree:', degree)

    # if degree less than threshold, then we assume the normal direction is same.
    if degree <= degree_threshold:
        return True
    else:
        return False

def clusterCheckUpDown(NormalVector):
    # True: up  False: Down
    if NormalVector[2] >= 0:
        return True




def cluster(PointCloud, PointCloudNormal, SampleDistance = 2, ColoredCluster = False):
    data = np.asarray(PointCloud)

    # Start Clustering (but not sorted yet in this part)
    model = DBSCAN(eps=SampleDistance, min_samples=2)
    model.fit_predict(data)
    # print(model.labels_)
    # print('Total Label',len(set(model.labels_)))

    # Prepare the list, prepare the list inside clusterlist.
    ClusterList = [[] for _ in range(len(set(model.labels_)))]
    ClusterNormalList = [[] for _ in range(len(set(model.labels_)))]
    ColoredClusterList = []

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
            ClusterList[clusterIdx].append(PointCloud[data_no])
            ClusterNormalList[clusterIdx].append(PointCloudNormal[data_no])

        # Tell the program that there are noise points
        elif clusterIdx == -1:
            noise = True

    # Remove List which contains noise points
    if noise:
        ClusterList.pop(len(ClusterList)-1)
        print('(There is cluster noise)')

    # Suppose we do clustering on the corner of cubic, then the points on 3 sides will be clustered to be 1 cluster.
    # Because Clustering is doing ball KNN. So the point clouds on top wing and bottom wing on the edge of wing will be clustered as 1 cluster.
    # So I do Clustering again based on normals
    NewClusterList = []
    NewClusterNormalList = []
    NewClusterNo = -1

    # ClusterNormalList = [[cluster0], [cluster1],..., [clusterN]]
    for cluster_no in range(0, len(ClusterNormalList)):
        #
        LastNumbCluster = len(NewClusterList)
        ReferenceCluster = []

        # Each cluster has many points [x,y,z]
        # e.g: cluster0 = [[point1], [point2],[point3], ... , [pointN]]
        for point_no in range(0, len(ClusterNormalList[cluster_no])):

            # If it is the first point, then we don't have comparison normal, so we set this normal as 1st ref point
            if point_no == 0:

                # Fill ref normal into ref cluster (Remember: we want to compare normal of each points, not point coord)
                ReferenceCluster.append(ClusterNormalList[cluster_no][point_no])

                NewClusterNo = NewClusterNo + 1

                NewClusterList.append([])
                NewClusterNormalList.append([])

                # input the point into new cluster list based on ref index
                NewClusterList[NewClusterNo].append(ClusterList[cluster_no][point_no])
                NewClusterNormalList[NewClusterNo].append(ClusterNormalList[cluster_no][point_no])

            # If it isn't first point, then we already had comparison normal
            else:
                # Step 1. We check the direction of normal is same with any ref normals or not
                same = False

                for RefClusterNo in range(0, len(ReferenceCluster)):

                    # Checking if it's same or not
                    # [ Note: Check this function on def!]
                    # If it is same with any ref normals, then we labeled it in 1 of ref cluster
                    if clusterCheck(ClusterNormalList[cluster_no][point_no], ReferenceCluster[RefClusterNo]):

                        NewClusterList[RefClusterNo + LastNumbCluster].append(ClusterList[cluster_no][point_no])
                        NewClusterNormalList[RefClusterNo + LastNumbCluster].append(ClusterNormalList[cluster_no][point_no])
                        same = True
                        break

                # Step 2. If not same with any references, then create new reference normal.
                if same == False:

                    # This part is same with the part on point_no == 0
                    ReferenceCluster.append(ClusterNormalList[cluster_no][point_no])

                    NewClusterNo = NewClusterNo + 1

                    NewClusterList.append([])
                    NewClusterNormalList.append([])

                    NewClusterList[NewClusterNo].append(ClusterList[cluster_no][point_no])
                    NewClusterNormalList[NewClusterNo].append(ClusterNormalList[cluster_no][point_no])


    # Cluster = [  [cluster [point]  ]  ]
    print('ClusterList:', len(ClusterList))
    print('NewClusterList:', len(NewClusterList))

    print('ClusterNormalList:', len(ClusterNormalList))
    print('NewClusterNormalList:', len(NewClusterNormalList))





    # Now let's combine all points which is facing up in 1 cluster, also for facing down in 1 cluster.
    TopWingIdxNo = -1
    TopWingPointList = []
    TopWingPointNormalList = []

    BotWingIdxNo = -1
    BottomWingPointList = []
    BottomWingPointNormalList = []

    TopWingSave = []

    last_cluster_no = -1
    for cluster_no in range(0, len(NewClusterNormalList)):
        # TopWingPointList.append([])
        # TopWingPointNormalList.append([])
        # BottomWingPointList.append([])
        # BottomWingPointNormalList.append([])


        for point_no in range(0, len(NewClusterNormalList[cluster_no])):

            # If normals face up
            if clusterCheckUpDown(NewClusterNormalList[cluster_no][point_no]):
                # If the cluster no is different
                if cluster_no != last_cluster_no:
                    TopWingIdxNo = TopWingIdxNo + 1
                    TopWingPointList.append([])
                    TopWingPointNormalList.append([])

                TopWingPointList[TopWingIdxNo].append(NewClusterList[cluster_no][point_no])
                TopWingPointNormalList[TopWingIdxNo].append(NewClusterNormalList[cluster_no][point_no])
                TopWingSave.append(NewClusterList[cluster_no][point_no])
            # If normals face down
            else:
                # If the cluster no is different
                if cluster_no != last_cluster_no:
                    BotWingIdxNo = BotWingIdxNo + 1
                    BottomWingPointList.append([])
                    BottomWingPointNormalList.append([])

                BottomWingPointList[BotWingIdxNo].append(NewClusterList[cluster_no][point_no])
                BottomWingPointNormalList[BotWingIdxNo].append(NewClusterNormalList[cluster_no][point_no])

            last_cluster_no = cluster_no




    # SaveFile('Top', TopWingSave)


    # If you want Colored Cluster File, set the ColoredCluster = yes
    if ColoredCluster:

        # Prepare the number of random color
        number_of_colors = len(NewClusterList)
        color = [([random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)])
                 for i in range(number_of_colors)]
        # print('color:\n', color)

        # Do coloring based on cluster
        for cluster_idx in range(0, len(NewClusterList)):
            for point_no in range(0, len(NewClusterList[cluster_idx])):
                ColoredClusterList.append(NewClusterList[cluster_idx][point_no] + color[cluster_idx])

    # print('Color List\n', ColoredClusterList)






    # print('Cluster Normal List:\n', ClusterNormalList)
    # return NewClusterList, ColoredClusterList, NewClusterNormalList

    # TopWingPointList = [ [cluster [point] ] ]
    return TopWingPointList, TopWingPointNormalList, BottomWingPointList, BottomWingPointNormalList, ColoredClusterList






