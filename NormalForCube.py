import glob
import shutil as shutil
import numpy as np
import os as os
import math as math
import open3d as o3d
import time
import estNormal

from sklearn.cluster import DBSCAN

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
    # print('Total Found ClusterList :', len(ClusterList))

    # Start sorting the data based on index number of clustering [after clustering step]
    for data_no in range(0, len(data)):
        # Check the point belongs to which cluster (cluster 1, cluster 2, cluster 3?)
        clusterIdx = model.labels_[data_no]

        # index = -1 means it is noise point
        if clusterIdx != -1:
            ClusterList[clusterIdx].append(Point[data_no])
    return ClusterList

def ReadXyzFile(filename):
    # print('File Path:', filename)
    f = open(filename, "r")
    lines = f.readlines()
    # print('No of Points [XYZ]:', len(lines))
    PointList = []

    for x in range(0, len(lines)):
        RawData = lines[x].strip().split() #[x y z] from File
        PointList.append([float(RawData[0]), float(RawData[1]), float(RawData[2])])

    return PointList

def SaveFile(Pcd_File_Name, PCDList):
    np.savetxt('{}'.format(Pcd_File_Name), PCDList, delimiter=' ')
    print('Saved File: [{}].'.format(Pcd_File_Name))

def Sq2(value):
    # Code untuk Kuadrat bilangan
    return value*value

def Sq2(value):
    return value * value

NameOfOref = 'cluster/orefCluster0.xyz'
NameOfref = 'cluster/refCluster0.xyz'
NameOfNor = 'C:\\Users\\User\\Desktop\\陳昱廷\\Extruded Parts Removal\\Result\\NormalForRef.xyz'
# -------------------------------------------------------------------------------------------------
Oref = ReadXyzFile(NameOfOref)
Nor = ReadXyzFile(NameOfNor)

for Oref_no, Oref_value in enumerate(Oref):
    Oref_value.append(Nor[0][0])
    Oref_value.append(Nor[0][1])
    Oref_value.append(Nor[0][2])
# print(Oref)q
SaveFile(NameOfref, Oref)
