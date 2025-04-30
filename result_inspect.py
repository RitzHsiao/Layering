'''
生成第一次個別掃描的 gorden model
'''
import os as os
import open3d as o3d
import numpy as np
import math
import glob
import sys
import File
import shutil as shutil
import FacePoint as Fp
import calc_height as Calc
import NxyzSurfacfit as Nsurf
import MinMax as Mm
import re
import time as time
from sklearn.neighbors import KDTree
from sklearn.cluster import DBSCAN

def remove_specific_files(folder_path):
    files = os.listdir(folder_path)
    for file in files:
        if any(file.startswith(prefix) and file.endswith(".xyz") for prefix in [
            "source_decrease_", "Cluster", "EdgePoint", "OBBpoint_SF", "OBBpoint",
            "RemovedPoints", "UnremovedPoints", "SF_SampledPoints_Cluster_", "rawCluster",
            "refCluster", "tra_0", "tra_inv_0", "grid_0", "grid_after_0", "SF_Grid_0",
            "SF_Grid_List", "XYZC_axiel_0", "After_SF"
        ]):
            os.remove(os.path.join(folder_path, file))
    shutil.rmtree('c:\\Users\\User\\Desktop\\陳昱廷\\Layering\\inc_trajectory')
    os.makedirs('c:\\Users\\User\\Desktop\\陳昱廷\\Layering\\inc_trajectory')

def Sq2(value):
    # Code untuk Kuadrat bilangan
    return value*value

def SampleThePoint(MainPoint, tree, K_number = 5, KNNRadius = 1.5):
    distances, indices = tree.query(MainPoint, K_number)
    SampledPoints = []
    for i in range(0, K_number):
        if distances[0][i] <= KNNRadius:
            SampledPoints.append(SourcePointFile_decrease[int(indices[0][i])])
    return SampledPoints

def ReadXyzFile(filename):
    print('File Path:', filename)
    f = open(filename, "r")
    lines = f.readlines()
    print('No of Points [XYZ]:', len(lines))
    PointList = []

    for x in range(0, len(lines)):
        RawData = lines[x].strip().split() #[x y z] from File
        PointList.append([float(RawData[0]), float(RawData[1]), float(RawData[2])])

    return PointList

def CovXYZ(Value1, Value2, Value1Mean, Value2Mean):
    # Formula: Cov(x,x) = [Summation (xi-xmean)(yi-ymean)] / (n-1)

    # Summation Part
    Sum = 0
    for point_no in range(0, len(Value1)):
        Sum = Sum + ((Value1[point_no] - Value1Mean)*(Value2[point_no] - Value2Mean))
    # print('SumPart=', Sum)

    # Compute the Covariance
    Cov = Sum / (len(Value1) - 1)

    return Cov

def SeparateXYZ(Point):
    X = []
    Y = []
    Z = []

    XSum = 0
    YSum = 0
    ZSum = 0

    # print('Len', len(Point))
    for PointNo in range(0, len(Point)):

        X.append(Point[PointNo][0])
        Y.append(Point[PointNo][1])
        Z.append(Point[PointNo][2])
        XSum = XSum + Point[PointNo][0]
        YSum = YSum + Point[PointNo][1]
        ZSum = ZSum + Point[PointNo][2]


    XMean = XSum / len(Point)
    YMean = YSum / len(Point)
    ZMean = ZSum / len(Point)

    return X, Y, Z, XMean, YMean, ZMean

def OrientedBoundingBox(Point, offset, SF_BB_offset):
    # // FOR FINDING 8 POINTS OF BOUNDING BOX FOR REMOVED POINTS AND BOUNDING BOX FOR SURFACE FITTING

    # // We list the value of all sample points as X, Y, Z list
    x, y, z, XMean, YMean, ZMean = SeparateXYZ(Point)
    # print('XYZMean:', XMean, YMean, ZMean)

    # // Calculate Covariance Matrix
    C = [[CovXYZ(x, x, XMean, XMean), CovXYZ(x, y, XMean, YMean), CovXYZ(x, z, XMean, ZMean)],
         [CovXYZ(y, x, YMean, XMean), CovXYZ(y, y, YMean, YMean), CovXYZ(y, z, YMean, ZMean)],
         [CovXYZ(z, x, ZMean, XMean), CovXYZ(z, y, ZMean, YMean), CovXYZ(z, z, ZMean, ZMean)]]
    # print('\nCovariance Matrix for OBB\n', C)

    # // Calculate the eigenvector
    eigenvalue, eigenvector = np.linalg.eig(C)
    # print('EigenVector:', eigenvector)

    cluster = []
    # // Get the X Y Z min and max for bounding box
    for PointNo in range(0, len(Point)):
        # Project the point using PCA, so the coordinate x0,y0,z0 will change
        a = np.dot(Point[PointNo], eigenvector)
        cluster.append(np.dot(Point[PointNo], eigenvector))

        # Initial the XYZ min and max
        if PointNo == 0:
            XminPCA = a[0]
            XmaxPCA = a[0]
            YminPCA = a[1]
            YmaxPCA = a[1]
            ZminPCA = a[2]
            ZmaxPCA = a[2]


        # Find the XY min and max
        else:
            if a[0] < XminPCA:
                XminPCA = a[0]
            if a[0] > XmaxPCA:
                XmaxPCA = a[0]
            if a[1] < YminPCA:
                YminPCA = a[1]
            if a[1] > YmaxPCA:
                YmaxPCA = a[1]
            if a[2] < ZminPCA:
                ZminPCA = a[2]
            if a[2] > ZmaxPCA:
                ZmaxPCA = a[2]

    # SaveFile('Result/first_cluster{}.xyz'.format(EdgeCluster_no), cluster)

    # // Get the middle and bounding box points (Normal and Surface Fitting) at PCA coordinate

    ## Get the middle
    midPCA = [((XmaxPCA - XminPCA)/2) + XminPCA, ((YmaxPCA - YminPCA)/2) + YminPCA, ((ZmaxPCA - ZminPCA)/2) + ZminPCA]
    # print('mid_PCA:', midPCA)


    ## Get the 8 points of bounding box
    # 1.Bottom box, counter-clockwise rotate
    XminYminZminPCA = [XminPCA - offset, YminPCA - offset, ZminPCA - offset]
    XmaxYminZminPCA = [XmaxPCA + offset, YminPCA - offset, ZminPCA - offset]
    XmaxYmaxZminPCA = [XmaxPCA + offset, YmaxPCA + offset, ZminPCA - offset]
    XminYmaxZminPCA = [XminPCA - offset, YmaxPCA + offset, ZminPCA - offset]
    # 2.Top box, counter-clockwise rotate
    XminYminZmaxPCA = [XminPCA - offset, YminPCA - offset, (ZmaxPCA + offset) ]
    XmaxYminZmaxPCA = [XmaxPCA + offset, YminPCA - offset, (ZmaxPCA + offset) ]
    XmaxYmaxZmaxPCA = [XmaxPCA + offset, YmaxPCA + offset, (ZmaxPCA + offset) ]
    XminYmaxZmaxPCA = [XminPCA - offset, YmaxPCA + offset, (ZmaxPCA + offset) ]


    ## Get the 8 points of bounding box (FOR SURFACE FITTING)
    # X dan Y diperlebar ( makanya pakai SF_BB_offset)
    # 1.Bottom box, counter-clockwise rotate
    XminYminZmin_SF_PCA = [XminPCA - SF_BB_offset, YminPCA - SF_BB_offset, ZminPCA - SF_BB_offset]
    XmaxYminZmin_SF_PCA = [XmaxPCA + SF_BB_offset, YminPCA - SF_BB_offset, ZminPCA - SF_BB_offset]
    XmaxYmaxZmin_SF_PCA = [XmaxPCA + SF_BB_offset, YmaxPCA + SF_BB_offset, ZminPCA - SF_BB_offset]
    XminYmaxZmin_SF_PCA = [XminPCA - SF_BB_offset, YmaxPCA + SF_BB_offset, ZminPCA - SF_BB_offset]
    # 2.Top box, counter-clockwise rotate
    XminYminZmax_SF_PCA = [XminPCA - SF_BB_offset, YminPCA - SF_BB_offset, ZmaxPCA + SF_BB_offset]
    XmaxYminZmax_SF_PCA = [XmaxPCA + SF_BB_offset, YminPCA - SF_BB_offset, ZmaxPCA + SF_BB_offset]
    XmaxYmaxZmax_SF_PCA = [XmaxPCA + SF_BB_offset, YmaxPCA + SF_BB_offset, ZmaxPCA + SF_BB_offset]
    XminYmaxZmax_SF_PCA = [XminPCA - SF_BB_offset, YmaxPCA + SF_BB_offset, ZmaxPCA + SF_BB_offset]

    ## Get the HalfExtent_lengths of x y z (For normal bounding box)
    HalfExtent_lengths = math.sqrt(Sq2(XmaxYmaxZmaxPCA[0] - midPCA[0]) + Sq2(XmaxYmaxZmaxPCA[1] - midPCA[1]) + Sq2(XmaxYmaxZmaxPCA[2] - midPCA[2]))

    # // Return the middle and all of the bounding box points to the original coordinate using inverse matrix
    eigenvector_inv = np.linalg.inv(eigenvector)
    mid = np.dot(midPCA, eigenvector_inv)

    ## Normal Bounding Box
    # 1.Bottom box, counter-clockwise rotate
    XminYminZmin = np.dot(XminYminZminPCA, eigenvector_inv)
    XmaxYminZmin = np.dot(XmaxYminZminPCA, eigenvector_inv)
    XmaxYmaxZmin = np.dot(XmaxYmaxZminPCA, eigenvector_inv)
    XminYmaxZmin = np.dot(XminYmaxZminPCA, eigenvector_inv)
    # 2.Top box, counter-clockwise rotate
    XminYminZmax = np.dot(XminYminZmaxPCA, eigenvector_inv)
    XmaxYminZmax = np.dot(XmaxYminZmaxPCA, eigenvector_inv)
    XmaxYmaxZmax = np.dot(XmaxYmaxZmaxPCA, eigenvector_inv)
    XminYmaxZmax = np.dot(XminYmaxZmaxPCA, eigenvector_inv)

    ## Bounding Box (FOR SURFACE FITTING)
    # 1.Bottom box, counter-clockwise rotate
    XminYminZmin_SF = np.dot(XminYminZmin_SF_PCA, eigenvector_inv)
    XmaxYminZmin_SF = np.dot(XmaxYminZmin_SF_PCA, eigenvector_inv)
    XmaxYmaxZmin_SF = np.dot(XmaxYmaxZmin_SF_PCA, eigenvector_inv)
    XminYmaxZmin_SF = np.dot(XminYmaxZmin_SF_PCA, eigenvector_inv)
    # 2.Top box, counter-clockwise rotate
    XminYminZmax_SF = np.dot(XminYminZmax_SF_PCA, eigenvector_inv)
    XmaxYminZmax_SF = np.dot(XmaxYminZmax_SF_PCA, eigenvector_inv)
    XmaxYmaxZmax_SF = np.dot(XmaxYmaxZmax_SF_PCA, eigenvector_inv)
    XminYmaxZmax_SF = np.dot(XminYmaxZmax_SF_PCA, eigenvector_inv)


    # // Store the BoundingBox Points.
    BBPoint = [XminYminZmin, XmaxYminZmin, XmaxYmaxZmin, XminYmaxZmin, XminYminZmax, XmaxYminZmax, XmaxYmaxZmax,
               XminYmaxZmax]
    BBPoint_SF = [XminYminZmin_SF, XmaxYminZmin_SF, XmaxYmaxZmin_SF, XminYmaxZmin_SF, XminYminZmax_SF, XmaxYminZmax_SF, XmaxYmaxZmax_SF,
               XminYmaxZmax_SF]

    return mid, BBPoint, HalfExtent_lengths, BBPoint_SF

def SurfVar(Point):
    # Method: Surface Variation is the value to determine if the point is edge or not
    # We list the value of all sample points as X, Y, Z list
    x, y, z, XMean, YMean, ZMean = SeparateXYZ(Point)
    # print('XYZMean:', XMean, YMean, ZMean)

    # Calculate Covariance Matrix
    C = [[CovXYZ(x, x, XMean, XMean), CovXYZ(x, y, XMean, YMean), CovXYZ(x, z, XMean, ZMean)],
         [CovXYZ(y, x, YMean, XMean), CovXYZ(y, y, YMean, YMean), CovXYZ(y, z, YMean, ZMean)],
         [CovXYZ(z, x, ZMean, XMean), CovXYZ(z, y, ZMean, YMean), CovXYZ(z, z, ZMean, ZMean)]]
    # print('Covariance Matrix:\n', C)

    # Calculate the eigenvalue(3 Lambda)
    eigenvalue, eigenvector = np.linalg.eig(C)
    # print('EigenValue:', eigenvalue)

    # Sort the lambda as Lambda0<Lambda1<Lambda2
    Lambda = []
    for eigenvalue_no in range(0, len(eigenvalue)):
        if eigenvalue_no == 0:
            Lambda.append(eigenvalue[0])
        else:
            InsertedCheck = False  # To check if the value is smaller than the value in the list or not
            for Lambda_no in range(0, len(Lambda)):
                if eigenvalue[eigenvalue_no] < Lambda[Lambda_no]:
                    Lambda.insert(Lambda_no, eigenvalue[eigenvalue_no])
                    InsertedCheck = True
                    break

            # There's no value in list which is higher than this new value
            if InsertedCheck == False:
                Lambda.append(eigenvalue[eigenvalue_no])

    # print('Lambda 0 to 3:', Lambda)

    # Calculate the Surface Variation
    SurfaceVariation = Lambda[0] / (Lambda[0] + Lambda[1] + Lambda[2])
    # print('Surface Variation:', SurfaceVariation)

    return(SurfaceVariation)

def SaveFile(Pcd_File_Name, PCDList):

    np.savetxt('{}'.format(Pcd_File_Name), PCDList, delimiter=' ')
    print('Saved File: [{}].'.format(Pcd_File_Name))


def Cluster(Point, SampleDistance=2, min_samples=2, output_dir='output'):
    data = np.asarray(Point)
    model = DBSCAN(eps=SampleDistance, min_samples=min_samples)
    model.fit_predict(data)

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    noise = False
    unique_labels = set(model.labels_)
    if -1 in unique_labels:
        noise = True
        unique_labels.remove(-1)

    print('Total Found Clusters :', len(unique_labels))

    ClusterList = []

    for clusterIdx in unique_labels:
        cluster_points = [Point[data_no] for data_no in range(len(data)) if model.labels_[data_no] == clusterIdx]
        SaveFile(f'{output_dir}/Index_{clusterIdx}.xyz', cluster_points)
        ClusterList.extend(cluster_points)

    if noise:
        noise_points = [Point[data_no] for data_no in range(len(data)) if model.labels_[data_no] == -1]
        SaveFile(f'{output_dir}/Index_noise.xyz', noise_points)
        ClusterList.extend(noise_points)
        print('(There is cluster noise)')

    return ClusterList

def uvw(PointBase, Point):
    # Choose one vector as reference first (from the base point to one random point)
    # Find the vector which has smallest distance
    for i in range(1, len(Point)):
        # print(Point[i])
        # Distance
        d = math.sqrt(Sq2(Point[i][0] - PointBase[0]) + Sq2(Point[i][1] - PointBase[1]) + Sq2(Point[i][2] - PointBase[2]))

        # Get the vector reference which has smallest distance from base point
        if i == 1:  # initial
            d_min = d
            Point_ref = [Point[i][0], Point[i][1], Point[i][2]]
        else:
            if d < d_min:
                d_min = d
                Point_ref = [Point[i][0], Point[i][1], Point[i][2]]
    A = np.subtract(Point_ref, PointBase)

    ChosenPoint = []
    ChosenPointDot = []
    for PointNo in range(1, len(Point)):
        # Get the vector base and another point
        B = np.subtract(Point[PointNo], PointBase)
        AB_dot = np.dot(A, B)
        DotProdIsSmaller_sign = False
        # Initial value for chosen point
        if PointNo == 0:
            ChosenPointDot.append(AB_dot)  # Store the dotprod score
            ChosenPoint.append(Point[PointNo])  # Store the point

        else:
            # Compare the current dotprod with the latest dotprod
            for ChosenPointDot_no in range(0, len(ChosenPointDot)):
                if AB_dot < ChosenPointDot[ChosenPointDot_no]:
                    ChosenPointDot.insert(ChosenPointDot_no, AB_dot)
                    ChosenPoint.insert(ChosenPointDot_no, Point[PointNo])
                    DotProdIsSmaller_sign = True
                    break

            if DotProdIsSmaller_sign == False:
                ChosenPointDot.append(AB_dot)
                ChosenPoint.append(Point[PointNo])

    ChosenPoint = ChosenPoint[:3]
    FinalChosenPointD_list = []
    FinalChosenPoint = []

    for i in range(0, len(ChosenPoint)):
        # Distance
        d = math.sqrt(
            Sq2(ChosenPoint[i][0] - PointBase[0]) + Sq2(ChosenPoint[i][1] - PointBase[1]) + Sq2(ChosenPoint[i][2] - PointBase[2]))

        DistIsSmaller_Sign = False

        if i == 0:  # initial
            FinalChosenPointD_list.append(d)
            FinalChosenPoint.append([ChosenPoint[i][0], ChosenPoint[i][1], ChosenPoint[i][2]])

        else:
            for FinalChosenPoint_no in range(0, len(FinalChosenPoint)):
                if d < FinalChosenPointD_list[FinalChosenPoint_no]:
                    FinalChosenPointD_list.insert(FinalChosenPoint_no, d)
                    FinalChosenPoint.insert(FinalChosenPoint_no,
                                            [ChosenPoint[i][0], ChosenPoint[i][1], ChosenPoint[i][2]])
                    DistIsSmaller_Sign = True
                    break

            if DistIsSmaller_Sign == False:
                FinalChosenPointD_list.append(d)
                FinalChosenPoint.append([ChosenPoint[i][0], ChosenPoint[i][1], ChosenPoint[i][2]])
    FinalChosenPoint[2] = Point_ref
    return FinalChosenPoint

def CheckPointsInsideOBB(SampledPoint, OBBPoint):
    uvwPoint = uvw(OBBPoint[0], OBBPoint)
    CheckedPoint = SampledPoint

    u = np.subtract(uvwPoint[0], OBBPoint[0])
    v = np.subtract(uvwPoint[1], OBBPoint[0])
    w = np.subtract(uvwPoint[2], OBBPoint[0])
    i = np.subtract(CheckedPoint, OBBPoint[0])

    # Check if the point is inside the bounding box or not
    if 0 <= round(np.dot(i, u), 7) <= round(np.dot(u, u), 7):
        if 0 <= round(np.dot(i, v), 7) <= round(np.dot(v, v), 7):
            if 0 <= round(np.dot(i, w), 7) <= round(np.dot(w, w), 7):
                inside = True
            else:
                inside = False
        else:
            inside = False
    else:
        inside = False

    return inside

# # estNormal
def search_radius_vector_3d(pcd, pcd_tree, i, dis):

    pcd.colors[i] = [0, 0, 1] # 查詢點
    # [k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[i], k)
    [k, idx, _] = pcd_tree.search_radius_vector_3d(pcd.points[i], dis)
    np.asarray(pcd.colors)[idx[1:], :] = [0, 1, 0]
    # print(pcd.points[idx[0]])

    X = []
    Y = []
    Z = []
    C = []

    XSum = 0
    YSum = 0
    ZSum = 0
    # print('pcd.points[idx[j]] =', pcd.points[idx[0]])

    for j in range(0, len(idx)):
        X = pcd.points[idx[j]][0]
        Y = pcd.points[idx[j]][1]
        Z = pcd.points[idx[j]][2]
        XSum = XSum + pcd.points[idx[j]][0]
        YSum = YSum + pcd.points[idx[j]][1]
        ZSum = ZSum + pcd.points[idx[j]][2]

    XMean = XSum / len(idx)
    YMean = YSum / len(idx)
    ZMean = ZSum / len(idx)

    C.append([XMean, YMean, ZMean])
    return idx, C

def EstimateNormal(before_raw, before_ref):
    pcd_array = np.asarray(before_raw.points)
    Pcd = pcd_array.tolist()
    # print('cluster_no = ', no)
    # Pcd = File.ReadXyzFile('cluster/rawCluster{}.xyz'.format(no))
    for PcdNo in range(0, len(Pcd)):
        Pcd[PcdNo].append(0)
        Pcd[PcdNo].append(0)
        Pcd[PcdNo].append(0)
    File.SaveFile('checkresult/rawCluster', Pcd)

    pcd = before_ref
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamRadius(radius=4))

    pcd1 = pcd
    dis = 12
    samplepointsNormal = []
    pcd1.paint_uniform_color([1, 0, 0])

    for i in range(0, len(pcd1.points)):  # pcd1 480 points
        pcd_tree = o3d.geometry.KDTreeFlann(pcd1)
        knn, C = search_radius_vector_3d(pcd1, pcd_tree, i, dis)

        # o3d.visualization.draw_geometries([pcd1], "kdtree points", width=800, height=600, left=50, top=50,
        #                                   point_show_normal=False, mesh_show_wireframe=False, mesh_show_back_face=False)

        p = pcd1.points[i]
        v = pcd1.points[i] - C[0]
        n = pcd1.normals[i]

        d_v = math.sqrt(Sq2(v[0]) + Sq2(v[1]) + Sq2(v[2]))
        d_n = math.sqrt(Sq2(n[0]) + Sq2(n[1]) + Sq2(n[2]))

        angle = (math.acos(np.dot(v, n) / d_v * d_n))
        angle = (angle * 180) / math.pi

        if angle > 88:
            pcd1.normals[i][0] = -pcd1.normals[i][0]
            pcd1.normals[i][1] = -pcd1.normals[i][1]
            pcd1.normals[i][2] = -pcd1.normals[i][2]

        p = p.tolist()
        samplepointsNormal.append(p)
        samplepointsNormal[i].append(n[0])
        samplepointsNormal[i].append(n[1])
        samplepointsNormal[i].append(n[2])

    SaveFile('checkresult/refCluster.xyz', samplepointsNormal)


SourcePointFile_decrease = []
comparePointFile_decrease = []

start_time = time.time()

folder_path = "C:\\Users\\User\\Desktop\\陳昱廷\\Layering\\checkresult"
remove_specific_files(folder_path)

file_path = 'Clusterpart_no.txt'
with open(file_path, 'r') as file:
    Clusterpart_no = int(file.readline().strip())
# Clusterpart_no = revise_txt[0]
print('研磨凸點編號 = ', Clusterpart_no)

# 未研磨的凸點
rawmodel = sorted(glob.glob(os.path.join("checkresult/", "rawmodel_*")))
rawmodel_array = rawmodel
print(rawmodel_array)


# 讀取正在研磨凸點ㄉ檔案
rawmodel_file = None

for rawfile in rawmodel_array:
    if f"rawmodel_{Clusterpart_no}.xyz" in rawfile:
        rawmodel_file = rawfile
        break

rawmodel_source = ReadXyzFile(rawmodel_file)
print('rawmodel_source =  ', rawmodel_file)
# checkresult_source = ReadXyzFile(checkresult_file)
# print('checkresult_file =  ', checkresult_file)

# 切切點雲!
for source_no in range(0, len(rawmodel_source)):
    if Clusterpart_no == 0:
        if rawmodel_source[source_no][0] > 3.2 and rawmodel_source[source_no][0] < 34 and rawmodel_source[source_no][1] > -15 and rawmodel_source[source_no][1] < 4.5:
            SourcePointFile_decrease.append(rawmodel_source[source_no])
    elif Clusterpart_no == 1:
        if rawmodel_source[source_no][0] > -15.57 and rawmodel_source[source_no][0] < 23.4 and rawmodel_source[source_no][1] > -17.05 and rawmodel_source[source_no][1] < 4.66:
            SourcePointFile_decrease.append(rawmodel_source[source_no])
    elif Clusterpart_no == 2:
        if rawmodel_source[source_no][0] > -29 and rawmodel_source[source_no][0] < 8 and rawmodel_source[source_no][1] > -5 and rawmodel_source[source_no][1] < 6:
            SourcePointFile_decrease.append(rawmodel_source[source_no])
    elif Clusterpart_no == 3:
        if rawmodel_source[source_no][0] > -12.8 and rawmodel_source[source_no][0] < 21.53 and rawmodel_source[source_no][1] > -16.24 and rawmodel_source[source_no][1] < 4.9:
            SourcePointFile_decrease.append(rawmodel_source[source_no])
    elif Clusterpart_no == 4:
        if rawmodel_source[source_no][0] > -5 and rawmodel_source[source_no][0] < 40 and rawmodel_source[source_no][1] > -8 and rawmodel_source[source_no][1] < 4:
            SourcePointFile_decrease.append(rawmodel_source[source_no])
SaveFile('checkresult/rawmodelPointFile_{}.xyz'.format(Clusterpart_no), SourcePointFile_decrease)

# # OBB
rawmodelPointFile = f"checkresult/rawmodelPointFile_{Clusterpart_no}.xyz"
NameOfEdgeFile = 'checkresult/EdgePoint.xyz'
check_result = "check_result.txt"
NameOfRemovedPointsFile = 'checkresult/RemovedPoints.xyz'
NameOfUnremovedPointsFile = 'checkresult/UnremovedPoints.xyz'
NameOfSF_SampledPoints_ClusterFolder = 'checkresult'
NameOfSF_ClusterFolder = 'checkresult'
FinalFile = 'checkresult/FinalSF.xyz'

# # Parameter Setting
SurfVarThreshold = 0.02
KNNRadius = 10
K_number = 250
OrientedBoundingBox_Offset = 1  # 0.5 # 1 # 0.2
SF_BB_offset = OrientedBoundingBox_Offset + 0.5  # 2   # 1.7
tree = KDTree(SourcePointFile_decrease, leaf_size=40)
EdgePoint = []
NewEdge = []


for Pcd_no in range(0, len(SourcePointFile_decrease)):
    # //Prepare the sampled points(KNN Points) for this chosen point0
    MainPoint = [SourcePointFile_decrease[Pcd_no]]
    SampledPoints = SampleThePoint(MainPoint, tree, K_number=K_number, KNNRadius=KNNRadius)

    # //SampledPoints < 3, we assume as noise point.
    if len(SampledPoints) >= 3:
        # Start to calculate the surface variation
        SurfaceVariation = SurfVar(SampledPoints)
    else:
        SurfaceVariation = 0.0
        print('SampledPoints less than 3.')

    if SurfaceVariation >= SurfVarThreshold:
        EdgePoint.append(MainPoint[0])

# Tell user if there's edge point
if len(EdgePoint) == 0:
    print('No Edge Point is detected')
    re_finish = open(check_result, "w")
    re_finish.write(str(1))
    re_finish.close()
    sys.exit()
else:
    # print('EdgePoint:', EdgePoint[5447][1])
    print('Edge Points are detected')
    mid_time = time.time()
    print('[Duration:', int((mid_time - start_time) / 60), 'minutes', int((mid_time - start_time) % 60), 'seconds]')
    re_finish = open(check_result, "w")
    re_finish.write(str(2))
    re_finish.close()
    SaveFile(NameOfEdgeFile, EdgePoint)



# //---------------------------- CLUSTER AND BOUNDING BOX THE EDGE POINT ------------------------------------------
print('\nCLUSTER AND BOUNDING BOX THE EDGE POINT')
EdgePoint_Cluster = Cluster(EdgePoint, SampleDistance=3, min_samples=2)

OBBPoint_list = []
OBBPoint_SF_list = []  # This is for surface fitting
OBBPoint_edge_list = []  # This is for list edge

print(len(EdgePoint_Cluster))
SaveFile('checkresult/Cluster{}.xyz'.format(Clusterpart_no), EdgePoint_Cluster)

OBBMidPoint, OBBPoint, HalfExtent_lengths, OBBPointSF = OrientedBoundingBox(EdgePoint_Cluster, offset=OrientedBoundingBox_Offset, SF_BB_offset=SF_BB_offset)
OBBPoint_list = OBBPoint       # Bounding Box for Removed Point
OBBPoint_SF_list = OBBPointSF  # Bounding Box for Surface Fitting

SaveFile('checkresult/OBBpoint{}.xyz'.format(Clusterpart_no), OBBPoint_list)
SaveFile('checkresult/OBBpoint_SF{}.xyz'.format(Clusterpart_no), OBBPoint_SF_list)

# // ----------- SEPARATE REMOVED AND UNREMOVED POINTS ---------------------------
print('\nSEPARATE REMOVED AND UNREMOVED POINTS')
# Method: Check if the point cloud is inside the bounding box, then we set it as removed point.
RemovedPoints = []
UnRemovedPoints = []
# Check the point one by one
for Pcd_no in range(0, len(SourcePointFile_decrease)):
    InsidePoints = CheckPointsInsideOBB(SourcePointFile_decrease[Pcd_no], OBBPoint_list)
    if InsidePoints == True:
        RemovedPoints.append(SourcePointFile_decrease[Pcd_no])
    if InsidePoints != True:
        UnRemovedPoints.append(SourcePointFile_decrease[Pcd_no])

# Save the removed and unremoved point
SaveFile(NameOfRemovedPointsFile, RemovedPoints)
SaveFile(NameOfUnremovedPointsFile, UnRemovedPoints)

# // ----------- FIND THE HOLE PARTS AND FILL THEM WITH SURFACE FITTING   ---------------------------
print('\nFIND THE HOLE PARTS AND FILL THEM WITH SURFACE FITTING')
# Method: Check if the point cloud is inside the bounding box(FOR SURFACE FITTING), then we set it as Surface Fitting sampled point.
SF_SampledPoints = []  # This is for collecting points around the removed parts which will be used as sample point for surface fitting.
for UnRemovedPoints_no in range(0, len(UnRemovedPoints)):
        InsidePoints = CheckPointsInsideOBB(UnRemovedPoints[UnRemovedPoints_no], OBBPoint_SF_list)
        if InsidePoints == True:
            SF_SampledPoints.append(UnRemovedPoints[UnRemovedPoints_no])
SaveFile('{}/SF_SampledPoints_Cluster_{}.xyz'.format(NameOfSF_SampledPoints_ClusterFolder, Clusterpart_no), SF_SampledPoints)

print('\nSURFACE FITTING')
SF_Grid_List = []
SF_Grid_Filtered = []
# SF_Grid = SurfaceFit(SF_SampledPoints[SF_BoundingBox_Cluster_No], GridPoint_dist=EachSurfaceFittingPoint_Distance)
file = "checkresult/SF_SampledPoints_Cluster_{}.xyz".format(Clusterpart_no)
OBBsffile = "checkresult/OBBpoint_SF{}.xyz".format(Clusterpart_no)
Grid, tra_inv, tra, XYZC_axiel = Nsurf.XYZchange(file, OBBsffile)
SaveFile('checkresult/tra_inv_{}.xyz'.format(Clusterpart_no), tra_inv)  # tra_inv:算完反推原坐標系
SaveFile('checkresult/tra_{}.xyz'.format(Clusterpart_no), tra)  # tra:原點轉換到凸點原點
SaveFile('checkresult/XYZC_axiel_{}.xyz'.format(Clusterpart_no), XYZC_axiel)
SaveFile('checkresult/grid_{}.xyz'.format(Clusterpart_no), Grid)
grid_before = o3d.io.read_point_cloud('checkresult/grid_{}.xyz'.format(Clusterpart_no))
grid_after = grid_before.transform(tra_inv)
o3d.io.write_point_cloud("checkresult/grid_after_{}.xyz".format(Clusterpart_no), grid_after)
sfgrid = "checkresult/grid_after_{}.xyz".format(Clusterpart_no)
SF_Grid = ReadXyzFile('{}'.format(sfgrid))

# SF_Grid = ReadXyzFile(file)



for SF_Grid_no in range(0, len(SF_Grid)):
    InsidePoints = CheckPointsInsideOBB(SF_Grid[SF_Grid_no], OBBPoint_list)
    if InsidePoints == True:
        SF_Grid_Filtered.append(SF_Grid[SF_Grid_no])
        # print('Removed the point no:", Pcd_no)
        SF_Grid_List.append(SF_Grid[SF_Grid_no])

SaveFile('checkresult/SF_Grid_List.xyz', SF_Grid_List)


end_time = time.time()
print('[Duration:', int((end_time - start_time) / 60), 'minutes', int((end_time - start_time) % 60), 'seconds]')
print('\n---------- PROGRAM END ----------\n')
