'''
將研磨後尚有殘餘部分依照高度標示顏色
並依據殘餘高度推算偏轉角度
testing: 現測試計算偏轉角度並反推 (成功)
'''
import open3d as o3d
import os as os
import glob

import os as os
import open3d as o3d
import numpy as np
import math
from numpy.linalg import inv
from scipy.spatial import KDTree
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.colors import TwoSlopeNorm
from scipy.spatial.transform import Rotation as R

import time
import shutil
import re

def Sq2(value):
    # Code untuk Kuadrat bilangan
    return value*value

def SaveFile(Pcd_File_Name, PCDList):

    np.savetxt('{}'.format(Pcd_File_Name), PCDList, delimiter=' ')
    print('Saved File: [{}].'.format(Pcd_File_Name))

def ReadXyzNorFile(filename):
    print('File Path:   ', filename)
    f = open(filename, "r")
    lines = f.readlines()
    # print('No of Points [XYZ]:', len(lines))
    PointList = []

    for x in range(0, len(lines)):
        RawData = lines[x].strip().split() #[x y z] from File
        # print('r = ', len(RawData))
        if len(RawData) > 3:
            PointList.append([float(RawData[0]), float(RawData[1]), float(RawData[2]), float(RawData[3])])
        else:
            PointList.append([float(RawData[0]), float(RawData[1]), float(RawData[2])])

    return PointList

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
        #
        # X = [round(i, 3) for i in X]
        # Y = [round(i, 3) for i in Y]
        # Z = [round(i, 3) for i in Z]
        #
        # XSum = math.floor(XSum + Point[PointNo][0] * 1000) / 1000.0
        # YSum = math.floor(YSum + Point[PointNo][1] * 1000) / 1000.0
        # ZSum = math.floor(ZSum + Point[PointNo][2] * 1000) / 1000.0

        XSum = XSum + Point[PointNo][0]
        YSum = YSum + Point[PointNo][1]
        ZSum = ZSum + Point[PointNo][2]


    XMean = XSum / len(Point)
    YMean = YSum / len(Point)
    ZMean = ZSum / len(Point)

    # XMean = math.floor(XSum / len(Point) * 1000) / 1000.0
    # YMean = math.floor(YSum / len(Point) * 1000) / 1000.0
    # ZMean = math.floor(YSum / len(Point) * 1000) / 1000.0

    # print('Mean', XMean, YMean, ZMean)
    return X, Y, Z, XMean, YMean, ZMean

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

    # print('\nPoint_ref and Point_base')
    # print(Point_ref)
    # print(PointBase)

    # Get the vector base and point reference (A)
    A = np.subtract(Point_ref, PointBase)

    ChosenPoint = []
    ChosenPointDot = []
    for PointNo in range(1, len(Point)):
        # Get the vector base and another point
        B = np.subtract(Point[PointNo], PointBase)
        # print('\nDot product to point:', Point[PointNo])
        AB_dot = np.dot(A, B)
        # print('dotprod:', AB_dot)

        # Sort the dot prod from smallest to largest:

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
        #
        # print('\n Chosen point dot')
        # print(ChosenPointDot)

    # Just take the 3 smallest dot prod value
    ChosenPoint = ChosenPoint[:3]
    # print('\nChosen points:')
    # print(ChosenPoint)

    # Filter the chosen point which has smallest distance (max 2 points)
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

    # print('\n Final Chosen point')
    # print(FinalChosenPoint)
    # print('Base Point')
    # print(PointBase)
    return FinalChosenPoint

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

    # print('\nShow the center and all bounding box point:')
    # print(mid)
    # print('Bottom Side of Box(Counter Clockwise Rotation)')
    # print(XminYminZmin)
    # print(XmaxYminZmin)
    # print(XmaxYmaxZmin)
    # print(XminYmaxZmin)
    # print('Top Side of Box(Counter Clockwise Rotation)')
    # print(XminYminZmax)
    # print(XmaxYminZmax)
    # print(XmaxYmaxZmax)
    # print(XminYmaxZmax)

    # // Store the BoundingBox Points.
    BBPoint = [XminYminZmin, XmaxYminZmin, XmaxYmaxZmin, XminYmaxZmin, XminYminZmax, XmaxYminZmax, XmaxYmaxZmax,
               XminYmaxZmax]
    BBPoint_SF = [XminYminZmin_SF, XmaxYminZmin_SF, XmaxYmaxZmin_SF, XminYmaxZmin_SF, XminYminZmax_SF, XmaxYminZmax_SF, XmaxYmaxZmax_SF,
               XminYmaxZmax_SF]

    return mid, BBPoint, HalfExtent_lengths, BBPoint_SF

def CheckPointsInsideOBB(SampledPoint, OBBPoint):
    # print('\nOBB point', OBBPoint)
    # // Have to wider the OBBPoint
    # print('OBBPoint', OBBPoint)

    # // Check if points are insert bounding box, OBBPoint, SampledPoint
    # 1. Get 4 points for the requirement of the next function
    # Illustration:
    #            . (Point 1)
    #            .
    #            .
    #            .
    #            .
    #  (Point 0) . . . . . . . (Point 2)
    #           .
    # (Point 3).

    # Algorithm:
    # The diagonal distance will be more far than not

    uvwPoint = uvw(OBBPoint[0], OBBPoint)
    # print('OBB_BasePoint', OBBPoint[0])
    # print('OBB_uvwPoint', uvwPoint)


    # 2. Check if the sampled points are inside the OBB
    # print('\nCheck if the points are inside the OBB')

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

    # if inside == True:
    #     print('Inside')
    # else:
        # print('Outside')
        # print(round(np.dot(i, u), 7), round(np.dot(u, u), 7))
        # print(round(np.dot(i, v), 7), round(np.dot(v, v), 7))
        # print(round(np.dot(i, w), 7), round(np.dot(w, w), 7))
        # print('\n')

    return inside

def CheckPointsInsideOBB_noMatterZ(SampledPoint, OBBPoint):

    uvwPoint = uvw(OBBPoint[0], OBBPoint)

    CheckedPoint = SampledPoint

    u = np.subtract(uvwPoint[0], OBBPoint[0])
    v = np.subtract(uvwPoint[1], OBBPoint[0])
    w = np.subtract(uvwPoint[2], OBBPoint[0])
    i = np.subtract(CheckedPoint, OBBPoint[0])

    # Check if the point is inside the bounding box or not
    if 0 <= round(np.dot(i, u), 7) <= round(np.dot(u, u), 7):
        if 0 <= round(np.dot(i, v), 7) <= round(np.dot(v, v), 7):
            inside = True
        else:
            inside = False
    else:
        inside = False

    # if inside == True:
    #     print('Inside')
    # else:
        # print('Outside')
        # print(round(np.dot(i, u), 7), round(np.dot(u, u), 7))
        # print(round(np.dot(i, v), 7), round(np.dot(v, v), 7))
        # print(round(np.dot(i, w), 7), round(np.dot(w, w), 7))
        # print('\n')

    return inside

# 創建目錄 ColorHeightResidualMarking，如果尚未存在
output_dir = "ColorHeightResidualMarking"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# 讀取檔案
golden_path = 'checkresult/SF_Grid_List.xyz'
afterGrind_path = 'checkresult/comparePointFile_3.xyz'
golden_PL = ReadXyzFile(golden_path)
afterGrind_PL = ReadXyzFile(afterGrind_path)

# 裁切檔案
in_afterGrind_PL = []
out_afterGrind_PL = []
_, BB_point, _, _ = OrientedBoundingBox(golden_PL, 0, 0)
for PL_index, PL_point in enumerate(afterGrind_PL):
    # Check if the point is inside in the bounding box
    isInside = CheckPointsInsideOBB_noMatterZ(PL_point, BB_point)
    if isInside:
        in_afterGrind_PL.append(PL_point)
    else:
        out_afterGrind_PL.append(PL_point)

# KDTree找尋高度 (從上往下找)
tree = KDTree(golden_PL)
distances, indices = tree.query(in_afterGrind_PL)  # 對曲面b的每個點找到曲面a中的最近點

# 判斷距離正負
golden_NP = np.array(golden_PL)
in_afterGrind_NP = np.array(in_afterGrind_PL)
golden_z_values = golden_NP[indices, 2]  # 根據 indices 挑選對應序號的z值
in_afterGrind_z_values = in_afterGrind_NP[:, 2]
signed_distances = np.where(in_afterGrind_z_values > golden_z_values, distances, -distances)

# 取得 signed_distances 的最大值和最小值
min_distance = signed_distances.min()
max_distance = signed_distances.max()
max_abs_distance = max(abs(min_distance), abs(max_distance))

print(f"最小值: {min_distance}")
print(f"最大值: {max_distance}")

# 生成自定義色彩映射，對應負數和正數部分
colors_under_0 = plt.cm.PiYG(np.linspace(0, 0.3, 128))  # 負數部分用 PiYG
colors_over_0 = plt.cm.viridis(np.linspace(0, 1, 128))  # 正數部分用 viridis
custom_colors = np.vstack((colors_under_0, colors_over_0))
custom_cmap = LinearSegmentedColormap.from_list('CustomPiYG_Viridis', custom_colors)

# 使用 TwoSlopeNorm 以 0 為中心來正規化數據
norm = TwoSlopeNorm(vmin=-max_abs_distance, vcenter=0, vmax=max_abs_distance)
colors = custom_cmap(norm(signed_distances))

# 可視化
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(in_afterGrind_NP[:, 0], in_afterGrind_NP[:, 1], in_afterGrind_NP[:, 2], c=colors[:, :3])

# 儲存點雲
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(in_afterGrind_NP)
pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
o3d.io.write_point_cloud("ColorHeightResidualMarking/colored_point_cloud_custom_viridis_piyg_signed.ply", pcd)
print("ColorHeightResidualMarking/點雲已保存為 colored_point_cloud_custom_viridis_piyg_signed.ply")

# 顯示顏色條，並設置均勻的刻度
mappable = plt.cm.ScalarMappable(norm=norm, cmap=custom_cmap)
cbar = plt.colorbar(mappable, shrink=0.5, aspect=5, label='Signed Distance')

# 設置顏色條刻度，讓正負刻度相同
cbar_ticks = np.linspace(-max_abs_distance, max_abs_distance, num=10)
cbar.set_ticks(cbar_ticks)
cbar.set_ticklabels([f'{tick:.2f}' for tick in cbar_ticks])

plt.show()
# --------------------------<平面化>--------------------------
# 過濾掉 signed_distances < 0 的部分
mask = signed_distances > 0.4
filtered_points = in_afterGrind_NP[mask]
filtered_signed_distances = signed_distances[mask]
filtered_colors = colors[mask]  # 保留對應的顏色

# 更新過濾後的點雲，並將 z 值設為 filtered_signed_distances
filtered_pcd = o3d.geometry.PointCloud()
filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
filtered_pcd.colors = o3d.utility.Vector3dVector(filtered_colors[:, :3])

# 將過濾後的 z 值設為 signed_distances
points = np.asarray(filtered_pcd.points)
points[:, 2] = filtered_signed_distances  # 將 z 值設為 signed_distances
filtered_pcd.points = o3d.utility.Vector3dVector(points)

# 儲存過濾後的點雲
o3d.io.write_point_cloud("ColorHeightResidualMarking/filtered_point_cloud_with_signed_distances.ply", filtered_pcd)
print("已將 z 值改為 signed_distances 並保存為 ColorHeightResidualMarking/filtered_point_cloud_with_signed_distances.ply")

# --------------------------<產生z為0的參照平面>--------------------------
# 複製 x 和 y 值，並將 z 值設為 0
xy_values = in_afterGrind_NP[:, :2]  # 提取 x 和 y
z_values = np.zeros(xy_values.shape[0])  # 生成 z 為 0 的陣列

# 合併 x, y 和新的 z
xy_plane_points = np.column_stack((xy_values, z_values))

# 生成新的點雲平面
xy_plane_pcd = o3d.geometry.PointCloud()
xy_plane_pcd.points = o3d.utility.Vector3dVector(xy_plane_points)

# 保存這個平面
o3d.io.write_point_cloud("ColorHeightResidualMarking/xy_plane_at_z0.ply", xy_plane_pcd)
print("已生成並保存 z 為 0 的平面點雲 ColorHeightResidualMarking/xy_plane_at_z0.ply")

# --------------------------<產生高度差的擬和平面>--------------------------
# 讀取已過濾的點雲資料
filtered_pcd = o3d.io.read_point_cloud("ColorHeightResidualMarking/filtered_point_cloud_with_signed_distances.ply")

# 使用 RANSAC 擬合平面
plane_model, inliers = filtered_pcd.segment_plane(distance_threshold=0.01,
                                                  ransac_n=3,
                                                  num_iterations=1000)

# 擬合平面方程式參數
[a, b, c, d] = plane_model
print(f"擬合平面方程式: {a}x + {b}y + {c}z + {d} = 0")

# 提取點雲的範圍來生成新的平面
points = np.asarray(filtered_pcd.points)
min_x, max_x = np.min(points[:, 0]), np.max(points[:, 0])
min_y, max_y = np.min(points[:, 1]), np.max(points[:, 1])

# 生成一個新的平面點雲
xx, yy = np.meshgrid(np.linspace(min_x, max_x, 100), np.linspace(min_y, max_y, 100))
zz = (-a * xx - b * yy - d) / c  # 計算平面上對應的 z 值

# 將平面數據轉換為點雲格式
height_diff_points = np.c_[xx.ravel(), yy.ravel(), zz.ravel()] # ravel: 將網格中的值展平誠一維
height_diff_pcd = o3d.geometry.PointCloud()
height_diff_pcd.points = o3d.utility.Vector3dVector(height_diff_points)

# 儲存擬合的平面點雲
o3d.io.write_point_cloud("ColorHeightResidualMarking/height_diff_plane.ply", height_diff_pcd)
print("已生成並保存擬合的平面點雲 ColorHeightResidualMarking/height_diff_plane.ply")

# --------------------------<取得平面旋轉資訊後 將平面轉回與z0平行>--------------------------
normal_z0 = np.array([0, 0, 1]) # 參照平面的法向量
normal_height_diff = np.array([a, b, c])

# 計算旋轉軸（通過叉積計算）
rotationAxis = np.cross(normal_height_diff, normal_z0)
rotationAxis = rotationAxis / np.linalg.norm(rotationAxis)  # 規範化

# 計算旋轉角度（通過點積計算）
rotationAngle = np.arccos(np.dot(normal_height_diff, normal_z0) / (np.linalg.norm(normal_height_diff) * np.linalg.norm(normal_z0)))

# 生成旋轉矩陣
rotation = R.from_rotvec(rotationAngle * rotationAxis)

# 暫時將平面的原點平移至平面中心 旋轉後再移回來
centroid = np.mean(np.asarray(height_diff_pcd.points), axis=0)
centered_points = np.asarray(height_diff_pcd.points) - centroid
rotated_points = rotation.apply(centered_points)
rotated_height_diff_points = rotated_points + centroid

# 點雲化
rotated_height_diff_pcd = o3d.geometry.PointCloud()
rotated_height_diff_pcd.points = o3d.utility.Vector3dVector(rotated_height_diff_points)

# 儲存
o3d.io.write_point_cloud("ColorHeightResidualMarking/rotated_height_diff_plane.ply", rotated_height_diff_pcd)
print("已生成並保存擬合的平面點雲 ColorHeightResidualMarking/rotated_height_diff_plane.ply")


