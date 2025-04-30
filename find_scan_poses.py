import open3d as o3d
import os as os
import glob

import os as os
import open3d as o3d
import numpy as np
import math
import Estnor as est
from numpy.linalg import inv

import time
import shutil
import re

def Sq2(value):
    # Code untuk Kuadrat bilangan
    return value*value

def ReadXyzFile(filename):
    print('File Path:', filename)
    f = open(filename, "r")
    lines = f.readlines()
    print('No of Points [XYZ]:', len(lines))
    PointList = []

    for x in range(0, len(lines)):
        RawData = lines[x].strip().split()  # [x y z] from File
        # print('r = ', len(RawData))
        if len(RawData) > 3:
            PointList.append(
                [float(RawData[0]), float(RawData[1]), float(RawData[2]), float(RawData[3]), float(RawData[4]),
                 float(RawData[5])])
        else:
            PointList.append([float(RawData[0]), float(RawData[1]), float(RawData[2])])

    return PointList

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

def cos(radius):
    '''
    [自訂] 輸入radius 輸出其cos
    '''
    return math.cos(radius)

def sin(radius):
    '''
    [自訂] 輸入radius 輸出其sin
    '''
    return math.sin(radius)

def getHomoTransferMat(configuration):
    '''
    [自訂] 輸入np.array[x,y,z(mm)u,v,w(degree)] 輸出homogeneous transform matrix
    '''
    x = configuration[0]; y = configuration[1]; z = configuration[2]
    u = math.radians(configuration[3]); v = math.radians(configuration[4]); w = math.radians(configuration[5])
    return np.array([[cos(w)*cos(v), -cos(u)*sin(w)+cos(w)*sin(v)*sin(u), cos(w)*cos(u)*sin(v)+sin(w)*sin(u), x],
                    [cos(v)*sin(w), cos(w)*cos(u)+sin(w)*sin(v)*sin(u), cos(u)*sin(w)*sin(v)-cos(w)*sin(u), y],
                    [-sin(v), cos(v)*sin(u), cos(v)*cos(u), z],
                    [0, 0, 0, 1]])

def getConfiguration_SIM(transferMat):
    '''[自訂] '''
    x = transferMat[0, 3] / 1000
    y = transferMat[1, 3] / 1000
    z = transferMat[2, 3] / 1000

    # # 對 transferMat[2, 0] 進行範圍檢查
    # if transferMat[2, 0] < -1.0:
    #     transferMat[2, 0] = -1.0
    # elif transferMat[2, 0] > 1.0:
    #     transferMat[2, 0] = 1.0

    V1 = -math.asin(transferMat[2, 0])
    V2 = math.pi + math.asin(transferMat[2, 0])

    Xp = math.sqrt(transferMat[0, 0]**2 + transferMat[1, 0]**2)

    epsilon = 1e-6  # 定義一個非常小的閾值

    if abs(Xp) < epsilon:
        W1 = 0
        W2 = math.pi
        U1 = math.atan2(-transferMat[1, 2], transferMat[1, 1])
        U2 = math.atan2(transferMat[1, 2], -transferMat[1, 1])
    else:
        W1 = math.atan2(transferMat[1, 0] / math.cos(V1), transferMat[0, 0] / math.cos(V1))
        W2 = math.atan2(transferMat[1, 0] / math.cos(V2), transferMat[0, 0] / math.cos(V2))
        U1 = math.atan2(transferMat[2, 1] / math.cos(V1), transferMat[2, 2] / math.cos(V1))
        U2 = math.atan2(transferMat[2, 1] / math.cos(V2), transferMat[2, 2] / math.cos(V2))

    config_1 = np.array([x, y, z, math.degrees(U1), math.degrees(V1), math.degrees(W1)])
    config_2 = np.array([x, y, z, math.degrees(U2), math.degrees(V2), math.degrees(W2)])

    return config_1, config_2

def getConfiguration_HIWIN(transferMat):
    '''[自訂] '''
    x = transferMat[0, 3]
    y = transferMat[1, 3]
    z = transferMat[2, 3]
    V1 = -math.asin(transferMat[2, 0])
    V2 = math.pi+math.asin(transferMat[2, 0])

    Xp = math.sqrt(transferMat[0, 0]**2 + transferMat[1, 0]**2)

    if (Xp==0):
        W1 = 0
        W2 = math.pi
        U1 = math.atan2(transferMat[1, 2], -transferMat[1, 1])
        U2 = math.atan2(-transferMat[1, 2], transferMat[1, 1])
    else:
        W1 = math.atan2(transferMat[1, 0]/math.cos(V1), transferMat[0, 0]/math.cos(V1))
        W2 = math.atan2(transferMat[1, 0]/math.cos(V2), transferMat[0, 0]/math.cos(V2))
        U1 = math.atan2(transferMat[2, 1]/math.cos(V1), transferMat[2, 2]/math.cos(V1))
        U2 = math.atan2(transferMat[2, 1]/math.cos(V2), transferMat[2, 2]/math.cos(V2))

    config_1 = np.array([x, y, z, math.degrees(U1), math.degrees(V1), math.degrees(W1)])
    config_2 = np.array([x, y, z, math.degrees(U2), math.degrees(V2), math.degrees(W2)])

    return config_1, config_2

# todo 從obb創造出適合我用的 local_axes    tra_local_axes
# todo 將手臂姿態值耶存到c++那

def compute_mean_with_normal(PL):

    X = []
    Y = []
    Z = []
    Xnor = []
    Ynor = []
    Znor = []

    XSum = 0
    YSum = 0
    ZSum = 0
    Xnorsum = 0
    Ynorsum = 0
    Znorsum = 0

    for PointNo in range(0, len(PL)):
        X.append(PL[PointNo][0])
        Y.append(PL[PointNo][1])
        Z.append(PL[PointNo][2])
        Xnor.append(PL[PointNo][3])
        Ynor.append(PL[PointNo][4])
        Znor.append(PL[PointNo][5])

        XSum = XSum + PL[PointNo][0]
        YSum = YSum + PL[PointNo][1]
        ZSum = ZSum + PL[PointNo][2]
        Xnorsum = Xnorsum + PL[PointNo][3]
        Ynorsum = Ynorsum + PL[PointNo][4]
        Znorsum = Znorsum + PL[PointNo][5]

    XMean = XSum / len(PL)
    YMean = YSum / len(PL)
    ZMean = ZSum / len(PL)
    XnorMean = Xnorsum / len(PL)
    YnorMean = Ynorsum / len(PL)
    ZnorMean = Znorsum / len(PL)
    Mean = [[XMean, YMean, ZMean]]
    norMean = [[XnorMean, YnorMean, ZnorMean]]

    return Mean, norMean

def euclidean_distance(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 +(point1[1] - point2[1]) ** 2 +(point1[2] - point2[2])) ** 2

def get_local_axes(surface_PATH, ext_idx, obb_PATH):

    nor_surface_PL = est.Estnor(file=surface_PATH) # [ritz] 估算法向量並使其一致 有法向量的擬合面
    surface_PL = ReadXyzFile('{}'.format(surface_PATH)) # 沒有法向量的擬合面
    obb_PL = ReadXyzFile('{}'.format(obb_PATH))

    Mean_PL, Mean_norPL= compute_mean_with_normal(nor_surface_PL) # CP CPnor


    # base_point
    # 條件 [1]nor方向最小的4個(最貼近表面) [2]符合1者camera座標x和y值最小的
    projections = np.dot(obb_PL, Mean_norPL) # 計算每個點在 Mean_norPL 方向上的投影
    min_indices = np.argsort(projections)[:4] # [1]nor方向最小的4個(最貼近表面)
    surface_obb_PL = obb_PL[min_indices]  # 根據索引篩選出底層點
    max_indices = np.argsort(projections)[-4:]  # 在上層的4個obb點
    upper_obb_PL = obb_PL[max_indices ]# 根據索引篩選出上層點

    base_point = min(surface_obb_PL, key=lambda p: (p[0], p[1])) # [2]符合1者camera座標x和y值最小的

    # x y 方向的點
    dist_surface_to_base = np.linalg.norm(surface_obb_PL - base_point, axis=1)
    sorted_indices = np.argsort(dist_surface_to_base)
    base_x_point = surface_obb_PL[sorted_indices[1]]  # 距離最小的點
    base_y_point = surface_obb_PL[sorted_indices[2]]  # 距離第二小的點

    # z 方向的點
    distances_upper_to_base = np.linalg.norm(upper_obb_PL - base_point, axis=1)
    base_z_point = upper_obb_PL[np.argmin(distances_upper_to_base)] # 上層和base_point最近的點

    local_axes = [base_point, base_x_point, base_y_point, base_z_point]
    SaveFile('local_axes', local_axes)

    return local_axes

# 常數
SCAN_CONFIG = np.array([-20, 1001, 660, -90, -90, 0]) # 我自己抓的點
# WHOLE_MODEL_SCAN_CONFIG = np.array([-20, 1116, 660, -90, -90, 0]) # 學姊的的拍照位置
TRAs_FILE = sorted(glob.glob(os.path.join("Source/", "tra*.xyz")))
EXTRUDED_NUM = np.size(TRAs_FILE)

print("已讀取{}個檔案\n".format(np.size(EXTRUDED_NUM)), TRAs_FILE)

for extruded_idex in range(EXTRUDED_NUM):
    # 路徑
    BASE_TRANSFER_MAT_PATH = "Source/tra_2.xyz"
    targetTraMat_PATH = "Source/tra_{}.xyz".format(extruded_idex)

    # 取 homogeneous transform matrix
    baseTraMat = np.genfromtxt(BASE_TRANSFER_MAT_PATH, dtype=None, comments='#', delimiter=' ')
    targetTraMat = np.genfromtxt(targetTraMat_PATH, dtype=None, comments='#', delimiter=' ')
    scanPosTraMat = getHomoTransferMat(SCAN_CONFIG)

    # 基於 baseTraMat 找到其餘凸點在掃描點的config
    targetScanTraMat = scanPosTraMat @ np.linalg.inv(baseTraMat) @ targetTraMat
    targetScanConf_1, targetScanConf_2 = getConfiguration_SIM(targetScanTraMat)

    # 存檔
    # 給程式跑、模擬器跑
    formatted_targetScanConf_1 = np.array([targetScanConf_1])  # 為了讓存檔時 一排有6個資訊(用空格隔開)
    SaveFile("Result/ScanPos_{}.xyz".format(extruded_idex), formatted_targetScanConf_1)
    # 直接餵點給上銀跑(不是給跑軌跡點的!)
    targetScanConf_1, targetScanConf_2 = getConfiguration_HIWIN(targetScanTraMat)
    formatted_targetScanConf_1 = np.array([targetScanConf_2])  # 為了讓存檔時 一排有6個資訊(用空格隔開)
    SaveFile("Result/ScanPos_HIWIN_{}.xyz".format(extruded_idex), formatted_targetScanConf_1)
