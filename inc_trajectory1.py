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
from numpy.linalg import inv
import single_pro_euler

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

def ReadXyzNorFile(filename):
    print('File Path:', filename)
    f = open(filename, "r")
    lines = f.readlines()
    print('No of Points [XYZ]:', len(lines))
    PointList = []

    for x in range(0, len(lines)):
        RawData = lines[x].strip().split() #[x y z] from File
        # print('r = ', len(RawData))
        if len(RawData) > 3:
            PointList.append([float(RawData[0]), float(RawData[1]), float(RawData[2]), float(RawData[3]), float(RawData[4]), float(RawData[5])])
        else:
            PointList.append([float(RawData[0]), float(RawData[1]), float(RawData[2])])

    return PointList

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
    dis = 5
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


def Sq2(value):
    # Code untuk Kuadrat bilangan
    return value*value

start_time = time.time()

shutil.rmtree('c:\\Users\\User\\Desktop\\陳昱廷\\Layering\\inc_trajectory')
os.makedirs('c:\\Users\\User\\Desktop\\陳昱廷\\Layering\\inc_trajectory')
# 研磨完的凸點
checkresult = sorted(glob.glob(os.path.join("checkresult/", "checkresult_*")))
checkresult_array = checkresult

file_path = 'revise_Tpoint.txt'
revise = open(file_path, 'r')
line = revise.readlines()
revise_txt = []
for x in range(0, len(line)):
    revise_txt = line[x].split(' ', 1)
Clusterpart_no = revise_txt[0]
print('研磨凸點編號 = ', Clusterpart_no)

print(checkresult_array)
checkresult_file = None
for checkfile in checkresult_array:
    if f"checkresult_{Clusterpart_no}.xyz" in checkfile:
        checkresult_file = checkfile
        break
return_trajectory_file = "return_trajectory.txt"
checkresult_source = ReadXyzFile(checkresult_file)
comparePointFile_decrease = []
for source_no in range(0, len(checkresult_source)):
    if Clusterpart_no == "0":
        if checkresult_source[source_no][0] > 3.2 and checkresult_source[source_no][0] < 34 and checkresult_source[source_no][1] > -15 and checkresult_source[source_no][1] < 4.5:
            comparePointFile_decrease.append(checkresult_source[source_no])
    elif Clusterpart_no == "1":
        if checkresult_source[source_no][0] > -17.4 and checkresult_source[source_no][0] < 21.8 and checkresult_source[source_no][1] > -16.3 and checkresult_source[source_no][1] < 3.69:
            comparePointFile_decrease.append(checkresult_source[source_no])
    elif Clusterpart_no == "2":
        if checkresult_source[source_no][0] > -29 and checkresult_source[source_no][0] < 8 and checkresult_source[source_no][1] > -5 and checkresult_source[source_no][1] < 6:
            comparePointFile_decrease.append(checkresult_source[source_no])
    elif Clusterpart_no == "3":
        if checkresult_source[source_no][0] > -12.8 and checkresult_source[source_no][0] < 21.53 and checkresult_source[source_no][1] > -16.24 and checkresult_source[source_no][1] < 4.9:
            comparePointFile_decrease.append(checkresult_source[source_no])
    elif Clusterpart_no == "4":
        if checkresult_source[source_no][0] > -5 and checkresult_source[source_no][0] < 40 and checkresult_source[source_no][1] > -8 and checkresult_source[source_no][1] < 4:
            comparePointFile_decrease.append(checkresult_source[source_no])
SaveFile('checkresult/comparePointFile_{}.xyz'.format(Clusterpart_no), comparePointFile_decrease)

before_raw = o3d.io.read_point_cloud('checkresult/comparePointFile_{}.xyz'.format(Clusterpart_no))
before_ref = o3d.io.read_point_cloud('checkresult/SF_Grid_List.xyz')
EstimateNormal(before_raw, before_ref)

ref_File = 'checkresult/refCluster.xyz'
raw_File = 'checkresult/comparePointFile_{}.xyz'.format(Clusterpart_no)
reviseT_path = "revise_Tpoint.txt"

NewPoint, NewPointIdx, NewNormal = File.ReadXYZNormalFile(ref_File)

Vertice1 = File.ReadXyzFile(ref_File)
Vertice2 = File.ReadXyzFile(raw_File)

Vertice2 = np.asarray(Vertice2)
tree = KDTree(Vertice2, leaf_size=8)



print('\nCALCULATE THE RESIDUAL HEIGHT OF THE EXTRUDE PART')

tra = "transform/tra_{}.xyz".format(Clusterpart_no)
tra_inv = "transform/tra_inv_{}.xyz".format(Clusterpart_no)
Layer_height = 0.1  # mm
Layer1_height = 0.1

total_layer, result = Calc.Layering(Clusterpart_no, tra, tra_inv, NewPoint, NewPointIdx, NewNormal, tree, Vertice2, LayerDepth=Layer_height, Layer1Depth=Layer1_height)
res_extrudepart = round(result, 2)
print('res_extrudepart = ', res_extrudepart)
inc_layer_result = round(res_extrudepart / 0.4)
print('inc_layer_result = ', inc_layer_result)
data = [Clusterpart_no, inc_layer_result]
with open('Res_extrudepart.txt', 'w') as f:
    for num in data:
        f.write((str(num)) + ' ')

# 新增軌跡
protrusion = Clusterpart_no
layer = inc_layer_result
if layer == 1:
    ru = open(return_trajectory_file, "w")
    ru.write(str(1))
    ru.close()
elif layer == 0:
    ru = open(return_trajectory_file, "w")
    ru.write(str(3))
    ru.close()
else:
    ru = open(return_trajectory_file, "w")
    ru.write(str(2))
    ru.close()
    print('須新增層數: ', layer)
    input()
    print('凸點編號 = ', protrusion)
    Layer_num = sorted(glob.glob(os.path.join("Output File/", "*traj{}*".format(protrusion))), key=lambda x: (int(re.split('traj|_|_|.xyz', x)[1])))
    Layer_number = len(Layer_num)
    Layer = Layer_number-6
    print('{} 號凸點原本的層數 = '.format(protrusion), Layer)

    Layer_file = sorted(glob.glob(os.path.join("Output File/", "*traj{}_{}*".format(protrusion, Layer-3))), key=lambda x: (int(re.split('traj|_|_|.xyz', x)[1])))  # 倒數第二層
    tra_0 = [ReadXyzFile(file) for file in Layer_file]
    # print('0-tra_0 = ', tra_0)
    Layer_file = sorted(glob.glob(os.path.join("Output File/", "*traj{}_{}*".format(protrusion, Layer-2))), key=lambda x: (int(re.split('traj|_|_|.xyz', x)[1])))  # 最後一層
    tra_1 = [ReadXyzFile(file) for file in Layer_file]
    # print('0-tra_1 = ', tra_1)

    for tra_No in range(0, layer):
        if tra_No == 0:
            for j in range(0, 3):
                new_trajectory = np.array(tra_1[j]) - (np.array(tra_0[j]) - np.array(tra_1[j]))
                SaveFile('inc_trajectory/inctra{}_{}_{}.xyz'.format(protrusion, tra_No, j+1), new_trajectory)
        elif tra_No == 1:
            inctra_file = sorted(glob.glob(os.path.join("inc_trajectory/", "*inctra{}_{}*".format(protrusion, tra_No - 1))),
                                key=lambda x: (int(re.split('inctra|_|_|.xyz', x)[2])))
            # print('inctra_file', inctra_file)
            tra_0 = tra_1
            tra_1 = [ReadXyzFile(file) for file in inctra_file]
            for j in range(0, 3):
                new_trajectory = np.array(tra_1[j]) - (np.array(tra_0[j]) - np.array(tra_1[j]))
                SaveFile('inc_trajectory/inctra{}_{}_{}.xyz'.format(protrusion, tra_No, j+1), new_trajectory)
        elif tra_No >= 2:
            inctra_file_last2layers = sorted(glob.glob(os.path.join("inc_trajectory/", "*inctra{}_{}*".format(protrusion, tra_No - 2))),
                                            key=lambda x: (int(re.split('inctra|_|_|.xyz', x)[2])))
            inctra_file_last1layers = sorted(glob.glob(os.path.join("inc_trajectory/", "*inctra{}_{}*".format(protrusion, tra_No - 1))),
                                            key=lambda x: (int(re.split('inctra|_|_|.xyz', x)[2])))
            tra_0 = [ReadXyzFile(file) for file in inctra_file_last2layers]
            tra_1 = [ReadXyzFile(file) for file in inctra_file_last1layers]
            for j in range(0, 3):
                new_trajectory = np.array(tra_1[j]) - (np.array(tra_0[j]) - np.array(tra_1[j]))
                SaveFile('inc_trajectory/inctra{}_{}_{}.xyz'.format(protrusion, tra_No, j+1), new_trajectory)


    shutil.rmtree('c:\\Users\\User\\Desktop\\陳昱廷\\Layering\\grindcinc{}'.format(protrusion))
    os.makedirs('c:\\Users\\User\\Desktop\\陳昱廷\\Layering\\grindcinc{}'.format(protrusion))
    alltrajcoor = []
    linenor = []
    linenor_1 = []
    linenor_2 = []
    linenor_3 = []
    for linenor_no in range(1, 4):
        print('line = ', linenor_no)
        NOR = ReadXyzNorFile('cluster/refCluster{}.xyz'.format(protrusion))
        TRpoint = ReadXyzFile('inc_trajectory/inctra{}_{}_{}.xyz'.format(protrusion, layer - 1, linenor_no))
        # print('NOR[0]', NOR[0])
        # print('len(NOR)', len(NOR))
        for TRpoint_no in range(0, len(TRpoint)):
            # print(SMpoint_no)
            dx = []
            for NORpoint_no in range(0, len(NOR)):  # len(SamplePoint)
                dx.append(math.sqrt(Sq2((TRpoint[TRpoint_no][0] - NOR[NORpoint_no][0]))
                                    + Sq2((TRpoint[TRpoint_no][1] - NOR[NORpoint_no][1]))
                                    + Sq2((TRpoint[TRpoint_no][2] - NOR[NORpoint_no][2]))))
            # print(dx)
            mn = np.min(dx)
            # print(mn)

            for i in range(0, len(dx)):
                if dx[i] == mn:
                    print('NOR[i] = ', NOR[i])
                    linenor.append([NOR[i][3], NOR[i][4], NOR[i][5]])
    linenor_1 = linenor[0:10]
    linenor_2 = linenor[10:20]
    linenor_3 = linenor[20:30]
    for Bfile0_no in range(0, layer):

        # ---------------------------------------------------------------------------------------------------------
        print('\nlayer =', Bfile0_no)
        layerline = sorted(glob.glob(os.path.join("inc_trajectory/", "*inctra{}_{}_*".format(protrusion, Bfile0_no))), key=os.path.getmtime)

        for layerline_no in range(0, len(layerline)):

            print(len(layerline), '條軌跡------------------------------', layerline)
            print('layerline_no = ', layerline_no + 1)

            Sfile = ReadXyzFile(layerline[layerline_no])
            grindoint = np.array(Sfile)

            for PointNo in range(0, len(grindoint)):
                # print(PointNo)
                xp = []
                yp = []
                zp = []

                if Bfile0_no + 1 < layer - 2:
                    xnor = linenor_2[PointNo][0]
                    ynor = linenor_2[PointNo][1]
                    znor = linenor_2[PointNo][2]
                else:
                    if layerline_no + 1 == 1:
                        xnor = linenor_1[PointNo][0]
                        ynor = linenor_1[PointNo][1]
                        znor = linenor_1[PointNo][2]
                    elif layerline_no + 1 == 2:
                        xnor = linenor_2[PointNo][0]
                        ynor = linenor_2[PointNo][1]
                        znor = linenor_2[PointNo][2]
                    elif layerline_no + 1 == 3:
                        xnor = linenor_3[PointNo][0]
                        ynor = linenor_3[PointNo][1]
                        znor = linenor_3[PointNo][2]


                if PointNo == len(grindoint) - 1:  # 軌跡最後一個點

                    xv = np.subtract(grindoint[PointNo - 1], grindoint[PointNo])
                    xp = -xv
                    # dx = math.sqrt(Sq2(xv[0]) + Sq2(xv[1]) + Sq2(xv[2]))
                    # xp = xv / dx

                elif PointNo == 0:
                    xp = np.subtract(grindoint[PointNo + 2], grindoint[PointNo])

                else:
                    xp = np.subtract(grindoint[PointNo + 1], grindoint[PointNo])
                    # print('xv =', xv)

                zp.append([xnor, ynor, znor])   # 這邊zp好像被220行的zp蓋掉ㄌ!?
                zp = np.asarray(zp)

                # 軌跡點坐標系
                yp = np.cross(zp, xp)
                # xp = np.cross(yp, zp)
                zp = np.cross(xp, yp)

                xp = xp.flatten()
                yp = yp.flatten()
                zp = zp.flatten()

                dx = math.sqrt(Sq2(xp[0]) + Sq2(xp[1]) + Sq2(xp[2]))
                xp = xp / dx
                dy = math.sqrt(Sq2(yp[0]) + Sq2(yp[1]) + Sq2(yp[2]))
                yp = yp / dy
                dz = math.sqrt(Sq2(zp[0]) + Sq2(zp[1]) + Sq2(zp[2]))
                zp = zp / dz

                xyz_vector0 = np.vstack([xp, yp, zp])
                # print('xp = ', xp)
                # print('xyz_vector0 = \n', xyz_vector0)

                xyz_vector = xyz_vector0.T
                # print('xyz_vector = \n', xyz_vector)
                eTt = np.c_[xyz_vector, grindoint[PointNo]]

                e = [[0, 0, 0, 1]]
                eTt = np.r_[eTt, e]
                tTe = inv(eTt)

                if len(layerline) == 3:
                    if layerline_no == 1:           # 一條軌跡
                        SaveFile('grindcinc{}/eTt{}_{}_{}_{}.xyz'.format(protrusion, protrusion, Bfile0_no, layerline_no + 1, (len(grindoint) - 1) - PointNo), eTt)
                    else:
                        SaveFile('grindcinc{}/eTt{}_{}_{}_{}.xyz'.format(protrusion, protrusion, Bfile0_no, layerline_no + 1, PointNo), eTt)
                else:                               # 三條軌跡
                    if (Bfile0_no + 1) % 2 == 0:
                        SaveFile('grindcinc{}/eTt{}_{}_{}_{}.xyz'.format(protrusion, protrusion, Bfile0_no, 2, (len(grindoint) - 1) - PointNo), eTt)
                    else:
                        SaveFile('grindcinc{}/eTt{}_{}_{}_{}.xyz'.format(protrusion, protrusion, Bfile0_no, 2, PointNo), eTt)

                xb = grindoint[PointNo] + xp
                yb = grindoint[PointNo] + yp
                zb = grindoint[PointNo] + zp

                alltrajcoor.append(xb.tolist())
                alltrajcoor.append(yb.tolist())
                alltrajcoor.append(zb.tolist())
                alltrajcoor.append(grindoint[PointNo].tolist())
                SaveFile('inc_coor/vec{}_{}_{}_{}.xyz'.format(protrusion, Bfile0_no, layerline_no, PointNo), alltrajcoor)
    #
    SaveFile('inc_coor/alltrajcoor{}.xyz'.format(protrusion), alltrajcoor)

    # 接收研磨點修正距離
    revise = open(reviseT_path, 'r')
    line = revise.readlines()
    revise_txt = []
    for x in range(0, len(line)):
        revise_txt = line[x].split(' ', 1)
    print('研磨點誤差', revise_txt)

    protruison_no = int(revise_txt[0])
    revise_dis = float(revise_txt[1])

    time.sleep(0.1)
    print("protrusion = ", protruison_no)
    print("revise distance = ", revise_dis)

    # 重新計算補償誤差後的研磨軌跡結束
    T, newT = single_pro_euler.inc_euler(protruison_no, revise_dis)
    end_time = time.time()

    print('[Duration:', int((end_time - start_time) / 60), 'minutes', int((end_time - start_time) % 60), 'seconds]')
    print('\n---------- PROGRAM END ----------\n')
