import os as os
import open3d as o3d
import numpy as np
import math

import glob
from numpy.linalg import inv

import time
import shutil
import re

def search_radius_vector_3d(pcd, pcd_tree, k, i, dis):

    pcd.colors[i] = [0, 0, 1] # 查詢點

    [k, idx, _] = pcd_tree.search_radius_vector_3d(pcd.points[i], dis)
    np.asarray(pcd.colors)[idx[1:], :] = [0, 1, 0]

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


def SaveFile(Pcd_File_Name, PCDList):

    np.savetxt('{}'.format(Pcd_File_Name), PCDList, delimiter=' ')
    print('Saved File: [{}].'.format(Pcd_File_Name))

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

def ReadXyzFile(filename):
    print('File Path:', filename)
    f = open(filename, "r")
    lines = f.readlines()
    # print('No of Points [XYZ]:', len(lines))
    PointList = []

    for x in range(0, len(lines)):
        RawData = lines[x].strip().split() #[x y z] from File
        PointList.append([float(RawData[0]), float(RawData[1]), float(RawData[2])])

    return PointList

def Sq2(value):
    # Code untuk Kuadrat bilangan
    return value*value

shutil.rmtree('c:\\Users\\User\\Desktop\\陳昱廷\\Layering\\coor')
os.makedirs('c:\\Users\\User\\Desktop\\陳昱廷\\Layering\\coor')

a = sorted(glob.glob(os.path.join("each layer/", "*.xyz")), key=lambda x: (int(re.split('Cluste_|_|.xyz', x)[1]),
                                                                           int(re.split('Cluste_|_|.xyz', x)[2])))
print(a)

protrusion_numaber = int(re.split('_|.xyz', a[len(a) - 1])[1]) + 1
print('protrusion_numaber = ', protrusion_numaber)

print('補償校正誤差位移 = ')
move_x = input('x 方向 ')
move_y = input('y 方向 ')

for protruison_no in range(0, protrusion_numaber):
    alltrajcoor = []
    print('\nprotruison_no = ', protruison_no)

    sa0 = sorted(glob.glob(os.path.join("Output File/", "*traj{}*".format(protruison_no))), key=lambda x: (int(re.split('traj|_|.xyz', x)[2]),
                                                                                                                             int(re.split('traj|_|.xyz', x)[3])))

    shutil.rmtree('c:\\Users\\User\\Desktop\\陳昱廷\\Layering\\grindcoor{}'.format(protruison_no))
    os.makedirs('c:\\Users\\User\\Desktop\\陳昱廷\\Layering\\grindcoor{}'.format(protruison_no))

    layer = len(glob.glob(os.path.join("each layer/", "Cluster_{}*".format(protruison_no))))
    print('該凸點總層數 = ', layer)

    linenor = []
    linenor_1 = []
    linenor_2 = []
    linenor_3 = []
    for linenor_no in range(1, 4):
        print('line = ', linenor_no)

        NOR = ReadXyzNorFile('cluster/refCluster{}.xyz'.format(protruison_no))
        TRpoint = ReadXyzFile('Output File/traj{}_{}_{}.xyz'.format(protruison_no, layer - 1, linenor_no))
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
                    print('NOR[i] = ', NOR[i] )
                    linenor.append([NOR[i][3], NOR[i][4], NOR[i][5]])

        # print('linenor = ', linenor)
        # print('linenor = ', len(linenor))

    linenor_1 = linenor[0:10]
    linenor_2 = linenor[10:20]
    linenor_3 = linenor[20:30]
    # print('linenor = ', len(linenor))
    # print('linenor_2 = ', linenor_2)
    # print('linenor_3 = ', linenor_3)
    # input()
    # print("next protrusion\n")

    for Bfile0_no in range(0, layer):

        # ---------------------------------------------------------------------------------------------------------
        print('\nlayer =', Bfile0_no)
        layerline = sorted(glob.glob(os.path.join("Output File/", "*traj{}_{}_*".format(protruison_no, Bfile0_no))), key=os.path.getmtime)

        for layerline_no in range(0, len(layerline)):

            print(len(layerline), '條軌跡------------------------------', layerline)
            print('layerline_no = ', layerline_no + 1)

            ## 坐標系修正 後面沒有拿來使用 西哩感逆
            TRFname = layerline[layerline_no]
            movematric = np.array([[1, 0, 0, move_x], [0, 1, 0, move_y], [0, 0, 1, 0], [0, 0, 0, 1]])
            # print(movematric)
            TRfile = o3d.io.read_point_cloud(TRFname)
            moveTR = TRfile.transform(movematric)
            o3d.io.write_point_cloud(TRFname, moveTR)
            ##

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
                        SaveFile('grindcoor{}/eTt{}_{}_{}_{}.xyz'.format(protruison_no, protruison_no, Bfile0_no, layerline_no + 1, (len(grindoint) - 1) - PointNo), eTt)
                    else:
                        SaveFile('grindcoor{}/eTt{}_{}_{}_{}.xyz'.format(protruison_no, protruison_no, Bfile0_no, layerline_no + 1, PointNo), eTt)
                else:                               # 三條軌跡
                    if (Bfile0_no + 1) % 2 == 0:
                        SaveFile('grindcoor{}/eTt{}_{}_{}_{}.xyz'.format(protruison_no, protruison_no, Bfile0_no, 2, (len(grindoint) - 1) - PointNo), eTt)
                    else:
                        SaveFile('grindcoor{}/eTt{}_{}_{}_{}.xyz'.format(protruison_no, protruison_no, Bfile0_no, 2, PointNo), eTt)

                xb = grindoint[PointNo] + xp
                yb = grindoint[PointNo] + yp
                zb = grindoint[PointNo] + zp
    #
    #             coor = np.vstack([xb, yb, zb, grindoint[PointNo]])
    #
    #             if len(layerline) == 3:
    #                 if layerline_no == 1:
    #                     SaveFile('coor/trajcoor{}_{}_{}_{}.xyz'.format(protruison_no, Bfile0_no, layerline_no + 1, (len(grindoint) - 1) - PointNo), coor)
    #                 else:
    #                     SaveFile('coor/trajcoor{}_{}_{}_{}.xyz'.format(protruison_no, Bfile0_no, layerline_no + 1, PointNo), coor)
    #             else:
    #                 if (Bfile0_no + 1) % 2 == 0:
    #                     SaveFile('coor/trajcoor{}_{}_{}_{}.xyz'.format(protruison_no, Bfile0_no, layerline_no + 1, (len(grindoint) - 1) - PointNo), coor)
    #                 else:
    #                     SaveFile('coor/trajcoor{}_{}_{}_{}.xyz'.format(protruison_no, Bfile0_no, layerline_no + 1, PointNo), coor)
    #
                alltrajcoor.append(xb.tolist())
                alltrajcoor.append(yb.tolist())
                alltrajcoor.append(zb.tolist())
                alltrajcoor.append(grindoint[PointNo].tolist())
                SaveFile('coor/vec{}_{}_{}_{}.xyz'.format(protruison_no, Bfile0_no, layerline_no, PointNo), alltrajcoor)
    #
    SaveFile('coor/alltrajcoor{}.xyz'.format(protruison_no), alltrajcoor)








