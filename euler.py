import numpy as np
import math
import glob
import os as os

import time
import shutil
import re
# import numba as nb


# @nb.jit
def SaveFile(Pcd_File_Name, PCDList):
    np.savetxt('{}'.format(Pcd_File_Name), PCDList, delimiter=' ')
    print('Saved File: [{}].'.format(Pcd_File_Name))

def Sq2(value):
    # Code untuk Kuadrat bilangan
    return value*value


a = sorted(glob.glob(os.path.join("each layer/", "*.xyz")), key=lambda x: (int(re.split('Cluste_|_|.xyz', x)[1]),
                                                                           int(re.split('Cluste_|_|.xyz', x)[2])))
print(a)

protrusion_numaber = int(re.split('_|.xyz', a[len(a) - 1])[1]) + 1
shutil.rmtree('c:\\Users\\User\\Desktop\\陳昱廷\\Layering\\xyzRxyz')
os.makedirs('c:\\Users\\User\\Desktop\\陳昱廷\\Layering\\xyzRxyz')

t1 = np.array([[-9.2061e+01, 1.2949e+03, 2.6117e+01]])   # 研磨點 原始點位 中間 後退0.15mm  model8長方形凸點
t2 = np.array([[-3.9082e+02, 1.2949e+03, 2.6117e+01]])     # 研磨點 原始點位 中間 後退0.15mm  model8長方形凸點
vt = t2 - t1
dvtz = math.sqrt(Sq2(vt[0][0]) + Sq2(vt[0][1]) + Sq2(vt[0][2]))
print('d = ', dvtz)
# input()
for protruison_no in range(0, protrusion_numaber):
    print('\nprotrusion_numaber =', protruison_no)

    Afile0 = sorted(glob.glob(os.path.join("rTe_matric{}/".format(protruison_no), "*.xyz")), key=os.path.getmtime)
    XYZfile0 = sorted(glob.glob(os.path.join("rTe_matric{}/".format(protruison_no), "*.xyz")), key=os.path.getmtime)
    AllxyzRxyz0 = []
    no = 1
    print('point num =', len(Afile0))
    for Afile0_no in range(0, len(Afile0)):  # len(Afile0)

        # file = open('rTe_matric/rTe0_0.xyz')
        file = open(Afile0[Afile0_no], "r")
        # print('------------------------------------------',Afile0[Afile0_no])

        lines = file.readlines()
        # print('No of Points [XYZ]:', len(lines))
        transf = []
        for x in range(0, len(lines)):
            RawData = lines[x].strip().split(" ")  # [x y z] from File
            transf.append([float(RawData[0]), float(RawData[1]), float(RawData[2]), float(RawData[3])])

        rTe = np.asarray(transf)
        # print('rTe =\n', rTe)

        # HIWIN 機械手臂旋轉 ZYX
        R = rTe
        if R[2, 0] < 1:
            if R[2, 0] > -1:
                thetaY = math.asin(-R[2, 0])
                thetaZ = math.atan2(R[1, 0], R[0, 0])
                thetaX = math.atan2(R[2, 1], R[2, 2])
            else:
                thetaY = math.pi / 2
                thetaZ = -math.atan2(-R[1, 2], R[1, 1])
                thetaX = 0
        else:
            thetaY = -(math.pi / 2)
            thetaZ = math.atan2(-R[1, 2], R[1, 1])
            thetaX = 0

        thetaX = thetaX * 180 / math.pi
        thetaY = thetaY * 180 / math.pi
        thetaZ = thetaZ * 180 / math.pi
        # print('euler angle = ', thetaX * 180 / math.pi, thetaY * 180 / math.pi, thetaZ * 180 / math.pi)
        # print(thetaX, thetaY, thetaZ)
        Rxyz = [thetaX, thetaY, thetaZ]
        print('Rxyz =', Rxyz)

        xyzfile = open(XYZfile0[Afile0_no], "r")
        lines = xyzfile.readlines()
        # print('No of Points [XYZ]:', len(lines))
        transf = []
        for x in range(0, len(lines)):
            RawData = lines[x].strip().split(" ")
            transf.append([float(RawData[0]), float(RawData[1]), float(RawData[2]), float(RawData[3])])

        rTexyz = np.asarray(transf)
        print('xyzRxyz =\n', rTexyz)

        xyz = np.array(rTexyz[0:3, 3] / 1000)  # 軟體需要，毫米單位轉公尺

        xyzRxyz = np.append(xyz, Rxyz)

        # SaveFile('xyzRxyz0/xyzRxyz0_{}.xyz'.format(Afile0_no), xyzRxyz)

        allxyzRxyz = xyzRxyz.reshape(1, -1)
        allxyzRxyz = allxyzRxyz.flatten()
        allxyzRxyz = allxyzRxyz.tolist()
        AllxyzRxyz0.append(allxyzRxyz)

        time.sleep(0.01)

    SaveFile('D:\\euler\\euler_angle\\xyzRxyz\\AllxyzRxyz{}.xyz'.format(protruison_no), AllxyzRxyz0)






