import os as os
import shutil as shutil
import open3d as o3d
import numpy as np
import math
import transformat as trans
import copy
import glob
from numpy.linalg import inv
import time

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

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
        RawData = lines[x].strip().split() #[x y z] from File
        PointList.append([float(RawData[0]), float(RawData[1]), float(RawData[2])])

    return PointList


def SaveFile(Pcd_File_Name, PCDList):

    np.savetxt('{}'.format(Pcd_File_Name), PCDList, delimiter=' ')
    print('Saved File: [{}].'.format(Pcd_File_Name))

shutil.rmtree('c:\\Users\\User\\Desktop\\陳昱廷\\Layering\\test')
os.makedirs('c:\\Users\\User\\Desktop\\陳昱廷\\Layering\\test')

# 手臂校正取點
oripoint = [0, 0, 0]
Uni = 1
Uox = [0 + Uni, 0, 0]
Uoy = [0, 0 + Uni, 0]
Uoz = [0, 0, 0 + Uni]
Oxyz = np.vstack([oripoint, Uox, Uoy, Uoz])
Oxyz = Oxyz.tolist()

# 校正塊
# chosepointo = [40.07, 40.07, 280.13]    #244.1 + 36.1
# chosepointx = [40.07, -40.07, 280.13]
# chosepointy = [-40.07, 40.07, 280.13]
# chosepointo = [40.015, 40.015, 280.13]    #244.1 + 36.1
# chosepointx = [40.015, -40.015, 280.13]
# chosepointy = [-40.015, 40.015, 280.13]
chosepointo = [40.000, 40.000, 280.00]
chosepointx = [40.000, -40.000, 280.00]
chosepointy = [-40.000, 40.000, 280.00]
vx = np.subtract(chosepointx, chosepointo)
vy = np.subtract(chosepointy, chosepointo)
vz = np.cross(vx, vy)

dvx = math.sqrt(Sq2(vx[0]) + Sq2(vx[1]) + Sq2(vx[2]))
vx = vx / dvx
dvy = math.sqrt(Sq2(vy[0]) + Sq2(vy[1]) + Sq2(vy[2]))
vy = vy / dvy
dvz = math.sqrt(Sq2(vz[0]) + Sq2(vz[1]) + Sq2(vz[2]))
vz = vz / dvz

choose_vector0 = np.vstack([vx, vy, vz])
# print('4654645498449849498494984', choose_vector0)
choose_vector = choose_vector0.T
eTcos = np.c_[choose_vector, chosepointo]
b = [[0, 0, 0, 1]]
eTcos = np.r_[eTcos, b]
print(eTcos)
# SaveFile('test/eTcos.xyz', eTcos)

# 掃描校正取點
filename = 'ScanResult_01.xyz'
basepoint = ReadXyzFile('{}'.format(filename))
cala = np.asarray(basepoint)

source = o3d.geometry.PointCloud()
source.points = o3d.utility.Vector3dVector(cala)
source_no = trans.demo_manual_registration(source)

# print('原點 = ', basepoint[source_no[0]][0], basepoint[source_no[0]][1], basepoint[source_no[0]][2])
# print('X = ', basepoint[source_no[1]][0], basepoint[source_no[1]][1], basepoint[source_no[1]][2])
# print('Y = ', basepoint[source_no[2]][0], basepoint[source_no[2]][1], basepoint[source_no[2]][2])


# # 校正姿態

# 取兩邊 最大值
# top o
if basepoint[source_no[0]][0] < basepoint[source_no[1]][0]:
    topo_x = basepoint[source_no[0]][0]
else:
    topo_x = basepoint[source_no[1]][0]

if basepoint[source_no[0]][1] > basepoint[source_no[1]][1]:
    topo_y = basepoint[source_no[0]][1]
else:
    topo_y = basepoint[source_no[1]][1]
topCchoseo = [topo_x, topo_y, (basepoint[source_no[0]][2] + basepoint[source_no[1]][2]) / 2]

# top x
if basepoint[source_no[2]][0] > basepoint[source_no[3]][0]:
    topx_x = basepoint[source_no[2]][0]
else:
    topx_x = basepoint[source_no[3]][0]

if basepoint[source_no[2]][1] > basepoint[source_no[3]][1]:
    topx_y = basepoint[source_no[2]][1]
else:
    topx_y = basepoint[source_no[3]][1]
topCchosex = [topx_x, topx_y, (basepoint[source_no[2]][2] + basepoint[source_no[3]][2]) / 2]

# top y
if basepoint[source_no[4]][0] < basepoint[source_no[5]][0]:
    topy_x = basepoint[source_no[4]][0]
else:
    topy_x = basepoint[source_no[5]][0]

if basepoint[source_no[4]][1] < basepoint[source_no[5]][1]:
    topy_y = basepoint[source_no[4]][1]
else:
    topy_y = basepoint[source_no[5]][1]
topCchosey = [topy_x, topy_y, (basepoint[source_no[4]][2] + basepoint[source_no[5]][2]) / 2]

top_oxy = topCchoseo + topCchosex + topCchosey
top_oxy = np.asarray(top_oxy).reshape(3, 3)
SaveFile('test/top_oxy.xyz', top_oxy)

xp = np.subtract(topCchosex, topCchoseo)
yp = np.subtract(topCchosey, topCchoseo)

# 取大概角點-------------------------------------------------------------------------------------
# topCchoseo = [basepoint[source_no[0]][0], basepoint[source_no[0]][1], basepoint[source_no[0]][2]]   # 用校正塊補償  glofmodel

# 校正塊座標
# xp = [basepoint[source_no[1]][0] - basepoint[source_no[0]][0],
#       basepoint[source_no[1]][1] - basepoint[source_no[0]][1],
#       basepoint[source_no[1]][2] - basepoint[source_no[0]][2]]
#
# yp = [basepoint[source_no[2]][0] - basepoint[source_no[0]][0],
#       basepoint[source_no[2]][1] - basepoint[source_no[0]][1],
#       basepoint[source_no[2]][2] - basepoint[source_no[0]][2]]
#
# xp = np.array(xp)
# yp = np.array(yp)
#----------------------------------------------------------------------------
xp = np.array(xp)
yp = np.array(yp)
zp = np.cross(xp, yp)

dxp = math.sqrt(Sq2(xp[0]) + Sq2(xp[1]) + Sq2(xp[2]))
xp = xp / dxp
dyp = math.sqrt(Sq2(yp[0]) + Sq2(yp[1]) + Sq2(yp[2]))
yp = yp / dyp
dzp = math.sqrt(Sq2(zp[0]) + Sq2(zp[1]) + Sq2(zp[2]))
zp = zp / dzp

Cpvector0 = np.vstack([xp, yp, zp])
Cpvector = Cpvector0.T
CTp = np.c_[Cpvector, topCchoseo]
b = [[0, 0, 0, 1]]
CTcosp = np.r_[CTp, b]
# print('CTp = ', CTcosp)
# SaveFile('test/CTcosp.xyz', CTcosp)

cospTC = np.linalg.inv(CTcosp)
# SaveFile('test/cosTC.xyz', cospTC)

# 原點轉移到選點
# pointbefore = o3d.io.read_point_cloud('ScanResult_01.xyz')
# pointafter = pointbefore.transform(cospTC)
# o3d.io.write_point_cloud("test/basepoint_choose.xyz", pointafter)

# 相機相對於法蘭
eTc = np.dot(eTcos, cospTC)
SaveFile('test/top_eTc.xyz', eTc)

# 原點轉移到法蘭
pointbefore = o3d.io.read_point_cloud('ScanResult_01.xyz')
pointafter = pointbefore.transform(eTc)
o3d.io.write_point_cloud("test/basepoint_frame.xyz", pointafter)

# 原點轉到法蘭
scanresult = o3d.io.read_point_cloud("ScanFile/topscan.xyz")
transresult = scanresult.transform(eTc)
o3d.io.write_point_cloud("test/topframeresult.xyz", transresult)
