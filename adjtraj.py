import numpy as np
import os as os
import glob
import shutil
import re

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

point = []
# 開啟檔案並讀取內容
with open('inf_of_extruded.txt', 'r') as f:
    line = f.readline()
    protrusion, layer_no, traj_no, point_no = map(int, line.split())
print('將第{}層, 第{}個點後的軌跡點, 往回修正0.2mm '.format(layer_no, point_no+1))

adj_layerno = layer_no - 1
adj_traj_no = traj_no
adj_point_no = point_no + 1
print('凸點編號 = ', protrusion)
Layer_num = sorted(glob.glob(os.path.join("Output File/", "*traj{}*".format(protrusion))), key=lambda x: (int(re.split('traj|_|_|.xyz', x)[1])))
Layer_number = len(Layer_num)
Layer = Layer_number-6
print('{} 號凸點原本的層數 = '.format(protrusion), Layer)

Layer_file = sorted(glob.glob(os.path.join("Output File/", "*traj{}_{}*".format(protrusion, Layer-2))), key=lambda x: (int(re.split('traj|_|_|.xyz', x)[1])))  # 倒數第二層
tra_0 = [ReadXyzFile(file) for file in Layer_file]
# print('0-tra_0 = ', tra_0)
Layer_file = sorted(glob.glob(os.path.join("Output File/", "*traj{}_{}*".format(protrusion, Layer-1))), key=lambda x: (int(re.split('traj|_|_|.xyz', x)[1])))  # 最後一層
tra_1 = [ReadXyzFile(file) for file in Layer_file]
# print('0-tra_1 = ', tra_1)

if Layer - 3 > layer_no:
    adj_trajectory = (np.array(tra_0) - (np.array(tra_1) - np.array(tra_0))) / 2
    SaveFile('adjtraj/adjtra{}_{}_{}_{}.xyz'.format(protrusion, adj_traj_no, 0, point), adj_trajectory)
# for j in range(0, 3):
#     adj_trajectory = (np.array(tra_0[j]) - (np.array(tra_1[j]) - np.array(tra_0[j])))/2
#     SaveFile('adjtraj/adjtra{}_{}_{}_{}.xyz'.format(protrusion, adj_traj_no, j, point), adj_trajectory)


