import numpy as np
import os as os
import glob
import shutil
import re
from sklearn.neighbors import KDTree

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
def SampleThePoint(MainPoint, tree, K_number = 5, KNNRadius = 1.5):

    distances, indices = tree.query(MainPoint, K_number)

    SampledPoints = []
    for i in range(0, K_number):
        if distances[0][i] <= KNNRadius:
            SampledPoints.append(Pcd[int(indices[0][i])])

    # print('SampledPoints:', SampledPoints)
    return SampledPoints, distances
#-------------Main-------------
shutil.rmtree('c:\\Users\\User\\Desktop\\陳昱廷\\Layering\\ReduceTrajectory')
os.makedirs('c:\\Users\\User\\Desktop\\陳昱廷\\Layering\\ReduceTrajectory')

KNNRadius = 2.5
K_number = 1
PointList = []
PointOne_Pronum = sorted(glob.glob(os.path.join("PointOne/", "*.xyz")), key=lambda x: (int(re.split('PointOne|_|_|.xyz', x)[4])))

print(PointOne_Pronum)
protrusion_number = int(re.split('|_|_|.xyz', PointOne_Pronum[len(PointOne_Pronum)-1])[18]) + 1
print('凸點數量 = ', protrusion_number)
print('=======================================================================================================================================================')
# # print('Layer_number = ', Layer_number)
# # trajectory_number = int(re.split('|_.xyz', a[len(a) - 1])move_x[3]) + 1
# # print('trajectory_number = ', trajectory_number)
count = 0
for protruison_no in range(0, protrusion_number):
    print('=======================================================================================================================================================')
    tra_Laynum = sorted(glob.glob(os.path.join("PointOne/", "*PointOne{}*".format(protruison_no))), key=lambda x: (int(re.split('PointOne|_|_|.xyz', x)[4])))
    # print(tra_Laynum)
    out_file = sorted(glob.glob(os.path.join("Output File/", "*traj{}*".format(protruison_no))), key=lambda x: (int(re.split('traj|_|_|.xyz', x)[1])))
    #print(out_file)
    Lay_file = sorted(glob.glob(os.path.join("Layer/", "*Layer{}*".format(protruison_no))), key=lambda x: (int(re.split('Layer|_|.xyz', x)[2])))
    # print(Lay_file)
    print('第 {} 號凸點, 共有 {} 層'.format(protruison_no, len(Lay_file)))
    # Layer_number = int(re.split('PointOne|_|_|.xyz', tra_Laynum[len(tra_Laynum) - 1])[2])
    # print(Layer_number)
    print('=========================================================================')
    for Layer_no in range(0, len(Lay_file)):
        print('Layer_no = ',  Layer_no)
        tra_tranum = sorted(glob.glob(os.path.join("PointOne/", "*PointOne{}_{}*".format(protruison_no, Layer_no))), key=lambda x: (int(re.split('PointOne|_|_|.xyz', x)[4])))
        print(tra_tranum)
        trajectory_number = int(re.split('PointOne|_|_|.xyz', tra_tranum[len(tra_tranum) - 1])[4])
        print('第 {} 層, 共有 {} 條軌跡'.format(Layer_no, trajectory_number))
        Layfile = ReadXyzFile(Lay_file[len(Lay_file) - Layer_no - 1])
        # print('Layfile', len(Lay_file))
        PointList = Layfile
        for trajectory_no in range(0, trajectory_number):
            # print('trajectory_no = ', trajectory_no)
            trafile = ReadXyzFile(tra_tranum[trajectory_no])
            MainPoint = trafile
            Pcd = PointList
            tree = KDTree(Pcd, leaf_size=40)
            SampledPoints, distances = SampleThePoint(MainPoint, tree, K_number=K_number, KNNRadius=KNNRadius)
            print('distances =', distances)
            if distances[0][0] < 1:
                pointlayer = Layer_no
                print('pointlayer =', pointlayer)
                print('有點')
                break
            else:
                print('沒點')
                print('-----------------------------------------------------------------------')
        else:
            continue
        # i = 0
        for pointlayer_no in range(pointlayer, len(Lay_file) + 6): # "+6"是因為最後有3層是三條軌跡 總共多了6個軌跡檔案
            tra_data = ReadXyzFile(out_file[pointlayer_no])
            tra_data_number2 = int(re.split('traj|_|_|.xyz', out_file[pointlayer_no])[2])
            tra_data_number3 = int(re.split('traj|_|_|.xyz', out_file[pointlayer_no])[3])
            if tra_data_number3 == 1:
                SaveFile('ReduceTrajectory/traj{}_{}_{}.xyz'.format(protruison_no, tra_data_number2-pointlayer, trajectory_no + 1), tra_data)
                print('\n')
                # i += 1
            elif tra_data_number3 == 2:
                SaveFile('ReduceTrajectory/traj{}_{}_{}.xyz'.format(protruison_no, tra_data_number2-pointlayer, trajectory_no + 2), tra_data)
                print('\n')
            elif tra_data_number3 == 3:
                SaveFile('ReduceTrajectory/traj{}_{}_{}.xyz'.format(protruison_no, tra_data_number2-pointlayer, trajectory_no + 3), tra_data)
                print('\n')
        break

