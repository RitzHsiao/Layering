import math
import scipy.linalg
import numpy as np
import open3d as o3d
import os as os

# import transformat as trans


import glob
import shutil
import re

def ReadXyzFile(filename):
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

def SaveFile(Pcd_File_Name, PCDList):

    np.savetxt('{}'.format(Pcd_File_Name), PCDList, delimiter=' ')
    print('Saved File: [{}].'.format(Pcd_File_Name))

def Sq2(value):

    return value*value


a = sorted(glob.glob(os.path.join("each layer/", "*.xyz")), key=lambda x: (int(re.split('Cluster_|_|.xyz', x)[1]),
                                                                           int(re.split('Cluster_|_|.xyz', x)[2])))
print(a)

protrusion_numaber = int(re.split('_|.xyz', a[len(a) - 1])[1]) + 1
print('protrusion_numaber = ', protrusion_numaber)

shutil.rmtree('c:\\Users\\User\\Desktop\\陳昱廷\\Layering\\Output File')
os.makedirs('c:\\Users\\User\\Desktop\\陳昱廷\\Layering\\Output File')
shutil.rmtree('c:\\Users\\User\\Desktop\\陳昱廷\\Layering\\PointOne')
os.makedirs('c:\\Users\\User\\Desktop\\陳昱廷\\Layering\\PointOne')

for protruison_no in range(0, protrusion_numaber):

    print('protruison_no = ', protruison_no)
    ba = sorted(glob.glob(os.path.join("Process/", "*layer_after{}*".format(protruison_no))), key=lambda x: (int(re.split('layer_after|_|.xyz', x)[2])))
    # sa = sorted(glob.glob(os.path.join("twotraj/", "*twodtraj{}*".format(protruison_no))), key=os.path.getmtime)

    print('filename = ', ba)
    for Bfile0_no in range(0, len(ba)):
        # -----------------------------------------------------------------------------------------------------------------

        sa = sorted(glob.glob(os.path.join("twotraj/", "*twodtraj{}_{}_*".format(protruison_no, Bfile0_no))), key=lambda x: (int(re.split('twodtraj|_|.xyz', x)[2]),
                                                                                                                             int(re.split('twodtraj|_|.xyz', x)[3])))
        print('\nsa =', sa)

        for layerline_no in range(0, len(sa)):
            print('line = ', layerline_no)

            Bfile = ReadXyzFile(ba[Bfile0_no])
            Sfile = ReadXyzFile(sa[layerline_no])

            data = np.asarray(Bfile)
            source = np.asarray(Sfile)

            X = []
            Y = []
            XX = []
            YY = []

            for i in range(0, len(source)):
                X.append(source[i][0])
                Y.append(source[i][1])

            XX = np.asarray(X)
            YY = np.asarray(Y)

            A = np.c_[np.ones(data.shape[0]), data[:, :2], np.prod(data[:, :2], axis=1), data[:, :2] ** 2, data[:, :2] ** 3]
            C, _, _, _ = scipy.linalg.lstsq(A, data[:, 2])

            Z = np.dot(np.c_[np.ones(XX.shape), XX, YY, XX * YY, XX ** 2, YY ** 2, XX ** 3, YY ** 3], C)
            ZZ = Z.flatten()

            Grid = []
            for pointNo in range(0, len(XX)):
                Grid.append([XX[pointNo], YY[pointNo], ZZ[pointNo]])

            SaveFile('Process/grid{}_{}_{}.xyz'.format(protruison_no, Bfile0_no, layerline_no + 1), Grid)

            tra_inv = np.genfromtxt("transform/tra_inv_{}.xyz".format(protruison_no), dtype=None, comments='#',
                                    delimiter=' ')
            traj_before = o3d.io.read_point_cloud('Process/grid{}_{}_{}.xyz'.format(protruison_no, Bfile0_no, layerline_no + 1))
            traj_after = traj_before.transform(tra_inv)

            ntraj_after = np.asarray(traj_after.points)
            newtraj_after = ntraj_after.tolist()

            PointOne = []
            if newtraj_after[0][2] < newtraj_after[len(newtraj_after) - 1][2]:
                newtraj_after.reverse()
                PointOne = [newtraj_after[2]] #存下編號2的點
                SaveFile('PointOne/PointOne{}_{}_{}.xyz'.format(protruison_no, Bfile0_no, layerline_no + 1), PointOne)
                SaveFile('Output File/traj{}_{}_{}.xyz'.format(protruison_no, Bfile0_no, layerline_no + 1), newtraj_after)
            else:
                PointOne = [newtraj_after[2]] #存下編號2的點
                SaveFile('PointOne/PointOne{}_{}_{}.xyz'.format(protruison_no, Bfile0_no, layerline_no + 1), PointOne)
                SaveFile('Output File/traj{}_{}_{}.xyz'.format(protruison_no, Bfile0_no, layerline_no + 1), newtraj_after)


