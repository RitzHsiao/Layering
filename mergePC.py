import os as os
import open3d as o3d
import glob
import re
import numpy as np


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
print('\n=========== PROGRAM START ===========\n')
ScanFile = sorted(glob.glob(os.path.join("ScanFile_merge/", "ScanResult*")), key=lambda x: (int(re.split('ScanResult_|.xyz', x)[1])))
topscan = sorted(glob.glob(os.path.join("ScanFile_merge/", "topscan*")), key=lambda x: (int(re.split('topscan|.xyz', x)[1])))
print('ScanFile = ', ScanFile)
print('Topscan = ', topscan)
print('\n------ Processing top point cloud------\n')
protrusion_number = int(re.split('|_|_|.xyz', ScanFile[len(ScanFile)-1])[29])
number = int(re.split('|_|_|.xyz', ScanFile[len(ScanFile)-1])[32])
top_number = int(re.split('|.xyz', topscan[len(topscan)-1])[23])
# print('protrusion_number = ', protrusion_number)
# print('number = ', number)
# print('topnumber = ', top_number)

topscan_merge = o3d.geometry.PointCloud()
for top_no in range(0, top_number):
    topdata = o3d.io.read_point_cloud(topscan[top_no])
    # print('topscan[top_no] = ', topscan[top_no])
    topscan_merge += topdata
o3d.io.write_point_cloud("ScanFile/topscan.xyz", topscan_merge)
print('Saved File: [ScanFile/topscan.xyz]')
print('\n------ Processing side point cloud------\n')

for protrusion_no in range(1, protrusion_number+1):
    scan_times = sorted(glob.glob(os.path.join("ScanFile_merge/", "*ScanResult_{}*".format(protrusion_no))),key=lambda x: (int(re.split('ScanResult_|_|.xyz', x)[2])))
    # print('scan_times = ', scan_times)
    protrusion_merge = o3d.geometry.PointCloud()
    for scan_times_no in range(0, number):
        protrusiondata = o3d.io.read_point_cloud(scan_times[scan_times_no])
        # print('ScanFile[scan_times_no]', scan_times[scan_times_no])
        protrusion_merge += protrusiondata
    o3d.io.write_point_cloud("ScanFile/ScanResult_{}.xyz".format(protrusion_no), protrusion_merge)
    print('Saved File: [ScanFile/Scanfile{}.xyz]'.format(protrusion_no))

print('\n=========== PROGRAM END ===========\n')
