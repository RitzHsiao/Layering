import File
import open3d as o3d

# BECAUSE IN LAYERING CODE, LAYERING ALGORITHM NEEDS NORMAL VECTORS FOR RAW MODEL, SO HERE WE JUST ADD NORMAL (0,0,0)
# FOR EACH POINT CLOUDS IN THE RAW MODEL FILE

print('\n---- Program Start ---------\n')

# file = "Source File/NewFile.xyz"
# opcd = o3d.io.read_point_cloud(file)
# pcd = opcd.voxel_down_sample(voxel_size=1.2)
# o3d.io.write_point_cloud("Source File/NewFile.xyz", pcd)

# Pcd = File.ReadXyzFile('Source File/RemovedPoints_9.xyz')
# for i in range(0, 5):
#     print('cluster_no = ', i)
#     Pcd = File.ReadXyzFile('cluster/rawCluster{}.xyz'.format(i))
#
#     for PcdNo in range(0, len(Pcd)):
#         Pcd[PcdNo].append(0)
#         Pcd[PcdNo].append(0)
#         Pcd[PcdNo].append(0)
#
#     # File.SaveFile('Source File/RemovedPoints_9', Pcd)
#     File.SaveFile('cluster/rawCluster{}'.format(i), Pcd)
#     print('this cluster have 0 normal vector\n')
#
# print('---- Program Finish ---------')


Pcd = File.ReadXyzFile('Source File/topframeresult.xyz')

for PcdNo in range(0, len(Pcd)):
    Pcd[PcdNo].append(0)
    Pcd[PcdNo].append(0)
    Pcd[PcdNo].append(0)

# File.SaveFile('Source File/RemovedPoints_9', Pcd)
File.SaveFile('Source File/topframeresult', Pcd)

print('\n---- Program Finish ---------')