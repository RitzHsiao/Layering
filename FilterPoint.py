import File
import open3d as o3d
import numpy as np

PointCloud = File.ReadXyzFile('CutFileSource/Layer 0.xyz')
Xmin = float(-99999)
Xmax = float(99999)
Ymin = float(-99999)
Ymax = float(99999)
Zmin = float(15)
Zmax = float(99999)


FilteredPoint = []
for point_no in range(0, len(PointCloud)):
    print(PointCloud[point_no][2])
    if Xmin <= PointCloud[point_no][0] <= Xmax and Ymin <= PointCloud[point_no][1] <= Ymax and Zmin <= PointCloud[point_no][2]<=Zmax:
        FilteredPoint.append(PointCloud[point_no])

File.SaveFile('CutFileSource/FilteredResult', FilteredPoint)

# # DOWNSAMPLE
# pcd = o3d.io.read_point_cloud('cutFileSource/FilteredResult.xyz')
# o3d.visualization.draw_geometries([pcd])
# pcd = o3d.geometry.PointCloud.voxel_down_sample(pcd, 1) #1
#
#
#
# # NORMAL
# o3d.geometry.PointCloud.estimate_normals(pcd,o3d.geometry.KDTreeSearchParamHybrid(radius=4,
#                                                           max_nn=300))
#
# o3d.geometry.PointCloud.orient_normals_consistent_tangent_plane(pcd, 15)
#
# o3d.visualization.draw_geometries([pcd])
#
# o3d.io.write_point_cloud('cutFileSource/DownsampledResult.xyzn',pcd, write_ascii=True)

# # MESH
# distances = pcd.compute_nearest_neighbor_distance()
# avg_dist = np.mean(distances)
# radius = 1.3 * avg_dist #1.5
#
# mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
#            pcd,
#            o3d.utility.DoubleVector([radius, radius * 2])) #2

# o3d.visualization.draw_geometries([mesh])
