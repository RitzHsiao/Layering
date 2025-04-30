import shutil as shutil
import open3d as o3d
import os
import numpy as np
import copy



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

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

shutil.rmtree('c:\\Users\\User\\Desktop\\陳昱廷\\Layering\\AfterGrinding')
os.makedirs('c:\\Users\\User\\Desktop\\陳昱廷\\Layering\\AfterGrinding')


# Load the two point cloud files
pcd1 = o3d.io.read_point_cloud("C:\\Users\\User\\Desktop\\陳昱廷\\Layering\\compare\\pcd3.xyz")
pcd2 = o3d.io.read_point_cloud("C:\\Users\\User\\Desktop\\陳昱廷\\Layering\\compare\\pcd2.xyz")

pcd1_down = pcd1
pcd2_down = pcd2
# Perform registration
threshold = 1
trans_init = np.identity(4)

reg_p2p = o3d.pipelines.registration.registration_icp(
    pcd1_down, pcd2_down, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20000))

trans = reg_p2p.transformation
# draw_registration_result(pcd1_down, pcd2_down, trans)
pcd1_transformed = pcd1_down.transform(trans)
pcd2_transformed = pcd2_down
o3d.io.write_point_cloud("compare/pcd1_transformed.xyz", pcd1_transformed)
o3d.io.write_point_cloud("compare/pcd2_transformed.xyz", pcd2_transformed)

pcd1 = pcd1_transformed
pcd2 = pcd2_transformed

# o3d.visualization.draw_geometries([pcd1, pcd2])

# # TEST1
# 計算 pcd1 到 pcd2 的距離
distances = pcd1.compute_point_cloud_distance(pcd2)
tree = o3d.geometry.KDTreeFlann(pcd2)
pcd1.paint_uniform_color([1, 1, 1])
pcd1_colors = np.asarray(pcd1.colors)
pcd1_points = np.asarray(pcd1.points)


for i in range(len(pcd1.points)):
    # 判斷 pcd1 的點是否在 pcd2 的內部或外部
    _, idx, _ = tree.search_knn_vector_3d(pcd1.points[i], 1)
    closest_point = np.asarray(pcd2.points)[idx[0], :]
    if np.linalg.norm(closest_point - pcd1.points[i]) > 0:
        if pcd1_points[i, 2] >= closest_point[2]:
            # 若 pcd1 的點在 pcd2 內部且在 pcd2 上方，則將點的顏色設定為紅色
            pcd1_colors[i] = [1, 0, 0]  # 紅色
        else:
            # 若 pcd1 的點在 pcd2 內部且在 pcd2 下方，則將點的顏色設定為藍色
            pcd1_colors[i] = [0, 0, 1]  # 藍色
    else:
        # 若 pcd1 的點在 pcd2 外部，則將點的顏色設定為紅色
        pcd1_colors[i] = [1, 0, 0] # 紅色
pcd1.colors = o3d.utility.Vector3dVector(pcd1_colors)
# # TEST1 END
# 將拼合後的點雲設為同一顏色
pcd = pcd1
distances = pcd.compute_point_cloud_distance(pcd2)
distances = np.asarray(distances)

o3d.visualization.draw_geometries([pcd])
# 找出不重疊的點雲
threshold = 0.55
non_overlap_idx = distances > threshold

# 刪除重疊的點雲
pcd = pcd.select_by_index(np.where(non_overlap_idx == True)[0])

# 顯示點雲
o3d.visualization.draw_geometries([pcd])
o3d.io.write_point_cloud("compare/compare.xyz", pcd)

