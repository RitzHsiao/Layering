import open3d as o3d
import numpy as np
import glob
import os

def hausdorff_distance(pcd1, pcd2):
    distances = pcd1.compute_point_cloud_distance(pcd2)
    hausdorff_dist = np.max(distances)
    return hausdorff_dist

def average_distance(pcd1, pcd2):
    distances = pcd1.compute_point_cloud_distance(pcd2)
    avg_dist = np.mean(distances)
    return avg_dist

def quantile_distance(pcd1, pcd2, quantile=0.85):
    distances = pcd1.compute_point_cloud_distance(pcd2)
    q_dist = np.quantile(distances, quantile)
    return q_dist



def calculate_grinding_percentage(before, expected, actual):
    dist_before = quantile_distance(bump_before, bump_expected)
    dist_after = quantile_distance(bump_before, bump_actual)
    print(dist_before)
    print(dist_after)
    if dist_after < 0.5:
        dist_after = 0
    else:
        dist_after = dist_after
    grinding_percentage = (dist_before - dist_after) / dist_before * 100
    if grinding_percentage <= 0:
        grinding_percentage = grinding_percentage
    else:
        grinding_percentage = 100 - grinding_percentage
    # grinding_percentage = dist_after * 10
    #
    # if grinding_percentage >= 100:
    #     grinding_percentage = 100 - grinding_percentage
    # else:
    #     grinding_percentage = grinding_percentage

    return grinding_percentage, dist_before, dist_after


# 讀點雲
pcd1 = o3d.io.read_point_cloud("compare/pcd1.xyz")  # 待研磨點雲
pcd2 = o3d.io.read_point_cloud("compare/pcd2.xyz")  # 預計研磨完
pcd3 = o3d.io.read_point_cloud("compare/pcd3.xyz")  # 實際研磨完

# Perform registration
threshold = 1
trans_init = np.identity(4)

reg_p2p = o3d.pipelines.registration.registration_icp(
    pcd3, pcd2, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20000))

trans = reg_p2p.transformation

pcd3 = pcd3.transform(trans)
pcd2 = pcd2
o3d.io.write_point_cloud("compare/pcd2_transformed.xyz", pcd2)
o3d.io.write_point_cloud("compare/pcd3_transformed.xyz", pcd3)

# o3d.visualization.draw_geometries([pcd3])
bumps_before_file = sorted(glob.glob(os.path.join("compare/", "rawCluster*")))  # 需要填入研磨前的凸点点云列表
bumps_expected_file = sorted(glob.glob(os.path.join("compare/", "refCluster*")))  # 需要填入预计研磨完成后的凸点点云列表

grinding_percentages = []
distances_before = []
distances_after = []
point_indices = []
# 新建一個點雲對象，用於儲存有色點雲
colored_cloud = o3d.geometry.PointCloud()
for part_no in range(0, len(bumps_before_file)):
    bumps_before_name = bumps_before_file[part_no]
    print('bumps_before : ', bumps_before_name)
    bumps_expected_name = bumps_expected_file[part_no]
    print('bumps_expected : ', bumps_expected_name)

    bump_before = o3d.io.read_point_cloud(bumps_before_name)
    bump_expected = o3d.io.read_point_cloud(bumps_expected_name)

    # 创建边界框
    obb = bump_before.get_oriented_bounding_box()
    # 设置边界框颜色为红色
    obb.color = (1, 0, 0)  # RGB颜色，范围是[0,1]

    # 将 OBB 转换为 Axis-Aligned Bounding Box (AABB) 并裁剪 pcd31er3456cheese
    aabb = obb.get_axis_aligned_bounding_box()
    bump_actual = pcd3.crop(aabb)

    # 显示边界框和点云
    # o3d.visualization.draw_geometries([bump_actual, bump_before, obb])
    # o3d.visualization.draw_geometries([pcd3, obb])
    # 计算研磨百分比
    grinding_percentage, dist_before, dist_after = calculate_grinding_percentage(
        bump_before, bump_expected, bump_actual)
    # 获取 bump_actual 中的点在 pcd3 中的索引
    bump_actual_tree = o3d.geometry.KDTreeFlann(bump_actual)
    pcd3_tree = o3d.geometry.KDTreeFlann(pcd3)
    indices = []

    for point in bump_actual.points:
        [_, idx, _] = pcd3_tree.search_knn_vector_3d(point, 1)
        indices.append(idx[0])

    for idx, point in zip(indices, bump_actual.points):
        if grinding_percentage < 0:
            colored_cloud.points.append(point)
            colored_cloud.colors.append([1, 0, 0])  # 過磨，紅色
            point_indices.append(idx)
        else:
            colored_cloud.points.append(point)
            colored_cloud.colors.append([0, 0, 1])  # 少磨，藍色
            point_indices.append(idx)

    grinding_percentages.append(grinding_percentage)
    distances_before.append(dist_before)
    distances_after.append(dist_after)


for i, grinding_percentage in enumerate(grinding_percentages):
    print(f"凸点 {i} 的研磨程度: {grinding_percentage:.2f}%")

# # 找出不重疊的點雲
distances = colored_cloud.compute_point_cloud_distance(pcd2)
distances = np.asarray(distances)
print(distances)
threshold = 0.45
non_overlap_idx = distances > threshold

# # 删除重叠的点云
pcd = colored_cloud.select_by_index(np.where(non_overlap_idx == True)[0])

# 顯示有色點雲
o3d.visualization.draw_geometries([pcd])