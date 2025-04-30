import open3d as o3d

# 讀取點雲

pcd = o3d.io.read_point_cloud("\\Users\\User\\Desktop\\陳昱廷\\Layering\\Trajectory\\Trajp3_0_1.xyz")
pcd1 = o3d.io.read_point_cloud("\\Users\\User\\Desktop\\陳昱廷\\Layering\\Trajectory\\Trajp3_6_1.xyz")
pcd2 = o3d.io.read_point_cloud("\\Users\\User\\Desktop\\陳昱廷\\Layering\\Trajectory\\Trajp3_6_2.xyz")
pcd3 = o3d.io.read_point_cloud("\\Users\\User\\Desktop\\陳昱廷\\Layering\\Trajectory\\Trajp3_6_3.xyz")
# 估算法向量
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=10))
pcd1.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=10))
pcd2.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=10))
pcd3.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=10))
# # 可視化點雲，可視化時你應該能夠看到點雲的法向量
# o3d.visualization.draw_geometries([pcd], point_show_normal=True)

# # 可視化點雲
# o3d.visualization.draw_geometries([pcd])

# 使用Ball-Pivoting algorithm重建表面
radius = 0.5  # 選擇一個合適的半徑
bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
               pcd,
               o3d.utility.DoubleVector([radius, radius * 2]))
radius = 0.5  # 選擇一個合適的半徑
bpa_mesh1 = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
               pcd1,
               o3d.utility.DoubleVector([radius, radius * 2]))
radius = 0.5  # 選擇一個合適的半徑
bpa_mesh2 = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
               pcd2,
               o3d.utility.DoubleVector([radius, radius * 2]))
radius = 0.5  # 選擇一個合適的半徑
bpa_mesh3 = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
               pcd3,
               o3d.utility.DoubleVector([radius, radius * 2]))

# # 可視化網格
# o3d.visualization.draw_geometries([bpa_mesh])

# 計算表面積
surface_area = bpa_mesh.get_surface_area()
surface_area1 = bpa_mesh1.get_surface_area()
surface_area2 = bpa_mesh2.get_surface_area()
surface_area3 = bpa_mesh3.get_surface_area()

print(f'Surface Area(mm²): {surface_area}')
per_surface = surface_area/18
print(f'The grinding area for the first point is approximately(mm²): {per_surface}')
force_onepoint = 17.129
print(f'The average force measured by the force sensor at the first point(N): {force_onepoint}')
force_unit = force_onepoint/(per_surface)
print(f'Force per unit area:(N/mm²): {force_unit}')
force1 = surface_area1 * force_unit / 18
force2 = surface_area2 * force_unit / 18
force3 = surface_area3 * force_unit / 18

print(f'Estimated force value for the "first" trajectory: {force1}')
print(f'Estimated force value for the "second" trajectory: {force2}')
print(f'Estimated force value for the "third" trajectory: {force3}')