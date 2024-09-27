import open3d as o3d


# Import point cloud
pcd = o3d.io.read_point_cloud("pointcloud.pcd")
print(pcd)
o3d.visualization.draw_geometries([pcd])

# Downsample the point cloud
downpcd = pcd.voxel_down_sample(voxel_size=0.05) # Voxel size in m
print(downpcd)
o3d.visualization.draw_geometries([downpcd])

# Estimate normals
# Search radius for neighbors in m, and max no. neighbors
downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
o3d.visualization.draw_geometries([downpcd],point_show_normal=True)

# Planar patch detection
# using all defaults
oboxes = downpcd.detect_planar_patches(
    normal_variance_threshold_deg=60,
    coplanarity_deg=75,
    outlier_ratio=0.75,
    min_plane_edge_length=0,
    min_num_points=0,
    search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))
print("Detected {} patches".format(len(oboxes)))
geometries = []
for obox in oboxes:
    mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obox, scale=[1, 1, 0.0001])
    mesh.paint_uniform_color(obox.color)
    geometries.append(mesh)
    geometries.append(obox)
geometries.append(downpcd)
o3d.visualization.draw_geometries(geometries,
                                  zoom=0.62,
                                  front=[0.4361, -0.2632, -0.8605],
                                  lookat=[2.4947, 1.7728, 1.5541],
                                  up=[-0.1726, -0.9630, 0.2071])


# Alpha shapes surface reconstruction
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(downpcd, 0.2) # Larger alpha makes larger triangles
mesh.compute_vertex_normals()
o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

# Ball pivoting surface reconstruction
# radii = [1, 2, 5, 10]
# rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
#     downpcd, o3d.utility.DoubleVector(radii))
# o3d.visualization.draw_geometries([rec_mesh])

# Poisson surface reconstruction
with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        downpcd, depth=5)
print(mesh)
mesh.paint_uniform_color([1, 0, 0])
o3d.visualization.draw_geometries([mesh])


