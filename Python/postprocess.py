import open3d as o3d
import numpy as np
import copy

def draw_registration_result(vis, source, target, result):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    #target_temp = target.voxel_down_sample(voxel_size=0.1) # Voxel size in m
    source_temp.paint_uniform_color([1, 0, 0]) # red
    target_temp.paint_uniform_color([0, 0, 1]) # blue

    # Paint included points green
    # 1st col of correspondence set are the indices of the hits
    hits = np.asarray(result.correspondence_set)[:,0]
    np.asarray(source_temp.colors)[hits,:] = np.tile([0, 1, 0],(hits.size,1))

    source_temp.transform(result.transformation)
    draw(vis,source_temp,target_temp)


def draw(vis,*args):

    vis.create_window(height=1928, width=3200)
    # Get the rendering options
    opt = vis.get_render_option()
    # Disable shading and shadows
    opt.light_on = False

    for geo in args:
        vis.add_geometry(geo)

    running = True
    while running:
        running = vis.poll_events()
        vis.update_renderer()

    for geo in args:
        vis.remove_geometry(geo)

    vis.destroy_window()


if __name__ == "__main__":

    # Create a visualizer
    vis = o3d.visualization.Visualizer()

    # Coord axis
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)


    # Import point clouds
    pcd1 = o3d.io.read_point_cloud("iphone1cropmore.ply") # Source
    print(pcd1)

    

    draw(vis,pcd1)

    #pcd1.colors = o3d.cpu.pybind.utility.Vector3dVector((np.asarray(pcd1.points)[:,1:2] > 2.5) * [0.5,0.5,0.5])
    # Cropping
    # points = np.asarray(pcd1.points)
    # keep = points[:,1:2] < 2.3
    # pcd1.points = o3d.cpu.pybind.utility.Vector3dVector(points*keep) # Crop y
    

    # For screenshot
    pcd1_copy = copy.deepcopy(pcd1)
    pcd1_copy.rotate([[1,0,0],[0,0,-1],[0,1,0]])
    draw(vis,pcd1_copy)

    pcd2 = o3d.io.read_point_cloud("iphone2cropmore.ply") # Target
    print(pcd2)
    # Cropping

    # points = np.asarray(pcd2.points)
    # keep = points[:,1:2] < 2.3
    # pcd2.points = o3d.cpu.pybind.utility.Vector3dVector(points*keep) # Crop y

    # draw(vis,pcd2)
    # Downsample point clouds
    voxel_size = 0.05
    downpcd1 = pcd1.voxel_down_sample(voxel_size=voxel_size) # Voxel size in m
    print(downpcd1)
    # draw(vis,downpcd1)
    downpcd2 = pcd2.voxel_down_sample(voxel_size=voxel_size) # Voxel size in m
    print(downpcd2)
    # draw(vis,downpcd2)

    # Estimate normals
    # Search radius for neighbors in m, and max no. neighbors
    downpcd1.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    downpcd2.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # Planar patch detection
    for pcd in (downpcd1,downpcd2):
        oboxes = pcd.detect_planar_patches(
            normal_variance_threshold_deg=60, # Smaller values tends to result in fewer, higher quality planes
            coplanarity_deg=75, # Larger values encourage a tighter distribution of points around the fitted plane
            outlier_ratio=0.75, # maximum allowable outlier ratio in a fitted planes associated set of points before being rejected
            min_plane_edge_length=2, # A planar patch’s largest edge much be greater than this value to be considered a true planar patch
            min_num_points=0, # How many points must be present when attempting to fit a plane
            search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))
        print("Detected {} patches".format(len(oboxes)))
        geometries = []
        for obox in oboxes:
            mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obox, scale=[1, 1, 0.0001])
            mesh.paint_uniform_color(obox.color)
            geometries.append(mesh)
            geometries.append(obox)
            
            # Add a line
            line = o3d.geometry.LineSet()
            line.points = o3d.utility.Vector3dVector([obox.center,obox.center + obox.R[:,2]])
            line.lines = o3d.utility.Vector2iVector([[0, 1]])
            geometries.append(line)

            # Eqn of plane

        geometries.append(pcd)


    # Point to point ICP
    threshold = 2*voxel_size # Maximum allowable distance between corresponding points after alignment. 1-3x voxel size
    init_trans = np.array([[1,0,0,0],[0,1,0,0.1],[0,0,1,0.2],[0,0,0,1]]) # for iphone scans
    #init_trans = np.array([[1,0,0,0],[0,1,0,-1],[0,0,-1,0],[0,0,0,1]]) # for gv01
    reg_p2p = o3d.pipelines.registration.registration_icp( # source, target
        downpcd1, downpcd2, threshold, init_trans,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=10000))
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)
    draw_registration_result(vis,downpcd1, downpcd2, reg_p2p)

    # Alpha shapes surface reconstruction
    # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(downpcd, 0.2) # Larger alpha makes larger triangles
    # mesh.compute_vertex_normals()
    #o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)




