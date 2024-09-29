import numpy as np
import open3d as o3d

# Function processes pointcloud of a wall to estimate rotation distance, r
# Pointcloud is generated with r = 0
# Function increases r of the points, using RANSAC to score each r value

def scan_calibrate(pcd):

    # Open3d setup
    vis = o3d.visualization.Visualizer()
    vis.create_window(height=480*5, width=640*3)
    vis.add_geometry(pcd)
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(np.array(([0,0,0],[1,0,0],[0,1,0],[0,0,1]))) # dummy mesh
    mesh.triangles = o3d.utility.Vector3iVector(np.array([[0, 1, 2],[3, 2, 1]]))
    vis.get_render_option().mesh_show_back_face = True # Set the rendering options to disable backface culling
    mesh.paint_uniform_color([0.9, 0.2, 0.5])
    vis.add_geometry(mesh)

    # Try between these r values
    R_MIN = 0
    R_MAX = 1
    VALS = 100 # No. values to trial
    PLANE_SIZE = 1 # In y and z coords

    # Initialise original pcd points. pcd is changed, so don't reference it
    points = np.empty(np.asarray(pcd.points).shape)
    points[:,:] = np.asarray(pcd.points)[:,:]
    # Initialise new pcd points
    newPoints = np.empty(np.shape(points))

    # Variables to store best scenario
    min_sum = 10000 # Store minimum sum of residuals
    best_r = 0 # Store associated r value
    best_points = np.empty(points.shape) # Store best points
    best_vertices = np.empty((4,3)) # Store best mesh vertices
    
    # For each r value being trialled
    for r in np.linspace(R_MIN,R_MAX,VALS):
        
        # Extend coordinates by current r
        for i,point in enumerate(points):
            norm = np.linalg.norm(point)
            newPoint = point + r/norm*point
            newPoints[i,:] = newPoint

        # Generate plane using SVD algorithm from chatGPT
        centroid = np.mean(newPoints,axis = 0)
        newPointsCentred = newPoints - centroid
        _, _, Vt = np.linalg.svd(newPointsCentred)
        normal = Vt[-1, :]
        a, b, c = normal
        d = -np.dot(normal, centroid) # Plane passes through centroid

        # Create square plane
        f = lambda y,z: -(b*y+c*z+d)/a # Find x using z and y
        vertices = np.array([
            [f(-PLANE_SIZE,-PLANE_SIZE),-PLANE_SIZE,-PLANE_SIZE],
            [f(-PLANE_SIZE,PLANE_SIZE),-PLANE_SIZE,PLANE_SIZE],
            [f(PLANE_SIZE,-PLANE_SIZE),PLANE_SIZE,-PLANE_SIZE],
            [f(PLANE_SIZE,PLANE_SIZE),PLANE_SIZE,PLANE_SIZE]
        ])
        mesh.vertices = o3d.utility.Vector3dVector(vertices)

        # Sum distances of each point from the plane
        denominator = np.sqrt(a**2 + b**2 + c**2)
        sum = 0
        for point in newPoints:
            distance = np.abs(a*point[0]+b*point[1]+c*point[2]+d)/denominator
            sum+=np.abs(distance)

        # Is this residual sum the lowest?
        if sum < min_sum:
            min_sum = sum
            best_r = r
            best_points[:,:] = newPoints[:,:] # Fill matrix
            best_vertices = vertices
            print(f'Found new r: {r:.4f}, sum of residuals: {sum:.3f}')

        # Update pcd
        pcd.points = o3d.utility.Vector3dVector(newPoints)
        vis.update_geometry(pcd)
        vis.update_geometry(mesh)

        # Run visualiser, limited by computer speed
        vis.poll_events()
        vis.update_renderer()

    # Finally, show best fit
    pcd.points = o3d.utility.Vector3dVector(best_points)
    vis.update_geometry(pcd)
    mesh.vertices = o3d.utility.Vector3dVector(best_vertices)
    mesh.paint_uniform_color([0.27, 0.79, 0.4])
    vis.update_geometry(mesh)

    # Keep renderer going
    running = True
    while running:
        running = vis.poll_events()
        vis.update_renderer()
    vis.destroy_window()

    # Finally, ask user if they want to save r
    save = input("\nWould you like to save this r value? y/n\n")
    if save == "y":
        
        np.savetxt("params_r",np.array([best_r]))






if __name__ == "__main__":
    
    
    scan_calibrate(o3d.io.read_point_cloud("pointcloud.pcd"))