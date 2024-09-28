import numpy as np
import open3d as o3d
import copy

# Function processes pointcloud of a wall to estimate rotation distance, r
# Pointcloud is generated with r = 0
# Function increases r of the points, using RANSAC to score each r value

def scan_calibrate(pcd):

    # Must make a deep copy of points, otherwise changing pcd.points will change these
    points = copy.deepcopy(np.asarray(pcd.points)) # [x,y,z] stacked

    # Try between these r values
    R_MIN = 0
    R_MAX = 1
    VALS = 1000 # No. values to trial

    # Initialise new pcd points
    newPoints = np.empty(np.shape(points))

    # Open3d setup
    vis = o3d.visualization.Visualizer()
    vis.create_window(height=480*5, width=640*3)
    vis.add_geometry(pcd)

    max_inliers = 0 # Store the max no. inliers obtained by RANSAC
    # For each r value being trialled
    for r in np.linspace(R_MIN,R_MAX,VALS):
        
        # Extend coordinates by current r
        for i,point in enumerate(points):
            norm = np.linalg.norm(point)
            newPoint = point + r/norm*point
            newPoints[i,:] = newPoint

        # Generate plane
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                                 ransac_n=np.shape(points)[0],
                                                 num_iterations=1)

        # Check if this r value is the best so far
        if len(inliers) > max_inliers:
            max_inliers = len(inliers)
            print(f'Found new r: {r:.4f}, points included: {len(inliers)/len(points)*100:.2f}%')
                    
        # Update pcd
        pcd.points = o3d.utility.Vector3dVector(newPoints)
        vis.update_geometry(pcd)

        # Run visualiser, limited by computer speed?
        vis.poll_events() # Returns false on window close
        vis.update_renderer()

    # Finally
    vis.destroy_window()



if __name__ == "__main__":
    
    
    scan_calibrate(o3d.io.read_point_cloud("pointcloud.pcd"))