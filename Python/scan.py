import open3d as o3d
import numpy as np
from ble import run
from fusion import fusion, queue_R, queue_lidar
import asyncio
from config import Cartesian, direc, r

queue_point = asyncio.Queue() # Queue for point cloud coordinates (cartesian objects)
queue_rotation = asyncio.Queue() # Queue for calculations function to pass rotations for vis function

# Reads from R and lidar queues, generating point cloud data
# Blocks on waiting for an item from both queue_R and queue_lidar
async def scan():

    while 1:
        # Wait for R and lidar items
        R = await queue_R.get()
        queue_R.task_done()

        lidar = await queue_lidar.get()
        queue_lidar.task_done()
        dist = lidar.dist # Extract distance (m)

        # Data check
        if dist > 0:

            # Pass the rotation to the queue for the visualisation (used for box rotation)
            queue_rotation.put_nowait(R)

            # Find coord
            coord = R @ ((dist+r)*direc) # Calculate coordinate
            queue_point.put_nowait(Cartesian(coord[0,0],coord[1,0],coord[2,0])) # Add point to queue

        else: print("Skipping point...")

# Reads from point queue, visualising point cloud data
# Blocks on waiting for new data
async def scanVis():

    # Open3d setup
    vis = o3d.visualization.Visualizer()
    vis.create_window(height=480*5, width=640*5)
    # Initialise pointcloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array([[-6,-6,-6],[6,6,6]])) # First two points are dummy points
    vis.add_geometry(pcd)

    # # Add coord axis
    # axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    # vis.add_geometry(axis)

    # Add box
    BOX_SCALE = 3
    BOX_DIMS = (0.09402, 0.0594, 0.03932) # x,y,z lengths
    LIDAR_CENTRE = (0.03376,-0.003,-BOX_DIMS[2]/2) # Where lidar distance is measured from (take bw lidar emitter/reciever)
    LIDAR_DIMS = (0.0185,0.035) # x,y
    LASER_CENTRE = (0.03376,0.01995)
    LASER_R = 0.00285
    points = [[-BOX_DIMS[0]/2, -BOX_DIMS[1]/2, -BOX_DIMS[2]/2],[BOX_DIMS[0]/2, -BOX_DIMS[1]/2, -BOX_DIMS[2]/2],
              [-BOX_DIMS[0]/2, BOX_DIMS[1]/2, -BOX_DIMS[2]/2],[BOX_DIMS[0]/2, BOX_DIMS[1]/2, -BOX_DIMS[2]/2],
              [-BOX_DIMS[0]/2, -BOX_DIMS[1]/2, BOX_DIMS[2]/2],[BOX_DIMS[0]/2, -BOX_DIMS[1]/2, BOX_DIMS[2]/2],
              [-BOX_DIMS[0]/2, BOX_DIMS[1]/2, BOX_DIMS[2]/2],[BOX_DIMS[0]/2,BOX_DIMS[1]/2, BOX_DIMS[2]/2], # Centred box

              [LIDAR_CENTRE[0]+LIDAR_DIMS[0]/2,LIDAR_CENTRE[1]+LIDAR_DIMS[1]/2,-BOX_DIMS[2]/2],
              [LIDAR_CENTRE[0]+LIDAR_DIMS[0]/2,LIDAR_CENTRE[1]-LIDAR_DIMS[1]/2,-BOX_DIMS[2]/2],
              [LIDAR_CENTRE[0]-LIDAR_DIMS[0]/2,LIDAR_CENTRE[1]+LIDAR_DIMS[1]/2,-BOX_DIMS[2]/2],
              [LIDAR_CENTRE[0]-LIDAR_DIMS[0]/2,LIDAR_CENTRE[1]-LIDAR_DIMS[1]/2,-BOX_DIMS[2]/2], # Lidar box

              [LASER_CENTRE[0]+LASER_R,LASER_CENTRE[1],-BOX_DIMS[2]/2],
              [LASER_CENTRE[0]-LASER_R,LASER_CENTRE[1],-BOX_DIMS[2]/2],
              [LASER_CENTRE[0],LASER_CENTRE[1]+LASER_R,-BOX_DIMS[2]/2],
              [LASER_CENTRE[0],LASER_CENTRE[1]-LASER_R,-BOX_DIMS[2]/2], # Laser box
              ]
    for list in points:
        list[0] = list[0] - LIDAR_CENTRE[0] # Translate lidar centre to origin
        list[1] = list[1] - LIDAR_CENTRE[1]
        list[2] = list[2] - LIDAR_CENTRE[2]
        list[:] = [x*BOX_SCALE for x in list] # Scale by box scale
        list[0] = list[0] + r*direc[0] # Translate box centre by rotation radius, in direction of lidar beam
        list[1] = list[1] + r*direc[1]
        list[2] = list[2] + r*direc[2]

        
    lines = [[0, 1],[0, 2],[1, 3],[2, 3],[4, 5],[4, 6],[5, 7],[6, 7],[0, 4],[1, 5],[2, 6],[3, 7], # Box
             [8,9],[8,10],[11,9],[11,10], # Lidar box
             [12,14],[12,15],[13,14],[13,15] # Laser box
            ]
    line_box = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines))
    vis.add_geometry(line_box)

    # Setup laser line
    line_laser = o3d.geometry.LineSet()
    line_laser.points = o3d.utility.Vector3dVector([[0,0,0],[1,1,1]]) # Initial ends of line
    line_laser.lines = o3d.utility.Vector2iVector([[0, 1]])
    line_laser.colors = o3d.utility.Vector3dVector([[1, 0, 0]])  # RGB for red
    vis.add_geometry(line_laser)

    # Loop
    POINT_SKIP = 4 # Skip some points to increase responsiveness
    running = True
    while running:

        # Add points
        for _ in range(POINT_SKIP+1): # Add POINT_SKIP points before visualising
            point = await queue_point.get()
            queue_point.task_done()
            # o3d stores points as Nx3 matrix
            pcd.points.extend(np.array([[point.x, point.y, point.z]]))
            # Grab rotations from the queue, only the last one is used
            R = await queue_rotation.get()
            queue_rotation.task_done()


        # Rotate box
        line_box.points = o3d.utility.Vector3dVector(points) # Reset points
        line_box.rotate(R,center=(0,0,0)) # Apply most recent rotation

        # Set laser line
        line_laser.points = o3d.utility.Vector3dVector([0.5*(line_box.points[-1]+line_box.points[-2]),[point.x,point.y,point.z]])

        vis.update_geometry(line_box) # Update box
        vis.update_geometry(pcd) # Update point cloud
        vis.update_geometry(line_laser) # Update laser

        # For o3d visualiser
        running = vis.poll_events() # Returns false on window close
        vis.update_renderer()

    vis.destroy_window()

    # Save point cloud if user wants
    save = input("\nWould you like to save this point cloud? y/n\n")
    if save == "y":
        pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points[2:]))
        if o3d.io.write_point_cloud("pointcloud.pcd",pcd):
            print('\nSaved!\n')
        else:
            print('\nError saving point cloud\n')




if __name__ == "__main__":

    run(fusion(calibrate=True,debug=False,init=False),scan(),scanVis())