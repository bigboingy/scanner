import bt
import constants as cnst
import imufusion
import numpy as np
import open3d as o3d
import calibration
import fusion

# Open port
port = bt.getPortHandle(port="/dev/cu.HC-05")

# Open3d visualisation
vis = o3d.visualization.Visualizer()
vis.create_window(height=480*5, width=640*5)
# Make a cube. Dimensions are in imu frame
r = cnst.RADIUS
cubePoints = [
    [-2*r, -r, -r/2],
    [2*r, -r, -r/2],
    [-2*r, r, -r/2],
    [2*r, r, -r/2],
    [-2*r, -r, r/2],
    [2*r, -r, r/2],
    [-2*r, r, r/2],
    [2*r, r, r/2],
]

cubeLines = [
        [0, 1],
        [0, 2],
        [1, 3],
        [2, 3],
        [4, 5],
        [4, 6],
        [5, 7],
        [6, 7],
        [0, 4],
        [1, 5],
        [2, 6],
        [3, 7],
]
cube = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(cubePoints),
    lines=o3d.utility.Vector2iVector(cubeLines),)
vis.add_geometry(cube)
# Initialise pointcloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.array([[-6,-6,-6],[6,6,6]]))
vis.add_geometry(pcd)
# Add coord axis
axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
vis.add_geometry(axis)
cam = vis.get_view_control() # For camera control. This call can result in segmentation error if too early??

# Imu fusion setup
ahrs = imufusion.Ahrs()
sample_rate = cnst.SAMPLE_RATE # Hz. On average 100 Hz when I set minlooptime to 10ms
ahrs.settings = imufusion.Settings(imufusion.CONVENTION_NED,  # convention - north east down
                                   0.5,  # gain
                                   1000,  # gyroscope range
                                   10,  # acceleration rejection
                                   10,  # magnetic rejection, max difference bw algorithm and magnetometer before mag is ignored
                                   5 * sample_rate)  # recovery trigger period = 5 seconds

# Store bytes from incomplete packets, for bt.read
unprocessedBytes = bytearray()
# Make initial request for data
bt.write(port,lidarOn=True,imuOn=True,count=-1)

# Loop
running = True
prevCounter = 0xFFFF/cnst.DATATIMER_FREQ # Initialisation. You need to put in dt between fusion updates!
while running:
    # These run all the time
    running = vis.poll_events() # Returns false on window close
    vis.update_renderer()

    data = {
        'lidar':[], # Contains lidar tuples
        'imu':[] # Imu tuples
    }

    # Read serial buffer and append to data
    newData = bt.read(port, unprocessedBytes)
    if newData['lidar']: data['lidar'].extend(newData['lidar'])
    if newData['imu']: data['imu'].extend(newData['imu'])

    # If 1+ lidar and imu have come through...
    if data['lidar'] and data['imu']:

        # Isolate pairs and trim data
        pairs = min(len(data['lidar']),len(data['imu']))
        lidars = data['lidar'][0:pairs]
        del data['lidar'][0:pairs]
        imus = data['imu'][0:pairs]
        del data['imu'][0:pairs]

        bt.write(port,lidarOn=True,imuOn=True,count=-1) # Respond so that timeout doesn't occur
        # Calibrate imu
        imu_cal = calibration.applyCalibration(imus,gyroCal=False,accCal=True)
        # Fusion for each imu
        
        ahrs,R,prevCounter = fusion.imuFusion(ahrs,imu_cal,prevCounter)

        # Apply rotations to lidars
        newPoints = np.empty((3,pairs)) # 
        for i,lidar in enumerate(lidars):
            # In Earth frame...
            centre_to_lidar = r * R[i] @ np.array(cnst.IMU_DIREC).reshape((3,1)) 
            lidar_to_point = lidar.dist * R[i] @ np.array(cnst.LIDAR_DIREC).reshape((3,1)) 
            position = centre_to_lidar + lidar_to_point

            # lidarVec = ( lidar.dist*np.array(cnst.LIDAR_DIREC) + cnst.RADIUS*np.array(cnst.LIDAR_DIREC) ).reshape((3,1)) # Get column vector
            newPoints[:,[i]] = position # Apply rotation and add to array
        
        # Update geometry
        pcd.points.extend(newPoints.T) # Need to take transpose as o3d stores as Nx3 matrix
        cube.points = o3d.utility.Vector3dVector(cubePoints) # Reset points
        cube.translate(centre_to_lidar,relative=False)
        cube.rotate(R[-1]) # Apply last rotation to box
        vis.update_geometry(cube)
        vis.update_geometry(pcd)

        # Move camera
        # cam_params = cam.convert_to_pinhole_camera_parameters()
        # camMatrix = np.copy(cam_params.extrinsic)
        # camMatrix[0:3,0:3] = np.array([[1,0,0],[0,0,-1],[0,1,0]]) @ np.array([[1,0,0],[0,0,-1],[0,1,0]]) @ R[-1]
        # cam_params.extrinsic = camMatrix
        # cam.convert_from_pinhole_camera_parameters(cam_params,True)
    


vis.destroy_window()
