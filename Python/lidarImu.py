import bt
import constants as cnst
import imufusion
import numpy as np
import open3d as o3d
import calibration


def o3d_loop(vis,pcd,cube):



    return



# Open port
# timeout waits for return of requested no. bytes specified in read() function, and also in port opening
port = bt.getPortHandle(port="/dev/cu.HC-05")

# Open3d visualisation
vis = o3d.visualization.Visualizer()
vis.create_window(height=480*5, width=640*5)
# Make a cube
cubePoints = [
        [-8, -4, -2],
        [8, -4, -2],
        [-8, 4, -2],
        [8, 4, -2],
        [-8, -4, 2],
        [8, -4, 2],
        [-8, 4, 2],
        [8, 4, 2],
]
# for pt in cubePoints:
#     for coord in pt:
#         coord = coord/10
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
pcd.points = o3d.utility.Vector3dVector(np.zeros((1,3))) # Put a pt at 000
vis.add_geometry(pcd)
# Add coord axis
axis = o3d.geometry.TriangleMesh.create_coordinate_frame()
vis.add_geometry(axis)
cam = vis.get_view_control() # For camera control. This call can result in segmentation error if too early??
# Imu fusion setup
ahrs = imufusion.Ahrs()
sample_rate = cnst.SAMPLE_RATE # Hz. On average 100 Hz when I set minlooptime to 10ms
ahrs.settings = imufusion.Settings(imufusion.CONVENTION_ENU,  # convention - east north up
                                   0.5,  # gain
                                   1000,  # gyroscope range
                                   10,  # acceleration rejection
                                   10,  # magnetic rejection, max difference bw algorithm and magnetometer before mag is ignored
                                   5 * sample_rate)  # recovery trigger period = 5 seconds

# Rejecting acc, mag is still rejected sometimes --> mag is unaligned with gyro
# Rejecting mag, acc is not rejected --> acc agrees with gyro
# Mag is unaligned with gyro/acc --> allign

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

    # Read serial buffer and append
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
        # Takes gyro (1 by 3 matrix), acc (1 by 3 matrix), mag (1 by 3 matrix) and dt
        R = [] # Store rotation matrices
        for imu in imu_cal:
            # Turn clock counter into a time difference
            if prevCounter > imu.time:
                dt = prevCounter-imu.time
            else:
                dt = prevCounter+(0xFFFF/cnst.DATATIMER_FREQ-imu.time) # 0xFFFF is the max counter (what clock resets to after reaching 0)
            
            # Update sensor fusion
            ahrs.update(np.array(imu.gyro), np.array(imu.acc), np.array(imu.mag), dt) # Transpose M to make row vector again
            prevCounter = imu.time

            # Check status flags and internal states
            flags = ahrs.flags
            if flags.initialising: print('Initialising')
            if flags.angular_rate_recovery: print('Angular rate recovery')
            if flags.acceleration_recovery: print('Acceleration recovery')
            if flags.magnetic_recovery: print('Magnetic recovery')
            states = ahrs.internal_states
            if states.accelerometer_ignored:
                print('Accelerometer ignored, error: %.1f' % states.acceleration_error)
            if states.magnetometer_ignored:
                print('Magnetometer ignored, error: %.1f' % states.magnetic_error)
            print('x')
            # Save rotation matrix
            R.append(ahrs.quaternion.to_matrix())

        # Apply rotations to lidars
        newPoints = np.empty((3,pairs)) # 3xN matrix
        for i,lidar in enumerate(lidars):
            lidarVec = lidar.dist*np.array(cnst.LIDAR_DIREC).reshape((3,1)) # Get column vector
            newPoints[:,[i]] = cnst.IMU_TO_O3D @ np.linalg.inv(R[i]) @ lidarVec # Apply rotation and add to array
        
        # Update geometry
        pcd.points.extend(newPoints.T) # Need to take transpose as o3d stores as Nx3 matrix
        cube.points = o3d.utility.Vector3dVector(cubePoints) # Reset points
        cube.rotate(cnst.IMU_TO_O3D @ R[-1]) # Apply last rotation to box
        vis.update_geometry(cube)
        vis.update_geometry(pcd)

        # Move camera
        # cam_params = cam.convert_to_pinhole_camera_parameters()
        # camMatrix = np.copy(cam_params.extrinsic)
        # camMatrix[0:3,0:3] = constRot @ R[-1].T
        # cam_params.extrinsic = camMatrix
        # cam.convert_from_pinhole_camera_parameters(cam_params,True)
    


vis.destroy_window()
