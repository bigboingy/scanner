import imufusion
import numpy as np
from config import FREQ_LIDAR
from ble import run, read
import asyncio
import open3d as o3d

queue_R = asyncio.Queue() # Queue for rotation matrices (3x3 np arrays)
queue_lidar = asyncio.Queue() # Queue for lidars, which correspond with R in queue_R (lidar objects)

# Loop to read imu vals from bt and apply fusion
# Pushes rotation matrices and lidars to queues
# Params:
# calibrate enables calibration of imu objects
# debug enables status printing
# init can be set to False to stop initalisation Rs and lidars being passed to queue
          
# Blocks on read()
async def fusion(calibrate=False, debug=False, init=True):

    # Fusion setup
    ahrs = imufusion.Ahrs()
    ahrs.settings = imufusion.Settings(imufusion.CONVENTION_NWU,  # convention
                                    0.5,  # gain
                                    1000,  # gyroscope range
                                    10,  # acceleration rejection
                                    10,  # magnetic rejection, max difference bw algorithm and magnetometer before mag is ignored
                                    5 * FREQ_LIDAR)  # recovery trigger period = 5 seconds

    # Infinite loop
    while 1:
        # Get ble data
        data_ble = await read()            
        imus = data_ble['imu'] # Extract imus
        lidars = data_ble['lidar'] # Extract lidars

        # Loop for each imu returned, putting R into the queue. Index is used to get corresponding lidar
        for i,imu in enumerate(imus):
            # Calibrate imu object, changing its values
            if calibrate: imu.calibrate(magCal=True,accCal=True,magAlign=True,gyroCal=False)
            reading = imu.extract() # Extract data

            # Update sensor fusion. Takes gyro (1 by 3 matrix), acc (1 by 3 matrix), mag (1 by 3 matrix) and dt
            ahrs.update(reading[:,1],reading[:,0],reading[:,2],imu.count)

            # Pass values to queues if init is true or ahrs is finished initialising
            flags = ahrs.flags
            if init or not flags.initialising:
                # Add rotation matrix to queue
                queue_R.put_nowait(ahrs.quaternion.to_matrix())
                queue_lidar.put_nowait(lidars[i])
            else:
                print("Initialising imu fusion, please wait...")

            # If debug is set to true, print status flags and internal states
            if debug:
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

# Reads fusion output from the queue to run o3d visualisaion of rotation matrices
# Blocks on waiting for a queue_R matrix
async def fusionVis():

    # Open3d visualisation setup
    vis = o3d.visualization.Visualizer()
    vis.create_window(height=480*5, width=640*5)
    points = [[-.8, -.4, -.32],[.8, -.4, -.32],[-.8, .4, -.32],[.8, .4, -.32], # Make a box
              [-.8, -.4, .32],[.8, -.4, .32],[-.8, .4, .32],[.8, .4, .32]]
    lines = [[0, 1],[0, 2],[1, 3],[2, 3],[4, 5],[4, 6],
            [5, 7],[6, 7],[0, 4],[1, 5],[2, 6],[3, 7]]
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines))
    vis.add_geometry(line_set)
    # Add coord axis
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    vis.add_geometry(axis)

    # Loop to update geometry
    R_SKIP = 5 # How many rotation matrices to skip, so that animation doesn't lag
    running = True # Loop can exit on window close
    while running:

        # Get rotation
        for _ in range(R_SKIP+1): # Skip R_SKIP rotation matrices
            R = await queue_R.get() # Wait for a rotation matrix to become available
            queue_R.task_done()

        # Update geometry. The following actions cost ~0.01 seconds
        line_set.points = o3d.utility.Vector3dVector(points) # Reset points
        line_set.rotate(R) # Apply rotation
        vis.update_geometry(line_set) # Update geometry
        # Requred for o3d interaction
        running = vis.poll_events() # Returns false on window close
        vis.update_renderer()

    vis.destroy_window()

# Test
if __name__ == "__main__":

    run(fusion(calibrate=True, debug=True),fusionVis(),loop=True)
    
