import imufusion
import numpy as np
from config import FREQ_LIDAR
from ble import run, read
import asyncio
import open3d as o3d
#import time

queue_R = asyncio.Queue() # Defaults to infinite length

#queue_time = asyncio.Queue() # For debugging how long it takes for read to be visualised

# Loop to read imu vals from bt and apply fusion
# Pushes rotation matrices to a queue
# Blocks on read()
async def fusion():

    # Fusion setup
    ahrs = imufusion.Ahrs()
    ahrs.settings = imufusion.Settings(imufusion.CONVENTION_NWU,  # convention
                                    0.5,  # gain
                                    1000,  # gyroscope range
                                    10,  # acceleration rejection
                                    10,  # magnetic rejection, max difference bw algorithm and magnetometer before mag is ignored
                                    5 * FREQ_LIDAR)  # recovery trigger period = 5 seconds
    # How many rotation matrices to send to queue_R. 
    R_TO_VIS = 3 # 3 seems good at current settings. Each one takes more time in visuslisation

    # Infinite loop
    while 1:

        # Get ble data
        imus = (await read())["imu"] # Only keep the imus list, discarding lidars
        step = (len(imus)-1)//(R_TO_VIS-1)

        # Loop for each imu returned, putting R into the queue
        for imu in imus:
            imu.calibrate(magCal=True,accCal=True,magAlign=True,gyroCal=False) # Calibrate imu object, changing its values
            data = imu.extract()
            # Update sensor fusion. Rakes gyro (1 by 3 matrix), acc (1 by 3 matrix), mag (1 by 3 matrix) and dt
            ahrs.update(data[:,1],data[:,0],data[:,2],imu.count)

            # Add rotation matrix to queue if it's to be visualised
            if imu in imus[::step]:
                queue_R.put_nowait(ahrs.quaternion.to_matrix())
                #queue_time.put_nowait(time.time())

            # Print status flags and internal states
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
        
        

# Reads fusion output from the queue to run o3d visualisaion
# Blocks on waiting for a queue item
async def fusionVis():

    # Open3d visualisation setup
    vis = o3d.visualization.Visualizer()
    vis.create_window(height=480*5, width=640*5)
    points = [[-.8, -.4, -.2],[.8, -.4, -.2],[-.8, .4, -.2],[.8, .4, -.2], # Make a cube
              [-.8, -.4, .2],[.8, -.4, .2],[-.8, .4, .2],[.8, .4, .2]]
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
    running = True # Loop can exit on window close
    while running:

        # Get rotation
        R = await queue_R.get() # Wait for a rotation matrix to become available
        queue_R.task_done()

        # stamp = queue_time.get_nowait()
        # queue_time.task_done()
        # print(f"It took {time.time()-stamp}s to visualise")

        # Update geometry. The following actions cost ~0.01 seconds
        line_set.points = o3d.utility.Vector3dVector(points) # Reset points
        line_set.rotate(R) # Apply rotation
        vis.update_geometry(line_set) # Update geometry
        # Requred for o3d interaction
        running = vis.poll_events() # Returns false on window close
        vis.update_renderer()

# Test
if __name__ == "__main__":

    a = [1,2,3,4,5,6,7,8,9]


    run(fusion(),fusionVis(),loop=True)
    
