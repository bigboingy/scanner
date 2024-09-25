import matplotlib.pyplot as plt
import numpy as np
from ble import run, read
import math

async def graph():

    # Graph setup
    MAX_READS = 100
    x_init = range(1,MAX_READS+1)
    y_init = np.linspace(-1,1,MAX_READS)
    # Define
    fig, (ax0,ax1,ax2,ax3) = plt.subplots(1,4) # Axis 0 for lidar, axis 1 for acc, Axis 2 for gyro, Axis 3 for mag
    l0 = ax0.plot(x_init,y_init)
    l1 = ax1.plot(x_init,y_init,x_init,y_init,x_init,y_init)
    l2 = ax2.plot(x_init,y_init,x_init,y_init,x_init,y_init)
    l3 = ax3.plot(x_init,y_init,x_init,y_init,x_init,y_init)
    # Style
    ax0.set_xlim([1, MAX_READS]) # Lidar limits
    ax0.set_ylim([0, 10])
    ax0.set_title('Distance (mm)')
    ax1.set_title('Accelerometer (g)')
    ax2.set_title('Gyroscope (dps)')
    ax3.set_title('Magnetometer (uT)')
    ax1.legend(l1,('x','y','z'),loc='upper left')
    ax2.legend(l2,('x','y','z'),loc='upper left')
    ax3.legend(l3,('x','y','z'),loc='upper left')
    # # Mag 3D figure
    # fig1 = plt.figure()
    # ax4 = plt.axes(projection='3d')
    # ax4.set_xlim3d(-1,1) 
    # ax4.set_ylim3d(-1,1) 
    # ax4.set_zlim3d(-1,1) 
    # ax4.set_xlabel('x')
    # ax4.set_ylabel('y')
    # ax4.set_zlabel('z')
    # x_mag, y_mag, z_mag = [0], [0], [0] # Lists for mag scatter plot
    # scat = ax4.scatter(x_mag,y_mag,z_mag,marker='o',c='r') # Plot origin as a red dot
    
    while True:
        
        # Get ble data
        data = await read()

        # Lidar update
        # Get current y vals
        y_old = l0[0].get_ydata()
        # Extract distance values
        dists = []
        for lidar in data['lidar']:
            dists.append(lidar.dist)
        # Append with newest lidar values
        y_combined = np.concatenate((y_old,dists))
        # Set data
        l0[0].set_ydata(y_combined[-MAX_READS:])

        # IMU update
        imus = data["imu"]
        newReads = np.empty((3,3*len(imus))) # Initialise matrix (3x3N) where N is no reads
        for read_index,imu in enumerate(imus): # Add new reads to 3xN array. Row is coord col is sensor

            #imu.calibrate(magCal=False,accCal=False,magAlign=False,gyroCal=False) # CARE this affects 3d graph

            newReads[:,3*read_index:3*read_index+3] = imu.extract()

        for sensor_index,l in enumerate((l1,l2,l3)): # For each sensor, get corresponding line list
            for coord_index in range(3): # For each line (coordinate)
                y_old = l[coord_index].get_ydata()
                y_new = newReads[coord_index,sensor_index::3] # Rows are coords and cols are sensors
                y_combined = np.hstack((y_old,y_new))
                l[coord_index].set_ydata(y_combined[-MAX_READS:])

        # Rescale graphs
        ax1.relim(); ax2.relim(); ax3.relim()
        ax1.autoscale(); ax2.autoscale(); ax3.autoscale()

        # # Mag 3D graph update
        # MAX = 100
        # for reading in imus:
        #     reading.calibrate(magCal=True,accCal=False,magAlign=True,gyroCal=False)

        #     # Add to storage
        #     if abs(reading.mag.x)<MAX: x_mag.append(reading.mag.x)
        #     if abs(reading.mag.y)<MAX: y_mag.append(reading.mag.y)
        #     if abs(reading.mag.z)<MAX: z_mag.append(reading.mag.z)

        #     # Work out heading
        #     heading = 0
        #     if reading.mag.x != 0: heading = math.atan2(reading.mag.y,reading.mag.x)*180/math.pi
        #     if heading < 0: heading = heading + 360
        #     print(heading)
        
        #     # Prepare new 'offsets'
        #     scat._offsets3d = (x_mag,y_mag,z_mag)

        # # Rescale
        # ax4.set_xlim3d(min(x_mag),max(x_mag)); ax4.set_ylim3d(min(y_mag),max(y_mag)); ax4.set_zlim3d(min(z_mag),max(z_mag))
        # ax4.set_box_aspect([1,1,1])

        # For graph event loop
        plt.pause(0.05)

if __name__ == "__main__":
    run(graph(),loop=True)