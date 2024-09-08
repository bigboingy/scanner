import matplotlib.pyplot as plt
import numpy as np
from ble import run, read

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
    ax0.set_ylim([0, 10000])
    ax0.set_title('Distance (mm)')
    ax1.set_title('Accelerometer (g)')
    ax2.set_title('Gyroscope (dps)')
    ax3.set_title('Magnetometer (uT)')
    ax1.legend(l1,('x','y','z'),loc='upper left')
    ax2.legend(l2,('x','y','z'),loc='upper left')
    ax3.legend(l3,('x','y','z'),loc='upper left')
    
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
            newReads[:,3*read_index:3*read_index+3] = imu.calibrate(magCal=True,accCal=True,magAlign=True,gyroCal=True).extract()

        for sensor_index,l in enumerate((l1,l2,l3)): # For each sensor, get corresponding line list
            for coord_index in range(3): # For each line (coordinate)
                y_old = l[coord_index].get_ydata()
                y_new = newReads[coord_index,sensor_index::3] # Rows are coords and cols are sensors
                y_combined = np.hstack((y_old,y_new))
                l[coord_index].set_ydata(y_combined[-MAX_READS:])

        # Rescale graphs
        ax1.relim(); ax2.relim(); ax3.relim()
        ax1.autoscale(); ax2.autoscale(); ax3.autoscale()

        # For graph event loop
        plt.pause(0.05)
        
        
if __name__ == "__main__":
    run(graph(),loop=True)