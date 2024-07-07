import imu
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from functools import partial

# File for graphing each imu sensor in 2d

# Get port handle
port = imu.getPortHandle(port="COM4", baud=115200, timeout=3)

## GRAPHING
# Graphing setup
plt.style.use('fivethirtyeight')
MAX_VALS = 20 # How many values to display on graph
initx = range(1,MAX_VALS+1)
inity = np.linspace(-1,1,MAX_VALS)
# Define
fig, (ax0,ax1,ax2) = plt.subplots(1,3) # Axis 0 for acc, Axis 1 for gyro, Axis 2 for mag
l0 = ax0.plot(initx,inity,initx,inity,initx,inity)
l1 = ax1.plot(initx,inity,initx,inity,initx,inity)
l2 = ax2.plot(initx,inity,initx,inity,initx,inity)
# Style
ax0.set_title('Accelerometer (g)')
ax1.set_title('Gyroscope (dps)')
ax2.set_title('Magnetometer (uT)')
ax0.legend(l0,('x','y','z'),loc='upper left')
ax1.legend(l1,('x','y','z'),loc='upper left')
ax2.legend(l2,('x','y','z'),loc='upper left')

# Gets given imuData every iteration from dataGen()
def graphUpdate(imuData:imu.Imu, portHandle):
    # For each axis and then for each line
    for sensor,ax in enumerate(fig.get_axes()):
        for coord,l in enumerate(ax.get_lines()):

            data = l.get_ydata() # Get data from previous iteration
            data = np.append(data[1:], imuData[sensor][coord]) # Update data
            l.set_ydata(data) # Set data on graph

        # Rescale graph
        ax.relim()
        ax.autoscale()

    # Clear data waiting to be read, eliminating buildup
    portHandle.reset_input_buffer()

    # It would be good to be able to tell the chip to make a reading, rather than relying on a timer


# Function to yield imu data
def dataGen():
    while True:
        yield imu.getImuData(port)

# Start the graphing animation
# Interval is delay between next call of graphUpdate in ms
GRAPH_INTERVAL = 0 # If 0, Limited by either call for serial data (which has a timeout) or computer speed
ani = animation.FuncAnimation(fig=fig, func=partial(graphUpdate,portHandle=port), frames = dataGen, cache_frame_data=False, interval=GRAPH_INTERVAL)
plt.show()




