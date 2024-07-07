import imu
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from functools import partial

port = imu.getPortHandle()
x,y,z = [0],[0],[0]
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.set_xlim3d(-50,50) 
ax.set_ylim3d(-50,50) 
ax.set_zlim3d(-50,50) 
scat = ax.scatter(x,y,z,marker='o',c='r') # Plot origin as a red dot




# Gets given imuData every iteration from dataGen()
def graphUpdate(imuData:imu.Imu, portHandle):

    # Update global coord lists
    x.append(imuData.mag.x)
    y.append(imuData.mag.y)
    z.append(imuData.mag.z)

    # Prepare new 'offsets'
    scat._offsets3d = (x,y,z)

    # Clear data waiting to be read, eliminating buildup
    portHandle.reset_input_buffer()

# Function to yield imu data
def dataGen():
    while True:
        yield imu.getImuData(port)



GRAPH_INTERVAL = 0 # If 0, Limited by either call for serial data (which has a timeout) or computer speed
ani = animation.FuncAnimation(fig=fig, func=partial(graphUpdate,portHandle=port), frames = dataGen, cache_frame_data=False, interval=GRAPH_INTERVAL)
plt.show()