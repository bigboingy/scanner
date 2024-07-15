import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from functools import partial
import bt
import constants as cnst
import math

port = bt.getPortHandle()
x,y,z = [0],[0],[0]
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.set_xlim3d(-1,1) 
ax.set_ylim3d(-1,1) 
ax.set_zlim3d(-1,1) 
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
scat = ax.scatter(x,y,z,marker='o',c='r') # Plot origin as a red dot

port.write(bytes([cnst.REQ])) # First request


# Gets given imuData every iteration from dataGen()
def graphUpdate(data):

    if data and len(data['imu']):  # Only update if there's an imu value (length not zero)
        port.write(bytes([cnst.REQ])) # First, respond so that timeout doesn't occur

        # Update global coord lists
        for read in data['imu']:

            # Add to storage
            x.append(read.mag[0])
            y.append(read.mag[1])
            z.append(read.mag[2])

            heading = 0
            if read.mag[0] != 0: heading = math.atan2(read.mag[1],read.mag[0])*180/math.pi
            if heading < 0: heading = heading + 360
            print(heading)
            
        # Prepare new 'offsets'
        scat._offsets3d = (x,y,z)


unprocessedBytes = bytearray() # Store bytes in incomplete packet
# Function to yield bt data call
def dataGen(port,unprocessedBytes):
    while True:
        # Get data
        data = bt.read(port, unprocessedBytes, magCalOn=True)
        # Update unprocessedBytes if data is returned
        if data:
            unprocessedBytes = data['bytes']
        # Pass data to graphUpdate
        yield data


GRAPH_INTERVAL = 0 # If 0, Limited by either call for serial data (which has a timeout) or computer speed
ani = animation.FuncAnimation(fig=fig, func=graphUpdate, frames = partial(dataGen,port=port,unprocessedBytes=unprocessedBytes), cache_frame_data=False, interval=GRAPH_INTERVAL)
plt.show()