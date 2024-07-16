import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import constants as cnst
from functools import partial
from bt import read

# Open port
port = serial.Serial(
    port="/dev/cu.HC-06", baudrate=115200, bytesize=8, timeout=5, stopbits=serial.STOPBITS_ONE
)
# First request
port.write(bytes([cnst.REQ]))

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

# Gets given bt every iteration from dataGen()
def graphUpdate(btData):

    if btData and len(btData['imu']):  # Only update if there's an imu value (length not zero)

        port.write(bytes([cnst.REQ])) # First, respond so that timeout doesn't occur

        # For each axis (sensor) and then for each line (coordinate x/y/z)
        for sensor,ax in enumerate(fig.get_axes()): # Sensor is the index
            for coord,l in enumerate(ax.get_lines()):  # Coord is the index

                newVals = np.empty(0)
                # Grab all the new values
                for read in btData['imu']:
                    newVals = np.append(newVals,read[sensor][coord]) # Access sensor and coord using index (0/1/2)

                oldData = l.get_ydata() # Get data from previous iteration

                # Join old and new data
                newData = np.concatenate((oldData,newVals))

                l.set_ydata(newData[-MAX_VALS:]) # Set data on graph, but only as much as MAX_VALS

            # Rescale graph
            ax.relim()
            ax.autoscale()

        for read in btData['imu']:
            print(read.time)

unprocessedBytes = bytearray() # Store bytes in incomplete packet
# Function to yield bt data call
def dataGen(port,unprocessedBytes):
    while True:
        # Get data
        data = read(port, unprocessedBytes, magCalOn=True)
        # Update unprocessedBytes if data is returned
        if data:
            unprocessedBytes = data['bytes']
        # Pass data to graphUpdate
        yield data

# Start the graphing animation
# Interval is delay between next call of graphUpdate in ms
GRAPH_INTERVAL = 100 # If 0, Limited by either call for serial data (which has a timeout) or computer speed. (ms)
ani = animation.FuncAnimation(fig=fig, func=graphUpdate, frames = partial(dataGen,port,unprocessedBytes), cache_frame_data=False, interval=GRAPH_INTERVAL)
plt.show()