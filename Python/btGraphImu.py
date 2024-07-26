import calibration
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from functools import partial
import bt

# Open port
port = bt.getPortHandle(port="/dev/cu.HC-05")

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

        # For each axis (sensor) and then for each line (coordinate x/y/z)
        for sensor,ax in enumerate(fig.get_axes()): # Sensor is the index
            for coord,l in enumerate(ax.get_lines()):  # Coord is the index

                newVals = np.empty(0)
                # Grab all the new values
                for read in btData:
                    newVals = np.append(newVals,read[sensor][coord]) # Access sensor and coord using index (0/1/2)

                oldData = l.get_ydata() # Get data from previous iteration

                # Join old and new data
                newData = np.concatenate((oldData,newVals))

                l.set_ydata(newData[-MAX_VALS:]) # Set data on graph, but only as much as MAX_VALS

            # Rescale graph
            ax.relim()
            ax.autoscale()

        # Debug
        for read in btData:
            #print(f'mag norm:{np.linalg.norm(read.mag,ord=2)}')
            #print(f'acc norm:{np.linalg.norm(read.acc,ord=2)}')
            print(read.time*1000)



bt.write(port,lidarOn=False,imuOn=True,count=-1) # First data request
unprocessedBytes = bytearray() # Store bytes from incomplete packets

# Function to yield bt data when a full imu is received
def dataGen(port,unprocessedBytes):
    while True: # Encase in infinite loop to keep yielding

        # Keep reading port until 1+ imus are found
        data = []
        while not data:
            # Get data
            data = bt.read(port, unprocessedBytes)["imu"]

        bt.write(port,lidarOn=False,imuOn=True,count=-1) # Respond to avoid timeout

        # Calibration
        calibrated_data = calibration.applyCalibration(data,accCal=True)
        yield calibrated_data # Pass data (imu tuple) to graphUpdate

# Start the graphing animation
# Interval is delay between next call of graphUpdate in ms
GRAPH_INTERVAL = 100 # (ms). On chip, timeout is at 150ms
ani = animation.FuncAnimation(fig=fig, func=graphUpdate, frames = partial(dataGen,port,unprocessedBytes), cache_frame_data=False, interval=GRAPH_INTERVAL)
plt.show()