import bt
import time

unprocessedBytes = bytearray() # Store bytes from incomplete packets
port = bt.getPortHandle(port="/dev/tty.HC-05")



bt.write(port,True,True,count=-1)


while 1:

    data = []
    while not data:
        # Get data
        data = bt.read(port, unprocessedBytes)['imu']

    bt.write(port,lidarOn=True,imuOn=True,count=-1) # Respond to avoid timeout

    print(data[0].time)


    # data = bt.read(port, unprocessedBytes)
    # if data["imu"]:
    #         bt.write(port,True,True,-1)
    #         print(data)
    

# import algorithms
# import numpy as np
# H,h = algorithms.gyroCalibrate([np.loadtxt('sampleY0'),np.loadtxt('sampleY1'),np.loadtxt('sampleY2')],np.loadtxt('sampleA_surr'),np.loadtxt('sampleM_surr'),100,np.loadtxt('samplewStill'),degrees=False)
# print(H,h)
# np.savetxt('params_gyroCal',np.hstack((H,h))) # Format is [Tm^-1 hm]
