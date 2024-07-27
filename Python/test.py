# import bt
# import time

# unprocessedBytes = bytearray() # Store bytes from incomplete packets
# port = bt.getPortHandle(port="/dev/tty.HC-05")


# bt.write(port,False,True,18)
# while 1:

#     data = bt.read(port, unprocessedBytes)["imu"]
#     if data:
#         print(f'found {len(data)}')
#         bt.write(port,False,True,0)

#         # for read in data:
#         #     print(read.time)


        
#         #bt.write(port,False,True,1)

# import numpy as np
# a = (1,2,3)
# print(np.reshape(a,(3,1)))
# moving_reads = [2,7,12] # Which reads are moving?

# map = [x+1 for x in range(15) if x not in moving_reads]
# print(map)

import algorithms
import numpy as np
H,h = algorithms.gyroCalibrate([np.loadtxt('sampleY0'),np.loadtxt('sampleY1'),np.loadtxt('sampleY2')],np.loadtxt('sampleA_surr'),np.loadtxt('sampleM_surr'),100,np.loadtxt('samplewStill'),degrees=False)
print(H,h)
np.savetxt('params_gyroCal',np.hstack((H,h))) # Format is [Tm^-1 hm]


# def getRam(fbegin,fend,mbegin,mend):

#     # Construct column vectors
#     a1 = fbegin
#     a2 = np.cross(fbegin,mbegin,axis=0)/np.linalg.norm(np.cross(fbegin,mbegin,axis=0))
#     a3 = np.cross(fbegin,np.cross(fbegin,mbegin,axis=0),axis=0)/np.linalg.norm(np.cross(fbegin,mbegin,axis=0))
#     b1 = fend
#     b2 = np.cross(fend,mend,axis=0)/np.linalg.norm(np.cross(fend,mend,axis=0))
#     b3 = np.cross(fend,np.cross(fend,mend,axis=0),axis=0)/np.linalg.norm(np.cross(fend,mend,axis=0))
    
#     # Calculate rotation matrix
#     Ram = np.hstack((b1,b2,b3)) @ np.hstack((a1,a2,a3)).T
#     return Ram

# fbegin=np.array([[-0.3],[-0.3],[-1]])
# fend=np.array([[-0.3],[-0.3],[-1]])
# mbegin=np.array([[-0.1],[-0.1],[1]])
# mend=np.array([[-0.5],[-0.1],[0.95]])

# print(getRam(fbegin,fend,mbegin,mend))
