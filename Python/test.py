import bt


unprocessedBytes = bytearray() # Store bytes from incomplete packets
port = bt.getPortHandle(port="/dev/cu.HC-05")


bt.write(port,False,True,31)
while 1:

    data = bt.read(port, unprocessedBytes)["imu"]



    if data:
        print(f'found {len(data)}')
        bt.write(port,False,True,0)


        
#         #bt.write(port,False,True,1)

import numpy as np
# a = (1,2,3)
# print(np.reshape(a,(3,1)))
moving_reads = [2,7,12] # Which reads are moving?
# map = [x+1 for x in range(15) if x not in moving_reads]
# print(map)

# a = list(range(15))
# for index in moving_reads:
#     a.insert(index,0)
# map = [x for i,x in enumerate(a) if i != len(a)-1 and a[i+1]==0]

# print(a)
# print(map)

# b = np.array(range(12))
# print(b[map])

# a = list(range(11))
# print(a)
# for rot in moving_reads:
#     a.insert(rot-1,-1)

# print(a)
# map = [x for i,x in enumerate(a) if x!=a[-1] and a[i+1]==-1]
# map = [x for i,x in enumerate(a) if x!=a[0] and a[i-1]==-1]

# print(map)

# print

import algorithms
import constants

a = np.arange(6).reshape(2,3)
print(a)
with np.nditer(a, op_flags=['readwrite']) as it:
   for x in it:
       print(x)
       print(x[...])
print(a)
    
print(algorithms.gyroCalibrate([np.loadtxt('sampleY0'),np.loadtxt('sampleY1'),np.loadtxt('sampleY2')],np.loadtxt('sampleA_surr'),np.loadtxt('sampleM_surr'),constants.SAMPLE_RATE,np.loadtxt('samplewStill')))

        