import bt
import time

unprocessedBytes = bytearray() # Store bytes from incomplete packets
port = bt.getPortHandle(port="COM8")
time.sleep(2)

bt.write(port,False,True,18)
while 1:

    data = bt.read(port, unprocessedBytes)["imu"]
    if data:
        print(f'found {len(data)}')
        bt.write(port,False,True,0)

        # for read in data:
        #     print(read.time)


        
#         #bt.write(port,False,True,1)

# import numpy as np
# a = (1,2,3)
# print(np.reshape(a,(3,1)))
# moving_reads = [2,7,12] # Which reads are moving?

# map = [x+1 for x in range(15) if x not in moving_reads]
# print(map)