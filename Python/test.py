# import bt


# unprocessedBytes = bytearray() # Store bytes from incomplete packets
# port = bt.getPortHandle(port="/dev/cu.HC-06")


# bt.write(port,False,True,31)
# while 1:

#     if port.in_waiting:
#         print(port.in_waiting)

#     data = bt.read(port, unprocessedBytes)["imu"]



#     if data:
#         print(f'found {len(data)}')
#         bt.write(port,False,True,0)


        
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

a = list(range(11))
print(a)
for rot in moving_reads:
    a.insert(rot-1,-1)

print(a)
map = [x for i,x in enumerate(a) if x!=a[-1] and a[i+1]==-1]
map = [x for i,x in enumerate(a) if x!=a[0] and a[i-1]==-1]

print(map)

print