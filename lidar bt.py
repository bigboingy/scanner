import serial
import time
from collections import namedtuple


# Values sent over bt to request data
LIDAR_REQ = 1<<0
IMU_REQ   = 1<<1

# Data structure for lidar data
Lidar = namedtuple('Lidar', 'dist str temp')



# Open port
# timeout waits for return of requested no. bytes specified in read() function, and also in port opening
port = serial.Serial(
    port="COM5", baudrate=115200, bytesize=8, timeout=5, stopbits=serial.STOPBITS_ONE
)



port.write(bytes([LIDAR_REQ]))

start = time.time()

PACKET_LENGTH = 9
serialBytes = bytearray()
lidarData = []
while 1:

    # if time.time()-start > 20:
    #     port.write(bytes([0x00]))
    #     print(len(lidarData))
    #     break

    # 1. Read all available serial data and place in serialBytes array
    if port.in_waiting > 0:
        serialBytes.extend(bytearray(port.read(port.in_waiting)))
        port.write(bytes([LIDAR_REQ])) # Respond so that timeout doesn't occur
    else:
        continue # Continue if no new data

    numBytes = len(serialBytes) # Get number of bytes to be processed

    # 2. Make sure serialBytes array starts with 0x59 0x59 frame header
    if numBytes>1:
        while (serialBytes[0] != 0x59 or serialBytes[1] != 0x59):
            serialBytes.pop(0)
            numBytes = len(serialBytes)
            if numBytes <=1 :
                break


    # 3. Sort data from serialBytes if there's enough, and process into lidarData
    if numBytes >= PACKET_LENGTH:
        #port.write(bytes([LIDAR_REQ]))
        # For each PACKET_LENGTH bytes, make a lidar tuple
        numPackets = numBytes//PACKET_LENGTH # Floor division
        for i in range(numPackets): # Each packet
            packet = serialBytes[i*PACKET_LENGTH:i*PACKET_LENGTH+PACKET_LENGTH]
            # Check the checksum
            checksum = sum(packet[:PACKET_LENGTH-1])&0xFF
            if checksum != packet[PACKET_LENGTH-1]:
                print('Checksum Failed!')
            
            # Make lidar tuple
            newLidar = Lidar(
                packet[2] + (packet[3]<<8), # Dist
                packet[4] + (packet[5]<<8), # Str
                (packet[6] + (packet[7]<<8))/8-256 # Temp
            )
            # Add to global array
            lidarData.append(newLidar)
            print(newLidar.dist)
        # Trim serialBytes, removing processed packets
        serialBytes = serialBytes[numPackets*PACKET_LENGTH:]

    