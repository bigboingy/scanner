import serial
from collections import namedtuple


# Controls - what is being requested?
LIDAR_ON = 0
IMU_ON = 1

# How many bytes are the packets? (including header and checksum)
LIDAR_LENGTH = 9
IMU_LENGTH = 24

# What are the header values?
LIDAR_HEADER = 0x59
IMU_HEADER = 0x58

# Values sent over bt to request data
LIDAR_REQ = 1<<0
IMU_REQ   = 1<<1
REQ = (LIDAR_ON*LIDAR_REQ) | (IMU_ON*IMU_REQ) # Set the request byte

# Scales for accelerometer and gyroscope. Can be 0,1,2,3. Used for obtaining sensor values
ACC_SCALE = 0
GYRO_SCALE = 0
# Scale factors for imu
accScaleFactors = {
    0: 16384, # +-2g
    1: 8192, # +-4g
    2: 4096, # +-8g
    3: 2048 # +-16g
}
gyroScaleFactors = {
    0: 131, # +-250dps
    1: 65.5, # +-500dps
    2: 32.8, # +-1000dps
    3: 16.4 # +-2000dps
}
accScaleFactor = accScaleFactors[ACC_SCALE] #LSB/g
gyroScaleFactor = gyroScaleFactors[GYRO_SCALE] #LSB/dps
magScaleFactor = 1/0.15 #LSB/microTesla
tempScaleFactor = 333.87 #LSB/degreeC

# Data structures for lidar and imu
Lidar = namedtuple('Lidar', 'dist str temp')
Cartesian = namedtuple('Cartesian', 'x y z')
Imu = namedtuple('Imu', 'acc gyro mag temp')

# Function to convert single integer (base 10) into two's complement integer
# In: val, length: how many bytes are being convreted
# Out: two's complement of the integer
def twos(val: int, length: int) -> int:
    byte = val.to_bytes(length, signed=False) # Convert to byte object
    return int.from_bytes(byte, signed = True) # Use inbuilt 2s complement conversion

# Open port
# timeout waits for return of requested no. bytes specified in read() function, and also in port opening
port = serial.Serial(
    port="/dev/cu.HC-06", baudrate=115200, bytesize=8, timeout=5, stopbits=serial.STOPBITS_ONE
)
port.write(bytes([REQ])) # Make request for data

#PACKET_LENGTH = LIDAR_ON*LIDAR_LENGTH + IMU_ON*IMU_LENGTH
serialBytes = bytearray() # Stores incoming bytes
sensorData = { # Stores processed data, a dictionary with lists containing named tuples
    'lidar':[],
    'imu':[]
}
while 1:

    # 1. Read all available serial data and place in serialBytes array
    if port.in_waiting > 0:
        serialBytes.extend(bytearray(port.read(port.in_waiting)))
        port.write(bytes([REQ])) # Respond so that timeout doesn't occur
    else:
        continue # Continue if no new data

    numBytes = len(serialBytes) # Get number of bytes to be processed

    # 2. Loop through bytes to find double frame headers
    skips = 0 # How many bytes can we skip past? (they have been read)
    for i in range(numBytes):

        # Skip this index if byte already processed
        if skips != 0:
            skips-=1
            continue

        # When we come to values we can have:
        # Double frame header with full packet - process, then skip the packet indices
        # Double frame header w/o full packet - leave, clear previous packets, wait for more data
        # Not enough bytes to left to assess double frame header - leave, wait for more data
        # Random byte - continue to next index

        # Break if not enough bytes to read next byte
        if i+1 >= numBytes :
            serialBytes = serialBytes[i:] # Clear processed bytes
            break

        # Double frame header
        # Lidar frame header
        if serialBytes[i] == LIDAR_HEADER and serialBytes[i+1] == LIDAR_HEADER:
            # Are there enough bytes in front?
            if len(serialBytes[i:]) >= LIDAR_LENGTH:
                # Isolate packet
                packet = serialBytes[i:i+LIDAR_LENGTH]
                # Check checksum
                checksum = sum(packet[:-1])&0xFF
                if checksum != packet[-1]:
                    print('Checksum Failed!')
                # Make a lidar tuple
                newLidar = Lidar(
                packet[2] + (packet[3]<<8), # Dist
                packet[4] + (packet[5]<<8), # Str
                (packet[6] + (packet[7]<<8))/8-256 # Temp
                )
                # Warn user if lidar is getting too hot
                if newLidar.temp >= 55:
                    print('Warning: lidar is %.0f degrees C, maximum operating temperature is 60 degrees C'
                        % newLidar.temp)
                # Push to sensorData
                sensorData['lidar'].append(newLidar)
                print(newLidar.dist)
                # Skip next 8 values
                skips = LIDAR_LENGTH-1
                continue
            else:
                serialBytes = serialBytes[i:] # Clear processed bytes
                break # Wait for next read if not enough data

        # IMU frame header
        if serialBytes[i] == IMU_HEADER and serialBytes[i+1] == IMU_HEADER:
            # Are there enough bytes in front?
            if len(serialBytes[i:]) >= IMU_LENGTH:
                # Isolate packet
                packet = serialBytes[i:i+IMU_LENGTH]
                # Check checksum
                checksum = sum(packet[:-1])&0xFF
                if checksum != packet[-1]:
                    print('Checksum Failed!')
                # Make an imu tuple
                newImu = Imu(
                    # Acc x,y,z
                    Cartesian(twos((packet[2]<<8) + packet[3],2)/accScaleFactor,
                            twos((packet[4]<<8) + packet[5],2)/accScaleFactor,
                            twos((packet[6]<<8) + packet[7],2)/accScaleFactor),
                    # Gyro x,y,z
                    Cartesian(twos((packet[8]<<8) + packet[9],2)/gyroScaleFactor,
                            twos((packet[10]<<8) + packet[11],2)/gyroScaleFactor,
                            twos((packet[12]<<8) + packet[13],2)/gyroScaleFactor),
                    # Mag x,y,z (low then high bit)
                    Cartesian(twos(packet[16] + (packet[17]<<8),2)/magScaleFactor,
                            twos(packet[18] + (packet[19]<<8),2)/magScaleFactor,
                            twos(packet[20] + (packet[21]<<8),2)/magScaleFactor),
                    # Temperature
                    twos((packet[14]<<8) + packet[15],2)/tempScaleFactor + 21
                )
                # Push to sensorData
                sensorData['imu'].append(newImu)
                print(newImu.acc.z)
                # Skip packet indices values
                skips = IMU_LENGTH-1
                continue
            else: # If not enough data, wait for more data
                serialBytes = serialBytes[i:] # Clear processed bytes
                break

        # If we come to a value that's not a frame header, go to next index (doesn't tend to happen)
        # This could be if there's a random byte between packets
        continue

        



    