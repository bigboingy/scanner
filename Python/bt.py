# Imports constants
import constants as cnst
import serial
import time

# Function to convert single integer (base 10) into two's complement integer
# In: val, length: how many bytes are being converted
# Out: two's complement of the integer
def twos(val: int, length: int) -> int:
    byte = val.to_bytes(length, byteorder='big', signed=False) # Convert to byte object, big endian
    return int.from_bytes(byte, byteorder='big', signed=True) # Use inbuilt 2s complement conversion


# Function to open port, returning port handle
def getPortHandle(port = "/dev/cu.HC-05", baud = 115200, timeout = 5):
    handle = serial.Serial(
        port=port, baudrate=baud, bytesize=8, timeout=timeout, stopbits=serial.STOPBITS_ONE)

    time.sleep(3) # Give time for hc-05 to be ready!!!

    return handle

# Function for formatting and sending bt requests
# Count is how many reads to add to count before output stops
# Count defaults to 0 which doesn't set any new reads, but refreshes timeout
def write(port,lidarOn:bool,imuOn:bool,count:int=0):

    # Continuous read if count is -1
    if count == -1:
        cont = 1
        count = 0 # Set count to 0 now
    else: cont = 0

    # Make request byte
    request = bytes([   cnst.COUNT_SHIFT(count) |
                        cont*cnst.CONT_MODE |
                        imuOn*cnst.IMU_ON |
                        lidarOn*cnst.LIDAR_ON ]) # Need to enclose in list to specify byte
    # Send request
    port.write(request)

# Function to read and process serial data
# port: port handle, oldBytes: bytes that haven't been processed yet (bytearray)
# Returns false if no new bytes, otherwise returns dictionary list of lidar tuples (may be none) and imu tuples (may be none)
def read(port, bytesArray:bytearray) -> dict:

    # Structure to return
    result = {
        'lidar':[], # Contains lidar tuples
        'imu':[] # Imu tuples
    }

    # 1. Read all available serial data and place in serialBytes array
    if port.in_waiting > 0:
        bytesArray.extend(bytearray(port.read(port.in_waiting))) # Extends bytes object that was passed in
    else:
        return result # Return empty result if no new data waiting
    
    numBytes = len(bytesArray) # Get number of bytes to be processed
    
    # 2. Loop through bytes to find double frame headers
    skips = 0 # How many bytes can we skip past? (they have been read)
    toDelete = 0 # How many bytes have we processed and can now delete?
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
            break

        # Double frame header
        # Lidar frame header
        if bytesArray[i] == cnst.LIDAR_HEADER and bytesArray[i+1] == cnst.LIDAR_HEADER:
            # Are there enough bytes in front?
            if len(bytesArray[i:]) >= cnst.LIDAR_LENGTH:
                # Isolate packet
                packet = bytesArray[i:i+cnst.LIDAR_LENGTH]
                # Check checksum
                checksum = sum(packet[:-1])&0xFF
                if checksum != packet[-1]:
                    print('Lidar Checksum Failed!')
                    continue # Try next bit
                # Make a lidar dataclass
                newLidar = cnst.Lidar(
                (packet[2] + (packet[3]<<8))/1000, # Dist, converted to m
                packet[4] + (packet[5]<<8), # Str
                (packet[6] + (packet[7]<<8))/8-256 # Temp
                )
                # Warn user if lidar is getting too hot
                if newLidar.temp >= 55:
                    print('Warning: lidar is %.0f degrees C, maximum operating temperature is 60 degrees C'
                        % newLidar.temp)
                # Push to result
                result['lidar'].append(newLidar)
                # Skip next 8 values
                skips = cnst.LIDAR_LENGTH-1
                toDelete += cnst.LIDAR_LENGTH
                continue
            else: # Not enough bytes
                break 

        # IMU frame header
        if bytesArray[i] == cnst.IMU_HEADER and bytesArray[i+1] == cnst.IMU_HEADER:
            # Are there enough bytes in front?
            if len(bytesArray[i:]) >= cnst.IMU_LENGTH:
                # Isolate packet
                packet = bytesArray[i:i+cnst.IMU_LENGTH]
                # Check checksum
                checksum = sum(packet[:-1])&0xFF
                if checksum != packet[-1]:
                    print('IMU checksum Failed!')
                    continue # Try next bit

                # Check magnetometer status 2 register for magnetic sensor overflow
                if packet[22] & 0x08:
                    print('Magnetic sensor overflow occurred')
                    continue # Try next bit

                # Make an imu tuple
                newImu = cnst.Imu(
                    # Acc x,y,z. (big endian) Negatives added experimentally to make right handed coord sys
                    cnst.Cartesian(twos((packet[2]<<8) | packet[3],2)/cnst.ACC_SCALE_FAC*-1,
                            twos((packet[4]<<8) | packet[5],2)/cnst.ACC_SCALE_FAC*-1,
                            twos((packet[6]<<8) | packet[7],2)/cnst.ACC_SCALE_FAC*-1),
                    # Gyro x,y,z. (big endian)
                    cnst.Cartesian(twos((packet[8]<<8) | packet[9],2)/cnst.GYRO_SCALE_FAC,
                            twos((packet[10]<<8) | packet[11],2)/cnst.GYRO_SCALE_FAC,
                            twos((packet[12]<<8) | packet[13],2)/cnst.GYRO_SCALE_FAC),
                    # Mag x,y,z (read as low then high bit, little endian)
                    cnst.Cartesian(twos(packet[16] | (packet[17]<<8),2)/cnst.MAG_SCALE_FAC,
                       twos(packet[18] | (packet[19]<<8),2)/cnst.MAG_SCALE_FAC*-1,
                       twos(packet[20] | (packet[21]<<8),2)/cnst.MAG_SCALE_FAC*-1),
                    # Temperature
                    twos((packet[14]<<8) | packet[15],2)/cnst.TEMP_SCALE_FAC + 21,
                    # Timestamp
                    (packet[23]<<8 | packet[24])/cnst.DATATIMER_FREQ # Convert to seconds
                )
                # Push to sensorData
                result['imu'].append(newImu)
                # Skip packet indices
                skips = cnst.IMU_LENGTH-1
                toDelete += cnst.IMU_LENGTH
                continue
            else:
                break # Not enough bytes
    
        # If we come to a value that's not a frame header, go to next index (doesn't tend to happen)
        # This could be the first bytes that come in, or if there's a random byte between packets
        toDelete += 1 # Delete this byte
        continue

    # 3. Trim bytes that we have looked at, and return result
    del bytesArray[0:toDelete]

    return result