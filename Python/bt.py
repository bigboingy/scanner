# Imports constants
import constants as cnst
import time

# Function to convert single integer (base 10) into two's complement integer
# In: val, length: how many bytes are being convreted
# Out: two's complement of the integer
def twos(val: int, length: int) -> int:
    byte = val.to_bytes(length, byteorder='big', signed=False) # Convert to byte object
    return int.from_bytes(byte, byteorder='big', signed=True) # Use inbuilt 2s complement conversion


# Function to read and process serial data
# port: port handle, oldBytes: bytes that haven't been processed yet (bytearray)
# Returns false if no new bytes, otherwise returns dictionary with unprocessed data, lidar tuples (may be none), and imu tuples (may be none)
def read(port, oldBytes:bytearray) -> dict:

    # Structure to return
    result = {
        'bytes': bytearray(),
        'lidar':[], # Contains lidar tuples
        'imu':[] # Imu tuples
    }

    serialBytes = oldBytes # Get old data. This is a bytearray that stores bytes to process

    # 1. Read all available serial data and place in serialBytes array
    if port.in_waiting > 0:
        serialBytes.extend(bytearray(port.read(port.in_waiting)))
    else:
        return False # Return false if no new data waiting
    
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
            result['bytes'].extend(serialBytes[i:]) # Add unprocessed bytes to result
            break

        # Double frame header
        # Lidar frame header
        if serialBytes[i] == cnst.LIDAR_HEADER and serialBytes[i+1] == cnst.LIDAR_HEADER:
            # Are there enough bytes in front?
            if len(serialBytes[i:]) >= cnst.LIDAR_LENGTH:
                # Isolate packet
                packet = serialBytes[i:i+cnst.LIDAR_LENGTH]
                # Check checksum
                checksum = sum(packet[:-1])&0xFF
                if checksum != packet[-1]:
                    print('Checksum Failed!')
                # Make a lidar dataclass
                newLidar = cnst.Lidar(
                packet[2] + (packet[3]<<8), # Dist
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
                continue
            else:
                result['bytes'].extend(serialBytes[i:]) # Add unprocessed bytes to result
                break

        # IMU frame header
        if serialBytes[i] == cnst.IMU_HEADER and serialBytes[i+1] == cnst.IMU_HEADER:
            # Are there enough bytes in front?
            if len(serialBytes[i:]) >= cnst.IMU_LENGTH:
                # Isolate packet
                packet = serialBytes[i:i+cnst.IMU_LENGTH]
                # Check checksum
                checksum = sum(packet[:-1])&0xFF
                if checksum != packet[-1]:
                    print('Checksum Failed!')
                # Make an imu dataclass
                newImu = cnst.Imu(
                    # Acc x,y,z
                    cnst.Cartesian(twos((packet[2]<<8) + packet[3],2)/cnst.ACC_SCALE_FAC,
                            twos((packet[4]<<8) + packet[5],2)/cnst.ACC_SCALE_FAC,
                            twos((packet[6]<<8) + packet[7],2)/cnst.ACC_SCALE_FAC),
                    # Gyro x,y,z
                    cnst.Cartesian(twos((packet[8]<<8) + packet[9],2)/cnst.GYRO_SCALE_FAC,
                            twos((packet[10]<<8) + packet[11],2)/cnst.GYRO_SCALE_FAC,
                            twos((packet[12]<<8) + packet[13],2)/cnst.GYRO_SCALE_FAC),
                    # Mag x,y,z (low then high bit)
                    cnst.Cartesian(twos(packet[16] + (packet[17]<<8),2)/cnst.MAG_SCALE_FAC,
                            twos(packet[18] + (packet[19]<<8),2)/cnst.MAG_SCALE_FAC,
                            twos(packet[20] + (packet[21]<<8),2)/cnst.MAG_SCALE_FAC),
                    # Temperature
                    twos((packet[14]<<8) + packet[15],2)/cnst.TEMP_SCALE_FAC + 21,

                    # Timestamp
                    time.time()
                )
                # Push to sensorData
                result['imu'].append(newImu)
                # Skip packet indices
                skips = cnst.IMU_LENGTH-1
                continue
            else:
                result['bytes'].extend(serialBytes[i:]) # Add unprocessed bytes to result
                break
    
        # If we come to a value that's not a frame header, go to next index (doesn't tend to happen)
        # This could be if there's a random byte between packets
        continue

    return result