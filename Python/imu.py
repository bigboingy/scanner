import serial
from collections import namedtuple

# Functions for receiving and sorting data from icm20948

# Data structures
Cartesian = namedtuple('Cartesian', 'x y z')
Imu = namedtuple('Imu', 'acc gyro mag')

# Scales for accelerometer and gyroscope. Can be 0,1,2,3. Used for obtaining sensor values
ACC_SCALE = 0
GYRO_SCALE = 0

# Function to open port, returning port handle
# timeout=2 means wait up to 2 seconds to return requested no. bytes specified in the read() function
def getPortHandle(port = "COM4", baud = 115200, timeout = 1):
    return serial.Serial(
        port=port, baudrate=baud, bytesize=8, timeout=timeout, stopbits=serial.STOPBITS_ONE
    )

# Converts single integer (base 10) into two's complement integer
# In: val, length: how many bytes are being convreted
# Out: two's complement of the integer
def twos(val: int, length: int) -> int:
    byte = val.to_bytes(length, signed=False) # Convert to byte object
    return int.from_bytes(byte, signed = True) # Use inbuilt 2s complement conversion

# Function to get one set of data from icm-20948
# In: handle of serial port, scales being used with acc and gyro
# Out: Imu namedtuple
# Data aquisition is similar to lidar, with only length of data and header values different
# Could integrate both functions, checking for header value
def getImuData(portHandle) -> Imu:

    # First, find what scale factor we need, depending on programmed settings
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

    # Loop to get one full set of TFmini data
    i = 0 # Counter showing which byte we are on
    imuData = [] # List to store imu data
    while 1:

        # First, grab serial data. Timeout is set to 2 seconds
        dataString = portHandle.read(size=1).hex() # Returns byte type, can be converted to hex string with .hex()
        if dataString == '': #If no data, reset i and try again
            print('No data found!')
            i=0
            continue
        data = int(dataString,16) # Convert to base 10 integer

        match i:
            # Header values
            case 0 | 1:
                if data == 0x58:
                    # Don't store value imuData[i] = data
                    i+=1
                    continue

            # Checksum footer
            # It would be better to send how many data values are expected, and then use that number to calculate this
            case 23:
                i = 0 # Reset counter, regardless
                checksum = (0x58 + 0x58 + sum(imuData)) & 0xFF
                if checksum != data:
                    # Don't store value lidarData[i] = data
                    # If checksum fails, reset i and try again
                    print('Checksum failed!')
                    continue

                # Escape loop if checksum is passed
                break 

            # Data values
            case _:
                imuData.append(data)
                i+=1

    # Merge low and high bits
    # Divide by scale factors to get values in g and dps
    formattedData = Imu(
        # Acc x,y,z
        Cartesian(twos((imuData[0]<<8) + imuData[1],2)/accScaleFactor,
                  twos((imuData[2]<<8) + imuData[3],2)/accScaleFactor,
                  twos((imuData[4]<<8) + imuData[5],2)/accScaleFactor),
        # Gyro x,y,z
        Cartesian(twos((imuData[6]<<8) + imuData[7],2)/gyroScaleFactor,
                  twos((imuData[8]<<8) + imuData[9],2)/gyroScaleFactor,
                  twos((imuData[10]<<8) + imuData[11],2)/gyroScaleFactor),
        # Mag x,y,z (low then high bit)
        Cartesian(twos(imuData[14] + (imuData[15]<<8),2)/magScaleFactor,
                  twos(imuData[16] + (imuData[17]<<8),2)/magScaleFactor,
                  twos(imuData[18] + (imuData[19]<<8),2)/magScaleFactor)
    )

    # Temperature data is also sent, but not used at this stage
    temperature = twos((imuData[12]<<8) + imuData[13],2)/tempScaleFactor + 21
    if temperature>50:
        print('IMU temperature exceeding 50C')

    return formattedData

