import serial
import time

# Function to get one set of data from TFmini
# In: handle of serial port
# Out: list with lidar data (int) or error (string)
def getLidarData(portHandle):
    # Loop to get one full set of TFmini data
    i = 0 # Counter showing which byte we are on
    lidarData = [] # List to store lidar data
    while 1:

        # First, grab serial data. Timeout is set to 2 seconds
        dataString = portHandle.read(size=1).hex() # Returns byte type, can be converted to hex string with .hex()
        if dataString == '':
            print('No data found!')
            continue
        data = int(dataString,16) # Convert data to an integer

        match i:
            # Header values
            case 0 | 1:
                if data == 0x59:
                    # Don't store value lidarData[i] = data
                    i+=1
                    continue

            # Checksum footer
            case 8:
                checksum = (0x59 + 0x59 + sum(lidarData)) & 0xFF
                if checksum != data:
                    # Don't store value lidarData[i] = data
                    print('Checksum failed!')
                    continue
                    
               
                i = 0 # Reset counter, regardless
                break # Escape loop

            # Data values
            case _:
                lidarData.append(data)
                i+=1


    # Merge low and high bits
    # Distance(mm), Strength, Temp
    # Strength "Represents the signal strength with the default value in the range of 0-65535"
    # Temp needs to be converted to C. C=T/8-256 Recommended to not exceed 60C
    formattedData = {'dist': lidarData[0] + (lidarData[1] << 8 ), 'strength': lidarData[2] + (lidarData[3] << 8 ),
                     'temp': (lidarData[4] + (lidarData[5] << 8))/8-256 }

    # Warn user if lidar is getting too hot
    if formattedData['temp'] >= 55:
        print('Warning: lidar is %.1f degrees C, maximum operating temperature is 60 degrees C'
              % formattedData['temp'])

    return formattedData

# Function to get average distance from a number of lidar reads
# In: function to read serial port for lidar data, handle for data port, number of values to average
# Out: averaged distance
def getAvDist(lidarDataReader, port, numVals):
    distances = [] # Store distances to be averaged
    while 1:
        # Get one set of lidar data
        lidarData = lidarDataReader(port)

        # Extract distance
        distance = lidarData['dist']
        if distance == 0:
                print('Signal strength outside working range!')
                continue # Go to next data read
        
        # Add to distances list
        distances.append(distance)

        # If numVals distances recorded, get average and return it
        numDistances = len(distances)
        if numDistances == numVals:
            avDist = sum(distances)/numDistances
            return(avDist)



# Open port
# timeout=2 means wait up to 2 seconds to return requested no. bytes specified in the read() function
port = serial.Serial(
    port="COM4", baudrate=115200, bytesize=8, timeout=5, stopbits=serial.STOPBITS_ONE
)

# Main loop
NUM_DIST_VALS_AV = 1

while 1:
    port.reset_input_buffer()
    # Get average distance
    start = time.time()
    distance = getAvDist(getLidarData, port, NUM_DIST_VALS_AV)
    print(time.time()-start)
    print("Distance: %.1fmm" % distance)
    time.sleep(1)
    