from bt import read
import constants as cnst
import serial
import time

# Open port
# timeout waits for return of requested no. bytes specified in read() function, and also in port opening
port = serial.Serial(
    port="/dev/cu.HC-06", baudrate=115200, bytesize=8, timeout=5, stopbits=serial.STOPBITS_ONE
)

# Stores processed data, a dictionary with lists containing named tuples
sensorData = {
    'lidar':[],
    'imu':[]
}

unprocessedBytes = bytearray() # Store bytes in incomplete packet

port.write(bytes([cnst.REQ])) # Make request for data
while 1:
   

    data = read(port,unprocessedBytes)
    if data:
        unprocessedBytes = data['bytes'] # Override bytes storage
        sensorData['lidar'].extend(data['lidar']) # Add new sensor data
        sensorData['imu'].extend(data['imu'])
        port.write(bytes([cnst.REQ])) # Respond so that timeout doesn't occur


        if len(sensorData['imu']) > 0:
            a = sensorData['imu'][-1]
            print(a.acc.z)

