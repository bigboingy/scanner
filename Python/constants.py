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
ACC_SCALE_FAC = accScaleFactors[ACC_SCALE] #LSB/g
GYRO_SCALE_FAC = gyroScaleFactors[GYRO_SCALE] #LSB/dps
MAG_SCALE_FAC = 1/0.15 #LSB/microTesla
TEMP_SCALE_FAC = 333.87 #LSB/degreeC

# Data structures for lidar and imu
from collections import namedtuple
Lidar = namedtuple('Lidar', 'dist str temp')
Cartesian = namedtuple('Cartesian', 'x y z')
Imu = namedtuple('Imu', 'acc gyro mag temp')