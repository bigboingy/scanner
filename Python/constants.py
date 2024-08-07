import numpy as np

# DataTimer frequency (Hz)
DATATIMER_FREQ = 100*1000 # One counter is a 100th of a ms

# Sample rate (Hz) (approx)
SAMPLE_RATE = 100

# How many bytes are the packets? (including header and checksum)
LIDAR_LENGTH = 9
IMU_LENGTH = 26

# What are the header values?
LIDAR_HEADER = 0x59
IMU_HEADER = 0x58

# Values sent over bt to request data
LIDAR_ON   = 1 << 0
IMU_ON     = 1 << 1
CONT_MODE   = 1 << 2
COUNT_SHIFT = lambda x: x<<3

# Scales for accelerometer and gyroscope. Can be 0,1,2,3. Used for obtaining sensor values
ACC_SCALE = 1
GYRO_SCALE = 2

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

# Data structures for lidar and imu. Timestamp added by python (not sent over bt)
from collections import namedtuple
Lidar = namedtuple('Lidar', 'dist str temp')
Cartesian = namedtuple('Cartesian', 'x y z')
Imu = namedtuple('Imu', 'acc gyro mag temp time')

# Lidar imu fusion constants, using open3d
IMU_DIREC = [1,0,0] # Unit vector of IMU direction that's facing outwards from centre of rotation (imu frame)
LIDAR_DIREC = [1,0,0] # Unit vector of lidar direction (imu frame)
RADIUS = 0.3 # How many m is the rotation radius to the lidar?
IMU_VEC_TO_O3D = lambda x: (np.array([[0,1,0],[0,0,1],[1,0,0]]) @ x).T # Change coordinate system and transpose (o3d uses row vecs)
IMU_MAT_TO_O3D = lambda x: (x @ np.array([[0,0,1],[1,0,0],[0,1,0]])) # Shift columns

# from dataclasses import dataclass
# @dataclass
# class Lidar:
#     dist: int
#     str: int
#     temp: int
#     timestamp: int

# @dataclass
# class Cartesian:
#     x: int
#     y: int
#     z: int

# @dataclass
# class Imu:
#     acc: Cartesian
#     gyro: Cartesian
#     mag: Cartesian
#     temp: int
#     timestamp: int

