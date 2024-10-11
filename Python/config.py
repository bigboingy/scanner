from dataclasses import dataclass
import numpy as np

# Packet lengths
BYTES_LIDAR = 6
BYTES_IMU   = 22

# Frequencies requency (Hz)
FREQ_LIDAR = 100 # Only used to estimate fusion recovery period
FREQ_TIMER = 10**6 # 1 MHz

# Scales for accelerometer and gyroscope. Can be 0,1,2,3
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
ACC_SCALE_FAC = accScaleFactors[ACC_SCALE] # LSB/g
GYRO_SCALE_FAC = gyroScaleFactors[GYRO_SCALE] # LSB/dps
MAG_SCALE_FAC = 1/0.15 # LSB/microTesla
TEMP_SCALE_FAC = 333.87 # LSB/degreeC

# Function to convert integer into 16 bit two's complement integer
# Params: val, the 16 bit value to be converted
# Return: two's complement of the integer
def twos16(val: int) -> int:
    if val & (1<<15):
        return -((~val & 0xFFFF) + 1) # If negative, invert bits and add 1. Mask to make int act like 16 bit
    return val

# Geometric variables
direc = np.array([[0],[0],[-1]]) # Which way the lidar faces relative to the imu coordinates
r = np.loadtxt("params_r").item() # Radius of rotation. The origin lies at the centre of this radius

# Dataclasses
class Lidar:
    dist: int = 0
    str: int = 0
    temp: int = 0
    def populate(self,bytes_lidar): # Populate using 6 bytes in order
        self.dist = (bytes_lidar[0] | bytes_lidar[1]<<8) / 1000 # Convert to metres

        self.str = bytes_lidar[2] | bytes_lidar[3]<<8
        # If str<100 or str=65535 then detection is unreliable and dist is set to 0
        if self.str < 100 or self.str == 65535:
            print(f'Lidar strength = {self.str}, distance has been set to 0')
            assert self.dist == 0

        self.temp = (bytes_lidar[4] | bytes_lidar[5]<<8)/8 - 256
        # Temp warning
        if self.temp > 57: print(f"Lidar temperature is {self.temp}, max is 60")


@dataclass
class Cartesian:
    x: int = 0
    y: int = 0
    z: int = 0

@dataclass
class Imu:
    acc: Cartesian = Cartesian()
    gyro: Cartesian = Cartesian()
    mag: Cartesian = Cartesian()
    temp: int = 0
    count: int = 0
    def populate(self,bytes_imu):
        self.acc = Cartesian(-twos16(bytes_imu[0] << 8 | bytes_imu[1])/ACC_SCALE_FAC, # Acc x, inverted to match datasheet
                             -twos16(bytes_imu[2] << 8 | bytes_imu[3])/ACC_SCALE_FAC, # Acc y
                             -twos16(bytes_imu[4] << 8 | bytes_imu[5])/ACC_SCALE_FAC) # Acc z
        
        self.gyro = Cartesian(twos16(bytes_imu[6] << 8 | bytes_imu[7])/GYRO_SCALE_FAC,   # Gyro x
                              twos16(bytes_imu[8] << 8 | bytes_imu[9])/GYRO_SCALE_FAC,   # Gyro y
                              twos16(bytes_imu[10] << 8 | bytes_imu[11])/GYRO_SCALE_FAC) # Gyro z
        
        # TEMP_degC = ((TEMP_OUT – RoomTemp_Offset)/Temp_Sensitivity) + 21degC
        self.temp = ( twos16(bytes_imu[12] << 8 | bytes_imu[13]) - 21 ) / TEMP_SCALE_FAC + 21

        self.mag = Cartesian(twos16(bytes_imu[14] | bytes_imu[15] << 8)/MAG_SCALE_FAC, # Mag x
                             -twos16(bytes_imu[16] | bytes_imu[17] << 8)/MAG_SCALE_FAC, # Mag y, inverted (indicated by datasheet)
                             -twos16(bytes_imu[18] | bytes_imu[19] << 8)/MAG_SCALE_FAC) # Mag z, inverted

        self.count = (bytes_imu[20] << 8 | bytes_imu[21])/FREQ_TIMER # Count (s)
    
    # Extract data as 3x3 list. Rows are coords and cols are sensors
    def extract(self):
    
        return np.array([[self.acc.x, self.gyro.x, self.mag.x],
                         [self.acc.y, self.gyro.y, self.mag.y],
                         [self.acc.z, self.gyro.z, self.mag.z]])
    
    # Method for calibrating the imu's data
    def calibrate(self,magCal=False,accCal=False,magAlign=False,gyroCal=False):

        # Get np arrays of current data
        data = self.extract()
        M = data[:,2:3] # Get np column arrays of current data
        A = data[:,0:1]
        G = data[:,1:2]

        # Apply calibrations
        if magCal:
            params = np.loadtxt('params_magCal')
            T_inv = params[:,0:3]
            h = params[:,3:4]
            M = T_inv@(M-h)

        if accCal:
            params = np.loadtxt('params_accCal')
            T_inv = params[:,0:3]
            h = params[:,3:4] # Enclose 3 in list to force col vector
            A = T_inv@(A-h)

        if magAlign: # This must come after calibrations
            R = np.loadtxt('params_magAlign')
            M = R@M

        if gyroCal:
            pass

        # Update data
        self.mag.x = M[0,0]; self.mag.y = M[1,0]; self.mag.z = M[2,0]
        self.acc.x = A[0,0]; self.acc.y = A[1,0]; self.acc.z = A[2,0]
        self.gyro.x = G[0,0]; self.gyro.y = G[1,0]; self.gyro.z = G[2,0]

        return self



if __name__ == "__main__":
    ...