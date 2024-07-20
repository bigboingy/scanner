# Implementation of the "MAG.I.C.AL. METHODOLOGY"
# https://ieeexplore.ieee.org/abstract/document/8723161/
#
# Contains script to run for getting calibration readings and calculating factors
# Also has a function for applying the factors

import bt
import constants as cnst
import algorithms
import numpy as np
import time
port = bt.getPortHandle()


# Function to apply calibration facrtors to a list of imu tuples (as returned by bt read)
def applyCalibration(imus:list,magCal:bool=True,magAlign:bool=True):

    N = len(imus) # How many readings?

    # Make blank arrays to fill with readings
    A,G,M = np.empty((3,N)),np.empty((3,N)),np.empty((3,N))
    # Fill arrays
    for i in range(N):
        A[:,i] = imus[i].acc.x,imus[i].acc.y,imus[i].acc.z
        G[:,i] = imus[i].gyro.x,imus[i].gyro.y,imus[i].gyro.z
        M[:,i] = imus[i].mag.x,imus[i].mag.y,imus[i].mag.z

    # Magnetometer calibration
    if magCal:
        params = np.loadtxt('param_magCal')
        T_inv = params[:,0:3]
        h = params[:,[3]] # Enclose 3 in list to force col vector
        M = T_inv@(M-h) # Apply calibration before alignment

    # Turn back into imu tuples
    calibrated_imus = []
    for i in range(N):
        imu = cnst.Imu(cnst.Cartesian(A[0,i],A[1,i],A[2,i]),cnst.Cartesian(G[0,i],G[1,i],G[2,i]),cnst.Cartesian(M[0,i],M[1,i],M[2,i]),
                       imus[i].temp, imus[i].time)
        calibrated_imus.append(imu)

    return calibrated_imus

    
    

# Calibration routine
if __name__ == "__main__":

    # 1. On user input, grab one IMU reading over bt, until NO_READS is satisfied
    NO_READS = 15 # How many datapoints
    imus = [] # Store IMU tuples here
    while len(imus) < NO_READS:
        inp = input("Press enter to take next reading.") # Blocking
        # Get one reading
        port.reset_input_buffer() # Remove bytes received from last read before timeout occured
        unprocessedBytes = bytearray() # Store bytes from incomplete packets, is updated by bt.read
        port.write(bytes([cnst.REQ])) # Make request for data
        
        # Loop while there's no imu yet
        imu = []
        while not imu:
            imu = bt.read(port,unprocessedBytes)["imu"]
        imus.append(imu[0]) # Take first imu received
        print(f"Reading taken: {imu}")

    # 2. Calibrate magnetometer
    # Prepare Y
    Y = [[],[],[]] # x,y,z
    for imu in imus:
        Y[0].append(imu.mag.x)
        Y[1].append(imu.mag.y)
        Y[2].append(imu.mag.z)
    # Run algorithm
    M,T,h = algorithms.magCalibrate(Y)
    # Save T^-1 and h to file
    np.savetxt('param_magCal',np.hstack((np.linalg.inv(T),h))) # Format is [T^-1 h]
    print(M,T,h)
    
