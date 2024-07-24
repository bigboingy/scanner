# Implementation of the "MAG.I.C.AL. METHODOLOGY"
# https://ieeexplore.ieee.org/abstract/document/8723161/
#
# Contains script to run for getting calibration readings and calculating factors
# Also has a function for applying the factors

import bt
import constants as cnst
import algorithms
import numpy as np
import math

# Function to apply calibration facrtors to a list of imu tuples (as returned by bt read)
def applyCalibration(imus:list,magCal:bool=True,accCal=True,magAlign:bool=True,gyroCal:bool=True):

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
        params = np.loadtxt('params_magCal')
        T_inv = params[:,0:3]
        h = params[:,[3]] # Enclose 3 in list to force col vector
        M = T_inv@(M-h) # Apply calibration before alignment

    if accCal:
        params = np.loadtxt('params_accCal')
        T_inv = params[:,0:3]
        h = params[:,[3]] # Enclose 3 in list to force col vector
        A = T_inv@(A-h) # Apply calibration before alignment

    if magAlign:
        R = np.loadtxt('params_magAlign')
        M = R@M

    if gyroCal:
        ...

    

    # Turn back into imu tuples
    calibrated_imus = []
    for i in range(N):
        imu = cnst.Imu(cnst.Cartesian(A[0,i],A[1,i],A[2,i]),cnst.Cartesian(G[0,i],G[1,i],G[2,i]),cnst.Cartesian(M[0,i],M[1,i],M[2,i]),
                       imus[i].temp, imus[i].time)
        calibrated_imus.append(imu)

    return calibrated_imus

    
    

# Calibration routine
if __name__ == "__main__":

    port = bt.getPortHandle()
    unprocessedBytes = bytearray() # Store bytes from incomplete packets, is updated by bt.read.
    # We should only be getting 1 imu at a time as singleRead=True

    # Function that takes a list of imu tuples and returns one imu tuple with their averaged data
    def imuAv(imus):
        n = len(imus)
        ax,ay,az,gx,gy,gz,mx,my,mz,temp,time = 0,0,0,0,0,0,0,0,0,0,0 # Sums for averaging
        for imu in imus:
            ax += imu.acc.x
            ay += imu.acc.y
            az += imu.acc.z
            gx += imu.gyro.x
            gy += imu.gyro.y
            gz += imu.gyro.z
            mx += imu.mag.x
            my += imu.mag.y
            mz += imu.mag.z
            temp += imu.temp
            time += imu.time # Time might be stuffed up if counter resets, but doesn't matter for this application
        averagedImu = cnst.Imu(cnst.Cartesian(ax/n,ay/n,az/n),cnst.Cartesian(gx/n,gy/n,gz/n),cnst.Cartesian(mx/n,my/n,mz/n),temp/n,time/n)
        return averagedImu

    # 1. On user input, grab bunches of IMU readings over bt to average, until NO_READS is satisfied
    NO_READS = 15 # How many datapoints
    STAT_AV = 25 # How many reads are taken at stationary datapoints, to be averaged
    imus = [] # Store IMU tuples here
    while len(imus) < NO_READS:
        inp = input("Press enter to take next reading.") # Blocking

        # Get readings
        bt.write(port,lidarOn=False,imuOn=True,count=STAT_AV)
        # Loop while we don't have all the requested imus yet
        imus_toAverage = []
        while len(imus_toAverage) < STAT_AV:
            imu = bt.read(port,unprocessedBytes)["imu"]
            if imu:
                imus_toAverage.extend(imu) # bt.read returns a list of tuples, so we extend rather than append

        # Average the imus and add to imus list
        avImu = imuAv(imus_toAverage)
        imus.append(avImu)
        print(f"Reading taken: {avImu}")
    
    # Fill arrays
    N = len(imus) # How many readings?
    A,G,M = np.empty((3,N)),np.empty((3,N)),np.empty((3,N))
    for i in range(N):
        A[:,i] = imus[i].acc.x,imus[i].acc.y,imus[i].acc.z
        G[:,i] = imus[i].gyro.x,imus[i].gyro.y,imus[i].gyro.z
        M[:,i] = imus[i].mag.x,imus[i].mag.y,imus[i].mag.z

    # 2. Calibrate magnetometer
    M_cal,Tm,hm = algorithms.magCalibrate(M)
    # Save T^-1 and h to file
    np.savetxt('params_magCal',np.hstack((np.linalg.inv(Tm),hm))) # Format is [Tm^-1 hm]

    # 3. Calibrate accelerometer
    A_cal,Ta,ha = algorithms.magCalibrate(A)
    # Save T^-1 and h to file
    np.savetxt('params_accCal',np.hstack((np.linalg.inv(Ta),ha))) # Format is [Ta^-1 ha]

    # 4. Align mag to acc
    R,delta = algorithms.magAlign(A_cal,M_cal)
    print(f"R = {R} and delta = {delta*180/math.pi}")
    # Save alignmnet matrix R
    np.savetxt('params_magAlign',R)
