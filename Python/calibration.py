# File for generating imu calibration data
# Uses algorithms in algorithms.py

import algorithms
import time
import asyncio
from ble import run, read
from config import Imu, Cartesian
import numpy as np

# Function to run through calibration routine
async def calibrate():

    # Settings
    NO_READS = 12 # How many datapoints
    AV = 20 # How many reads are taken at stationary datapoints, to be averaged
    ROT_TIME = 2 # How many seconds does rotation go for?
    DELAY = 3 # How many s to give user to position imu
    moving_reads = [] #[12,15,18] # Which reads are moving?

    # Storage
    static_imus = [] # Store static IMUs here, used for mag+acc cal and alignment, and gyro cal and alignment
    moving_imus = [] # Store moving IMUs here, a list of lists of imus in a rotation, used for gyro cal and alignment

    for i in range(1,NO_READS+1): # Start at 1 so that i matches current read
        startTime = time.time()

        # If the read is a rotation
        if i in moving_reads:
            ...

        # Static read
        else:
            while DELAY - (time.time() - startTime ) > 0:
                print(f"Static read in: {DELAY - (time.time() - startTime ):.2f} s")
                await asyncio.sleep(0.1) # Sleep value determines how often message is sent
            
            # After countdown finished
            data = await read() # We don't actually take new reads, just use the most recent ones
            # Get the average
            ax,ay,az,gx,gy,gz,mx,my,mz,temp,count = 0,0,0,0,0,0,0,0,0,0,0 # Sums for averaging
            for imu in data['imu'][-AV:]: # Sum. Would be good to add outlier detection
                ax += imu.acc.x; ay += imu.acc.y; az += imu.acc.z
                gx += imu.gyro.x; gy += imu.gyro.y; gz += imu.gyro.z
                mx += imu.mag.x; my += imu.mag.y; mz += imu.mag.z
                temp += imu.temp; count += imu.count
            av = Imu(Cartesian(ax/AV,ay/AV,az/AV),Cartesian(gx/AV,gy/AV,gz/AV),Cartesian(mx/AV,my/AV,mz/AV),
                     temp/AV,count/AV)
            static_imus.append(av) # Add to storage list
            print("Reading taken!")

    # Generate static arrays and arguments
    N = len(static_imus) # How many static readings?
    A,G,M = np.empty((3,N)),np.empty((3,N)),np.empty((3,N)) # Static reading arrays, 3xN
    for i in range(N):
        data = static_imus[i].extract()
        A[:,i:i+1] = data[:,0:1]
        G[:,i:i+1] = data[:,1:2]
        M[:,i:i+1] = data[:,2:3]

    # Calibrate magnetometer
    M_cal,Tm,hm = algorithms.magCalibrate(M)
    # Save T^-1 and h to file
    np.savetxt('params_magCal',np.hstack((np.linalg.inv(Tm),hm))) # Format is [Tm^-1 hm]

    # Calibrate accelerometer
    A_cal,Ta,ha = algorithms.magCalibrate(A)
    # Save T^-1 and h to file
    np.savetxt('params_accCal',np.hstack((np.linalg.inv(Ta),ha))) # Format is [Ta^-1 ha]

    # Align mag to acc
    R,delta = algorithms.magAlign(A_cal,M_cal)
    M_align = R@M_cal # Grab this for gyro cal
    print(f"R = {R} and delta = {delta*180/3.141592654}")
    # Save alignmnet matrix R
    np.savetxt('params_magAlign',R)

    # Galibrate and align gyro




# Script for generating calibration data
if __name__ == "__main__":

    run(calibrate())