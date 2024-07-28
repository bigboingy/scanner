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
import time

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
        params = np.loadtxt('params_gyroCal')
        H = params[:,0:3]
        h =  h = params[:,[3]]
        G = H@(G-h)

    # Turn back into imu tuples
    calibrated_imus = []
    for i in range(N):
        imu = cnst.Imu(cnst.Cartesian(A[0,i],A[1,i],A[2,i]),cnst.Cartesian(G[0,i],G[1,i],G[2,i]),cnst.Cartesian(M[0,i],M[1,i],M[2,i]),
                       imus[i].temp, imus[i].time)
        calibrated_imus.append(imu)

    return calibrated_imus

    
    

# Calibration routine
if __name__ == "__main__":

    port = bt.getPortHandle(port="/dev/cu.HC-05")
    unprocessedBytes = bytearray() # Store bytes from incomplete packets, is updated by bt.read.

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
    
    # no is how many are being averaged
    def getAvImu(no):
        # Request readings to be averaged
        bt.write(port,lidarOn=False,imuOn=True,count=STAT_AV)
        startTime = time.time()
        TIMEOUT = 2 # Seconds, if one byte goes missing or something
        # Loop while we don't have all the requested imus yet
        imus_toAverage = []
        while len(imus_toAverage) < no and time.time()-startTime < TIMEOUT:
            imu = bt.read(port,unprocessedBytes)["imu"]
            print("Hold IMU still...")
            if imu:
                imus_toAverage.extend(imu) # bt.read returns a list of tuples, so we extend rather than append
                bt.write(port,lidarOn=False,imuOn=True,count=0) # Respond to avoid timeout

        # Average the imus
        avImu = imuAv(imus_toAverage)

        # Return
        return avImu


    # 1. On user input, grab bunches of IMU readings over bt to average, until NO_READS is satisfied
    # At certain reads, track IMU moving between two points instead.
    # Gyrocope still reading is taken in first read (must be still)
    
    NO_READS = 19 # How many datapoints
    STAT_AV = 20 # How many reads are taken at stationary datapoints, to be averaged
    ROT_TIME = 2 # How many seconds does rotation go for?
    moving_reads = []#[12,15,18] # Which reads are moving?
    static_imus = [] # Store static IMU tuples here, used for mag+acc cal and alignment, and gyro cal and alignment
    moving_imus = [] # Store moving IMUs here, a list of lists of imus in a rotation, used for gyro cal and alignment
    for i in range(1,NO_READS+1): # Go from 1 so that i matches what read we are up to

        # If the next read is a rotation, prepare for rotation
        if i+1 in moving_reads:
            input("Press enter to start rotation.") # Wait until user is ready
            imu = getAvImu(STAT_AV)
            static_imus.append(imu)
            print(f"Reading taken: {imu}")

        # Rotation
        elif i in moving_reads:
            # We don't wait, we go directly into rotation
            rot = [] # Put imus here
            start = time.time()
            bt.write(port,lidarOn=False,imuOn=True,count=-1) # Unlimited request
            # Will there be a lag here?
            while time.time()-start < ROT_TIME:
                print(f"Rotation time remaining: {round(ROT_TIME - (time.time()-start),2)}")
                # Read data and push to rot
                data = bt.read(port,unprocessedBytes)["imu"]
                if data:
                    rot.extend(data)
                    bt.write(port,lidarOn=False,imuOn=True,count=-1) # Maintain request
            print('Rotation finished')
            bt.write(port,lidarOn=False,imuOn=False,count=0) # Turn off output and flush inc data
            time.sleep(0.2)
            print(port.in_waiting)
            port.reset_input_buffer()
            moving_imus.append(rot) # Add list to list
            
        # End of rotation
        elif i-1 in moving_reads:
            imu = getAvImu(STAT_AV)
            static_imus.append(imu)
            print(f"Reading taken: {imu}")
            

        # Static read
        else:
            input("Press enter to take static reading.") # Wait until user is ready
            #print(port.in_waiting)
            imu = getAvImu(STAT_AV)
            static_imus.append(imu)
            print(f"Reading taken: {imu}")

    port.close() # We don't need any more readings

    # Generate static arrays and arguments
    Ns = len(static_imus) # How many static readings?
    A,G,M = np.empty((3,Ns)),np.empty((3,Ns)),np.empty((3,Ns)) # Static arrays
    for i in range(Ns):
        A[:,i] = static_imus[i].acc.x,static_imus[i].acc.y,static_imus[i].acc.z
        G[:,i] = static_imus[i].gyro.x,static_imus[i].gyro.y,static_imus[i].gyro.z
        M[:,i] = static_imus[i].mag.x,static_imus[i].mag.y,static_imus[i].mag.z
    
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
    M_align = R@M_cal # Grab this for gyro cal
    print(f"R = {R} and delta = {delta*180/math.pi}")
    # Save alignmnet matrix R
    np.savetxt('params_magAlign',R)

    # 5. Calibrate and align gyro
    # First setup variables
    if moving_reads:
        Y = [] # List with a list for each rotation
        for rot in moving_imus: # Each rotation list full of imus
            Nr = len(rot) # How many imus in this rotation
            Yrot = np.empty((3,Nr)) # List for this rotation
            for i in range(Nr): # Each imu index in the rotation
                Yrot[:,i] = rot[i].gyro.x, rot[i].gyro.y, rot[i].gyro.z
            Y.append(Yrot)
        # Isolate start/end of rotations from static arrays for gyroCalibrate
        indices = list(range(NO_READS-len(moving_reads))) # Prepare maps
        for rot_index in moving_reads:
            indices.insert(rot_index-1,-1)
        # Use -1s to get indices around the rotations
        map = [x for i,x in enumerate(indices) if (x!=indices[-1] and indices[i+1]==-1) or (x!=indices[0] and indices[i-1]==-1)]
        A_surr = A_cal[:,map] # Isolate surrounding acc and mag reads
        M_surr = M_align[:,map]
        # Others for gyroCalibrate
        freq = cnst.SAMPLE_RATE # Sample rate
        wStill = static_imus[0].gyro # Will be converted into column vec in algorithms

        # Calibrate!
        Hg,hg = algorithms.gyroCalibrate(Y,A_surr,M_surr,freq,wStill)
        print(f"Tg = {np.linalg.inv(Hg)} and hg = {hg}")
        # H (which is T^-1) and h to file
        np.savetxt('params_gyroCal',np.hstack((Hg,hg))) # Format is [Tm^-1 hm]
