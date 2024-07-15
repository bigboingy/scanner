import numpy as np

# Takes N magnetometer reading vectors and returns calibrated values, as well as T and h calibration parameters
# Based on algorithm presented in https://ieeexplore.ieee.org/abstract/document/8723161
# args:
# Y (array_like), a 3xN matrix of magnetic field strength readings (should be more than 12).
# tol (int), minimum % difference between iterations to consider converged
def calibrate(Y,tol_percent=0.001):
    # Convert Y to np array if it's not already
    if not isinstance(Y,np.ndarray):
        Y = np.array(Y)

    # Get number of readings
    N = np.shape(Y)[1] 

    # 1. Initialise M (true magnetic field vector) as normalised Y
    M = Y/np.linalg.norm(Y,axis=0)

    # Initialise lambda, set to ||T|| in first loop
    lmbd = 0
    # Initialise J
    J=None

    # Iterate
    while 1:

        # Make G matrix
        G = np.vstack((M,np.ones((1,N))))

        # 2. Solve for L using least-squares, L=YGt(GGt)^-1
        # Get G transpose
        Gt = np.transpose(G)
        # G*Gt
        GGt = np.matmul(G,Gt)
        # GGt^-1
        GGt_inv = np.linalg.inv(GGt)
        # L
        L = np.matmul(Y,np.matmul(Gt,GGt_inv))

        # 3. Extract T and h
        T = L[:,0:3]
        h = L[:,[3]]

        # 4. Update M
        M_tilde = np.matmul(np.linalg.inv(T),(Y-h))
        # Get norm of M_tilde vectors
        M = M_tilde/np.linalg.norm(M_tilde,axis=0)

        # Set lambda if needed
        if not lmbd:
            lmbd = np.linalg.norm(T) # Set to norm of first iteration of T

        # 6. Calculate J (step 5 in paper is skipped here as we recalculate G in the loop)
        Jnew = sum(np.linalg.norm(Y-np.matmul(T,M)-h,axis=0)**2 + lmbd * (np.linalg.norm(M,axis=0)**2-1)**2 )
        print(Jnew)
        # Is this J small enough?
        if J and abs(J-Jnew)/J < tol_percent/100:

            return M,T,h # Retrun calibrated data and calibration parameters

        # If not, update J
        J = Jnew


# Function to read 'magCalParams' and apply T and h to a 3x1 or 1x3 magnetic field reading (Y)
def applyCalibration(Y):
    # Convert Y to np array if it's not already
    if not isinstance(Y,np.ndarray):
        Y = np.array(Y)

    # Convert to column vector if it's a row vector
    if np.shape(Y)[0] == 3:
        Y = Y[:,None]

    # Get magnetometer calibration matrices
    magCal = np.loadtxt('MagCalParams')
    T_inv = magCal[:,0:3]
    h = magCal[:,[3]] # Enclose 3 in list to force col vector

    # Apply calibration factors
    M = np.matmul(T_inv,Y-h)[:,0] # Isolate the column

    return M


# Calibration routine
if __name__ == "__main__":
    import bt
    import constants as cnst
    import time
    import matplotlib.pyplot as plt

    port = bt.getPortHandle()
    # Store bytes from incomplete packets
    unprocessedBytes = bytearray()
    # Make initial request for data
    port.write(bytes([cnst.REQ]))
    # Collect mag data as a 3xN matrix
    mag = [[],[],[]]
    # Timer
    startTime = time.time()
    TIMEOUT = 20
    # Loop
    while 1:
        # Read bt buffer
        data = bt.read(port,unprocessedBytes,magCalOn=False)

        # If anything is read
        if data:
            unprocessedBytes = data['bytes'] # Update bytes storage

            # If there's a complete lidar reading
            if data['imu']:
                port.write(bytes([cnst.REQ])) # Respond so that timeout doesn't occur

                # Put mag data into storage list
                for reading in data['imu']:
                    mag[0].append(reading.mag.x)
                    mag[1].append(reading.mag.y)
                    mag[2].append(reading.mag.z)


        # Check if user wants more time
        if time.time()-startTime > TIMEOUT:
            inp = input("Ready? y/n: ")
            if inp == 'y':
                break
            else:
                startTime = time.time() # Give more time

    # Plot uncalibrated readings
    fig = plt.figure(figsize=(12, 12))
    ax = fig.add_subplot(projection='3d')
    ax.scatter(mag[0], mag[1], mag[2])

    # Get calibration factors
    mag_cal,T,h = calibrate(mag)

    # Plot calibrated readings
    fig1 = plt.figure(figsize=(12, 12))
    ax1 = fig1.add_subplot(projection='3d')
    ax1.scatter(mag_cal[0,:], mag_cal[1,:], mag_cal[2,:])
    plt.show()

    # Save as txt file if desired
    save = input('Save calibration factors to file? y/n: ')
    if save == 'y':
        np.savetxt('magCalParams',np.hstack((np.linalg.inv(T),h)))
        # Format is [T^-1 h]
