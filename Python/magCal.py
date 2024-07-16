import numpy as np
import math

# Takes N magnetometer reading vectors and returns calibrated values, as well as T and h calibration parameters
# Based on algorithm presented in https://ieeexplore.ieee.org/abstract/document/8723161
# args:
# Y (array_like), a 3xN matrix of magnetic field strength readings (should be more than 12).
# tol (int), max % difference between iterations to consider converged
def calibrate(Y,tol=0.001):
    # Convert Y to np array if it's not already
    if not isinstance(Y,np.ndarray):
        Y = np.array(Y)

    # Get number of readings
    N = np.shape(Y)[1] 

    # 1. Initialise M (true magnetic field vector) as normalised Y. Must specify 2-norm
    M = Y/np.linalg.norm(Y,axis=0,ord=2)

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
        M = M_tilde/np.linalg.norm(M_tilde,axis=0,ord=2)

        # Set lambda if needed
        if not lmbd:
            lmbd = np.linalg.norm(T) # Set to norm of first iteration of T

        # 6. Calculate J (step 5 in paper is skipped here as we recalculate G in the loop)
        Jnew = sum(np.linalg.norm(Y-np.matmul(T,M)-h,axis=0,ord=2)**2 + lmbd * (np.linalg.norm(M,axis=0,ord=2)**2-1)**2 )
        print(Jnew)
        # Is this J small enough?
        if J and abs(J-Jnew)/J < tol/100:

            return M,T,h # Retrun calibrated data and calibration parameters

        # If not, update J
        J = Jnew

# Function to align mag axis with acc
# Gradient descent method
# F: Acc values, M: mag values (numpy arrays), tol: stopping criteria (max % difference between iterations to consider converged)
# Returns R (rotation matrix applied to calibrated mag data) and delta (magnetic field inclination angle)
def align(F,M,tol=0.001):

    # Convert F and M to np array if they aren't already
    if not isinstance(F,np.ndarray):
        F = np.array(F)
    if not isinstance(M,np.ndarray):
        M = np.array(M)


    # 1. Initialise delta and R
    delta = (90-69)*math.pi/180 # Initial guess for inclination angle. In 'degrees down' it's -69
    R = np.eye(3,3) # Rotation matrix
    N = len(F) # How many readings

    # 2. Initialise constants
    # Alpha and beta - backtracking line search constants
    alpha = 0.1
    beta = 0.5
    lmbda = 1
    mu = 1 # Lambda and mu are weightings applied to dJdR summands

    # Loop
    J=None # Initialise J, we check how much it's changed from prev iter
    while 1:
        # 3. Calculate the gradient. Need to loop thru each reading
        dJdR = np.zeros((9,1)) # 9x1
        dJdd = np.zeros((1,1)) # 1x1
        sinDelta = math.sin(delta)
        for k in range(N):
            normProd = np.linalg.norm(F[:,k],ord=2)*np.linalg.norm(M[:,k],ord=2)
            part1 = sinDelta - (F[:,k].T @ R @ M[:,k])/normProd # A scalar
            dJdd += part1
            part2 = part1 * np.reshape(np.kron(M[:,k],F[:,k])/normProd,(9,1)) # Need to reshape otherwise it goes row
            dJdR += part2

        detR = np.linalg.det(R)

        dJdR = -2*dJdR + 4*lmbda*np.reshape(R@R.T@R-R,(9,1)) + 2*mu*(detR-1)*np.reshape( (detR*np.linalg.inv(R)).T , (9,1) ) # Vectorise into col vectors
        dJdd = 2*math.cos(delta)*dJdd

        # 4. Work out step size, t
        # Calculate J depending on x, which is [[vecR],[delta]]
        def getJ(x):

            R = np.reshape(x[0:-1,0],(3,3)) # Get R and return it to a 3x3 matrix
            delta = x[-1][0]
            sinDelta = math.sin(delta)
            J = 0
            for k in range(N):
                normProd = np.linalg.norm(F[:,k],ord=2)*np.linalg.norm(M[:,k],ord=2)
                J += (sinDelta - (F[:,k].T @ R @ M[:,k])/normProd)**2 # A scalar
            J += lmbda*np.linalg.norm(R@R.T-np.eye(3,3),ord='fro')**2 + mu*(np.linalg.det(R)-1)**2
            return J

        # Decrease t until condition satisfied
        t = 1
        gradJ = np.vstack((dJdR,dJdd))
        deltaX = -gradJ
        x = np.vstack( ( np.reshape(R,(9,1)) , delta ))
        while getJ(x + t*deltaX) > getJ(x) + alpha*t*((gradJ.T)@deltaX)[0][0] : # Need the two zeros to get value out from doubly enclosed list
            t = beta*t

        # 5. Update R and delta
        xNew = x + t*deltaX
        R = np.reshape(xNew[0:-1,0],(3,3))
        delta = xNew[-1][0]

        # 6. Calculate J
        Jnew = getJ(xNew)
        print(Jnew)
        # Is this J small enough?
        if J and abs(J-Jnew)/J < tol/100:
            return R,delta # Retrun calibrated data and calibration parameters
        # If not, update J
        J = Jnew
        

# Function to read 'magCalParams' and apply T and h to a 3x1 or 1x3 magnetic field reading (Y).
# Extended to apply R, rotation matrix to allign mag axes with acc's
# Returns calibrated reading M (1x3 row vector)
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
    R = magCal[:,4:] 

    # Apply calibration factors
    M = R@(T_inv@(Y-h))[:,0] # Isolate the column, but return row vector for ease of access

    return M


# Calibration routine
if __name__ == "__main__":
    # a = [[1,2,3,4,5,6],[6,5,4,3,2,1],[3,4,3,4,3,4]]
    # b = [[2,5,2,5,2,6],[4,7,2,4,7,2],[2,5,2,6,4,2]]
    # align(a,b)
    

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
    # Collect acc data as a 3xN matrix
    acc = [[],[],[]]
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

                # Put mag and acc data into storage lists
                for reading in data['imu']:
                    mag[0].append(reading.mag.x)
                    mag[1].append(reading.mag.y)
                    mag[2].append(reading.mag.z)
                    acc[0].append(reading.acc.x)
                    acc[1].append(reading.acc.y)
                    acc[2].append(reading.acc.z)

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

    # Allign mag with acc
    R,delta = align(acc,mag_cal)

    # Save as txt file if desired
    save = input('Save calibration factors to file? y/n: ')
    if save == 'y':
        np.savetxt('magCalParams',np.hstack((np.linalg.inv(T),h,R)))
        # Format is [T^-1 h R]

    applyCalibration([1,1,1])
