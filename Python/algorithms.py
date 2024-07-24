import numpy as np
import math

# Replaced by calibration.py

# Takes N magnetometer reading vectors and returns calibrated values, as well as T and h calibration parameters
# Based on algorithm presented in https://ieeexplore.ieee.org/abstract/document/8723161
# Args:
# Y (array_like), a 3xN matrix of magnetic field strength readings.
# tol (int), max % difference between iterations to consider converged.
# Returns:
# M, calibrated magnetometer data
# T and h, calibration parameters
# This function can also be used to calibrate accelerometer data! This is bcs both acc/mag magnitude should be constant w orientation
def magCalibrate(Y,tol=0.001):
    # Convert Y to np array if it's not already
    if not isinstance(Y,np.ndarray):
        Y = np.array(Y)

    # Get number of readings
    N = np.shape(Y)[1] 

    # 1. Initialise M (true magnetic field vector) as normalised Y (must specify 2-norm), and G
    M = Y/np.linalg.norm(Y,axis=0,ord=2)
    G = np.vstack((M,np.ones((1,N))))

    # Initialise lambda, set to ||T|| in first loop
    lmbd = 0
    # Initialise J
    J=None

    # Iterate
    while 1:

        # 2. Make L=YGt(GGt)^-1
        L = Y@G.T@np.linalg.inv(G@G.T)

        # 3. Extract T and h
        T = L[:,0:3]
        h = L[:,[3]]

        # 4. Update M
        M_tilde = np.linalg.inv(T)@(Y-h)
        # Get norm of M_tilde vectors
        M = M_tilde/np.linalg.norm(M_tilde,axis=0,ord=2)

        # Set lambda if needed
        if not lmbd:
            lmbd = np.linalg.norm(T) # Set to norm of first iteration of T

        # 5. Update G
        G = np.vstack((M,np.ones((1,N))))

        # 6. Calculate J
        Jnew = sum(np.linalg.norm(Y-T@M-h,axis=0,ord=2)**2 + lmbd * (np.linalg.norm(M_tilde,axis=0,ord=2)**2-1)**2 ) # Paper has M instead of M_tilde
        #Jnew = sum((np.linalg.norm(M_tilde,axis=0,ord=2)**2-1)) - what a review recommends
        print(Jnew)
        # Is this J small enough?
        if J and abs(J-Jnew)/J < tol/100:

            return M,T,h # Retrun calibrated data and calibration parameters

        # If not, update J
        J = Jnew

# Function to align mag axis with acc
# Uses the gradient descent method
# Args:
# A, 3xN matrix of calibrated acc values
# M, 3xN matrix of calbrated mag values 
# tol, stopping criteria (max % difference between iterations to consider converged)
# Returns:
# R, rotation matrix to applied to mag data to align with acc
# delta, magnetic field inclination angle
def magAlign(A,M,tol=0.001):

    # Convert F and M to np array if they aren't already
    if not isinstance(A,np.ndarray):
        A = np.array(A)
    if not isinstance(M,np.ndarray):
        M = np.array(M)

    # 1. Initialise delta and R
    delta = -69*math.pi/180 # Initial guess for inclination angle. Negative means M points upwards
    R = np.eye(3,3) # Rotation matrix
    N = np.shape(A)[1] # How many readings

    # 2. Initialise constants
    alpha = 0.1 # Alpha and beta are backtracking line search constants
    beta = 0.5
    lmbda = 1 # Lambda and mu are weightings applied to dJdR summands
    mu = 1 

    # Function to calculate J depending on x, which is [vecR.T, delta].T
    def getJ(x):
        R = np.reshape(x[0:-1,0],(3,3),order='F') # Get R and return it to a 3x3 matrix. Order = F means read/place column wise
        delta = x[-1][0]
        sinDelta = math.sin(delta)
        J = 0
        for k in range(N):
            normProd = np.linalg.norm(A[:,k],ord=2)*np.linalg.norm(M[:,k],ord=2)
            J += (sinDelta - (A[:,k].T @ R @ M[:,k])/normProd)**2 # A scalar
        J += lmbda*np.linalg.norm(R@R.T-np.eye(3,3),ord='fro')**2 + mu*(np.linalg.det(R)-1)**2
        return J

    # Loop
    J=None # Initialise J, we check how much it's changed from prev iter
    while 1:
        # 3. Calculate the gradient. Need to loop thru each reading
        dJdR = np.zeros((9,1)) # 9x1
        dJdd = np.zeros((1,1)) # 1x1
        sinDelta = math.sin(delta)
        for k in range(N):
            normProd = np.linalg.norm(A[:,k],ord=2)*np.linalg.norm(M[:,k],ord=2)
            part1 = sinDelta - (A[:,k].T @ R @ M[:,k])/normProd # A scalar
            dJdd += part1
            part2 = part1 * np.reshape(np.kron(M[:,k],A[:,k])/normProd,(9,1)) # Need to reshape otherwise it goes row
            dJdR += part2

        detR = np.linalg.det(R)
        dJdR = -2*dJdR + 4*lmbda*np.reshape(R@R.T@R-R,(9,1)) + 2*mu*(detR-1)*np.reshape( (detR*np.linalg.inv(R)).T , (9,1) ) # Vectorise into col vectors
        dJdd = 2*math.cos(delta)*dJdd
        gradJ = np.vstack((dJdR,dJdd))
        deltaX = -gradJ

        # 4. Work out step size, t
        # Decrease t until condition satisfied. Order = F means flatten column wise
        t = 1
        x = np.vstack( ( np.reshape(R.flatten(order='F'),(9,1)) , delta ))
        while getJ(x + t*deltaX) > getJ(x) + alpha*t*((gradJ.T)@deltaX)[0][0] : # Need the two zeros to get value out from doubly enclosed list
            t = beta*t

        # 5. Update R and delta
        xNew = x + t*deltaX
        R = np.reshape(xNew[0:-1,0],(3,3),order='F')
        delta = xNew[-1][0]

        # 6. Calculate J
        Jnew = getJ(xNew)
        print(Jnew)
        # Is this J small enough?
        if J and abs(J-Jnew)/J < tol/100:
            return R,delta # Retrun calibrated data and calibration parameters
        # If not, update J
        J = Jnew
        
# Function to calibrate gyro, incidentally aligning it with mag and acc axes
# In paper, superscript n is the rotation and subscript j is the sample no. during rotation
# Need to supply 3+ rotations. For each rotation need: acc+mag readings at start and end, gyro readings at start during and end.
# 
# Args: 
# Y, a list of N (number of rotations) 3xMn matrices of gyro values (Mn is no. samples in nth rotation)
# A, a 3x2N matrix containing acc vectors taken at start and end each rotation (averaged)
# M, a 3x2N matrix containing mag vectors taken at start and end each rotation (averaged)
# wStill is a 3x1 matrix containg gyro vector when IMU is stationary (averaged)
# Freq is the sample rate (Hz)
# tol is the stopping criteria (max % difference between iterations to consider converged)
#
# Returns:
# G, calibrated gyro data
# Tg and hg, gyro calibration parameters
def gyroCalibrate(Y,A,M,freq,wStill,tol=0.001):
    # Convert elements of Y to np array if they aren't already
    if not isinstance(Y[0],np.ndarray):
        for i in range(len(Y)):
            Y[i] = np.array(Y[i])
    # Convert A to np array if it's not already
    if not isinstance(A,np.ndarray):
        A = np.array(A)
    # Convert M to np array if it's not already
    if not isinstance(M,np.ndarray):
        M = np.array(M)

    # How many rotations?
    N = len(Y)

    # 1. Initialise Hg and hg, where Hg = Tg^−1
    H = np.eye(3,3)
    h = np.zeros((3,1))

    # 2. Initialise a and b (t initialised in loop)
    alpha = 0.1
    beta = 0.5
    lmbda = 1 # Applied to 2nd summand when finding J
    epsilon = np.finfo(np.float64).eps # Machine epsilon


    # Function to get Ram for a single rotation
    # Inputs are 3x1 column vectors
    # np.norm defaults to 2-norm for vectors. np.cross needs axis=0 for column vectors
    def getRam(fbegin,fend,mbegin,mend):

        # Construct column vectors
        a1 = fbegin
        a2 = np.cross(fbegin,mbegin,axis=0)/np.linalg.norm(np.cross(fbegin,mbegin,axis=0))
        a3 = np.cross(fbegin,np.cross(fbegin,mbegin,axis=0),axis=0)/np.linalg.norm(np.cross(fbegin,mbegin,axis=0))
        b1 = fend
        b2 = np.cross(fend,mend,axis=0)/np.linalg.norm(np.cross(fend,mend,axis=0))
        b3 = np.cross(fend,np.cross(fend,mend,axis=0),axis=0)/np.linalg.norm(np.cross(fend,mend,axis=0))
        
        # Calculate rotation matrix
        Ram = np.hstack((b1,b2,b3)) @ np.hstack((a1,a2,a3)).T
        return Ram
    
    # Ram is constant, calculate them here (one for each rot)
    Ram_list = []
    # For each rotation
    for i in range(N):
        # There are two readings per rotation
        Ram = getRam(A[:,[2*i]],A[:,[2*i+1]],M[:,[2*i]],M[:,[2*i+1]])
        Ram_list.append(Ram)

    # Function to get Rg for a single rotation
    # Uses uncalibrated gyroscope data w, a 3xMn matrix
    # Calibrates the data with H and h
    def getRg(w,H,h):

        # How mamy samples? Columns of w
        K = np.shape(w)[1]

        # Calibrate w
        w_tilde = H@(w-h)

        # Make R
        Rg = np.eye(3,3)
        for k in range(K): # For each read (column)
            wk = w_tilde[:,k] # This read, as a row vector
            omega = np.array([ [0,-wk[2],wk[1]], [wk[2],0,-wk[0]], [-wk[1],wk[0],0] ]) # Make omega
            Rg = Rg @ (np.eye(3,3) + freq*omega)

        return Rg

    # Function to return J (scalar)
    # x is defined as [vec(Hg).T hg.T].T (column)
    def getJ(x):
        # Extract Hg and hg
        H = np.reshape(x[0:9,0],(3,3),order='F') # Get H and return it to a 3x3 matrix
        h = np.reshape(x[-3:,0],(3,1),order='F') # Need reshape to make a column vector

        # Calculate Rg for each rotation
        Rg_list = []
        for i in range(N):
            Rg = getRg(Y[i],H,h)
            Rg_list.append(Rg)

        # Calculate J
        J = 0
        # First, calculate first summand
        for i in range(N):
            J += np.linalg.norm(Ram_list[i]-Rg_list[i],ord=2)**2

        # Then, add on second summand
        J += lmbda*np.linalg.norm(H@(wStill-h))**2

        return J

    # Loop breaks when J(x) is less than the tolerance
    J = None # Initialise J
    while 1:
        # 3. Calculate the gradient of J numerically: f' = [ f(x+h) - f(x-h) ]/ 2h, for each element of x
        gradJ = np.empty((12,1)) # Preallocate 9x1 vector

        # Make x [vec(Hg).T hg.T].T 
        x = np.vstack( ( np.reshape(H.flatten(order='F'),(9,1)) , np.reshape(h.flatten(order='F'),(3,1)) ))

        # Work out how J changes when changing each variable in x
        MIN_X = 0.01 # How close to zero can x be to use the optimised h formula?
        for i in range(len(x)):
            if abs(x[i]) < MIN_X:
                h = 0.001 # Default to this if preferred formula can't be used
            else:
                h = epsilon**(1/3)*x[i] # Optimal stepsize
            # Calculate gradient
            gradJ[i] = (getJ(x+h) - getJ(x-h))/(2*h)
        # We move opposite to gradient (steepest descent)    
        deltaX = -gradJ

        # 4. Backtracking line search
        t = 1
        while getJ(x + t*deltaX) > getJ(x) + (alpha*t*gradJ.T@deltaX)[0][0]: # Get value from double list
            t = beta*t # Reduce stepsize
        
        # 5. Update x using x=x+t*deltaX, and set its contents
        xNew = x + t*deltaX
        H = np.reshape(xNew[0:9,0],(3,3),order='F')
        h = np.reshape(xNew[-3:,0],(3,1),order='F')

        # 6. Calculate J
        Jnew = getJ(xNew)
        print(Jnew)

        # Is this J small enough?
        if J and abs(J-Jnew)/J < tol/100:
            return H,h # Retrun calibration parameters
        # If not, update J
        J = Jnew

        


 
        


