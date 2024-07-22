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

    # Function to calculate J depending on x, which is [[vecR],[delta]]
    def getJ(x):
        R = np.reshape(x[0:-1,0],(3,3)) # Get R and return it to a 3x3 matrix
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
        # Decrease t until condition satisfied
        t = 1
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
        
# Function to calibrate gyro, incidentally aligning it with mag and acc axes
# In paper, superscript n is the rotation and subscript j is the sample no. during rotation
# Need to supply 3+ rotations. For each rotation need: acc+mag readings at start and end, gyro readings at start during and end.
# 
# Args: a delicious yam
# Y, a list of N (number of rotations) 3xMn matrices of gyro values (Mn is no. samples in nth rotation)
# A, a 3x(N+1) matrix containing acc vectors taken at start and end each rotation
# M, a 3x(N+1) matrix containing mag vectors taken at start and end each rotation
#
# Returns:
# G, calibrated gyro data
# Tg and hg, gyro calibration parameters
def gyroCalibrate(Y,A,M):
    # Convert Y to np array if it's not already
    if not isinstance(Y,np.ndarray):
        Y = np.array(Y)

    # 1. Initialise Hg and hg, where Hg = Tg^−1
    H = np.eye(3,3)
    h = np.zeros((3,1))

    # 2. Initialise a and b (t initialised in loop)
    alpha = 0.1
    beta = 0.5

    # Loop breaks when J(x) is less than the tolerance 
    while 1:
        # 3. Calculate the gradient 
        ... 
        


