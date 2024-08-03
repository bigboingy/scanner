import imufusion
import constants as cnst
import numpy as np

# In: ahrs object, list of imu tuples, and previous counter
# Out: updated ahrs, associated list of rotation matrices and new previous counter
def imuFusion(ahrs,imus,lastCounter):

    prevCounter = lastCounter

    R = []

    # For each imu
    for imu in imus:

    # Find dt
        if prevCounter > imu.time:
            dt = prevCounter-imu.time
        else:
            dt = prevCounter+(0xFFFF/cnst.DATATIMER_FREQ-imu.time) # 0xFFFF is the max counter (what clock resets to after reaching 0)
    
        # Update sensor fusion
        # Takes gyro (1 by 3 matrix), acc (1 by 3 matrix), mag (1 by 3 matrix) and dt
        ahrs.update(np.array(imu.gyro), np.array(imu.acc), np.array(imu.mag), dt) # Transpose M to make row vector again
        prevCounter = imu.time

        # Check status flags and internal states
        flags = ahrs.flags
        if flags.initialising: print('Initialising')
        if flags.angular_rate_recovery: print('Angular rate recovery')
        if flags.acceleration_recovery: print('Acceleration recovery')
        if flags.magnetic_recovery: print('Magnetic recovery')
        states = ahrs.internal_states
        if states.accelerometer_ignored:
            print('Accelerometer ignored, error: %.1f' % states.acceleration_error)
        if states.magnetometer_ignored:
            print('Magnetometer ignored, error: %.1f' % states.magnetic_error)
        print('x')
        # Save rotation matrix
        R.append(ahrs.quaternion.to_matrix())
    
    return ahrs,R,prevCounter


