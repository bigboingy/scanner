from bt import read
import constants as cnst
import serial
import imufusion
import numpy as np
import open3d as o3d

# Open port
# timeout waits for return of requested no. bytes specified in read() function, and also in port opening
port = serial.Serial(
    port="/dev/cu.HC-06", baudrate=115200, bytesize=8, timeout=5, stopbits=serial.STOPBITS_ONE
)

# Open3d visualisation
vis = o3d.visualization.Visualizer()
vis.create_window(height=480*5, width=640*5)
# Make a cube
points = [
        [-8, -4, -2],
        [8, -4, -2],
        [-8, 4, -2],
        [8, 4, -2],
        [-8, -4, 2],
        [8, -4, 2],
        [-8, 4, 2],
        [8, 4, 2],
]
lines = [
        [0, 1],
        [0, 2],
        [1, 3],
        [2, 3],
        [4, 5],
        [4, 6],
        [5, 7],
        [6, 7],
        [0, 4],
        [1, 5],
        [2, 6],
        [3, 7],
]
line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(points),
    lines=o3d.utility.Vector2iVector(lines),)
vis.add_geometry(line_set)
# Add coord axis
axis = o3d.geometry.TriangleMesh.create_coordinate_frame()
vis.add_geometry(axis)

# Imu fusion setup
ahrs = imufusion.Ahrs()
euler = np.empty((0, 3))
sample_rate = 100 # Hz
ahrs.settings = imufusion.Settings(imufusion.CONVENTION_ENU,  # convention -  north west up
                                   0.5,  # gain
                                   1000,  # gyroscope range
                                   1,  # acceleration rejection
                                   10,  # magnetic rejection, max difference bw algorithm and magnetometer before mag is ignored
                                   5 * sample_rate)  # recovery trigger period = 5 seconds

# Rejecting acc, mag is still rejected sometimes --> mag is unaligned with gyro
# Rejecting mag, acc is not rejected --> acc agrees with gyro
# Mag is unaligned with gyro/acc --> allign

# Store bytes from incomplete packets
unprocessedBytes = bytearray()
# Make initial request for data
port.write(bytes([cnst.REQ]))

# Loop
running = True
prevCounter = 0xFFFF # You need to put in dt between fusion updates!
while running:
    data = read(port,unprocessedBytes,magCalOn=True,magAlignOn=True)
    if data:
        unprocessedBytes = data['bytes'] # Update bytes storage

        # If there's a complete lidar reading
        if data['imu']:
            port.write(bytes([cnst.REQ])) # Respond so that timeout doesn't occur

            # Fusion
            # Takes gyro (1 by 3 matrix), acc (1 by 3 matrix), mag (1 by 3 matrix) and dt
            for reading in data['imu']:

                # Turn clock counter into a time difference
                if prevCounter > reading.time:
                    dt = (prevCounter-reading.time)/(100*1000)
                else:
                    dt = (prevCounter+(0xFFFF-reading.time))/(100*1000) # 0xFFFF is the max counter (what clock resets to after reaching 0)
                # Update sensor fusion
                ahrs.update(np.array(reading.gyro), np.array(reading.acc), np.array(reading.mag), dt) # Transpose M to make row vector again
                prevCounter = reading.time

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
                

            # Update geometry
            line_set.points = o3d.utility.Vector3dVector(points) # Reset points
            line_set.rotate(ahrs.quaternion.to_matrix()) # Apply rotation
            vis.update_geometry(line_set)

    running = vis.poll_events() # Returns false on window close
    vis.update_renderer()

vis.destroy_window()
