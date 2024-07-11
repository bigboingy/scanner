from bt import read
import constants as cnst
import serial
import time
import imufusion
import numpy as np
import open3d as o3d
# import open3d as o3d

# Open port
# timeout waits for return of requested no. bytes specified in read() function, and also in port opening
port = serial.Serial(
    port="/dev/cu.HC-06", baudrate=115200, bytesize=8, timeout=5, stopbits=serial.STOPBITS_ONE
)

startTime = time.time()

# Open3d visualisation
vis = o3d.visualization.Visualizer()
vis.create_window(height=480*5, width=640*5)
# Make a cube
points = [
        [-4, -4, -4],
        [4, -4, -4],
        [-4, 4, -4],
        [4, 4, -4],
        [-4, -4, 4],
        [4, -4, 4],
        [-4, 4, 4],
        [4, 4, 4],
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

# Store bytes from incomplete packets
unprocessedBytes = bytearray()
# Make request for data
port.write(bytes([cnst.REQ]))

# Loop
running = True
while running:
    data = read(port,unprocessedBytes)
    if data:
        unprocessedBytes = data['bytes'] # Update bytes storage

        # If there's a complete lidar reading
        if data['imu']:
            port.write(bytes([cnst.REQ])) # Respond so that timeout doesn't occur

            # Magnetometer-free fusion
            # Takes gyro (1 by 3 matrix), acc (1 by 3 matrix)
            for reading in data['imu']:
                ahrs.update_no_magnetometer(np.array(reading.gyro), np.array(reading.acc), 1 / 1000)  # 1000 Hz sample rate
                #np.append(euler,ahrs.quaternion.to_euler())
                #print(ahrs.quaternion.to_matrix())

                
            # Update geometry
            line_set.points = o3d.utility.Vector3dVector(points) # Reset points
            line_set.rotate(ahrs.quaternion.to_matrix()) # Apply rotation
            vis.update_geometry(line_set)

    running = vis.poll_events() # Returns false on window close
    vis.update_renderer()

vis.destroy_window()
