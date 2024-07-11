
import constants as cnst
import numpy as np
import open3d as o3d
import time

# Open3d visualisation
vis = o3d.visualization.Visualizer()
vis.create_window(height=480*5, width=640*5)

points = [
        [0, 0, 0],
        [1, 0, 0],
        [0, 1, 0],
        [1, 1, 0],
        [0, 0, 1],
        [1, 0, 1],
        [0, 1, 1],
        [1, 1, 1],
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
    lines=o3d.utility.Vector2iVector(lines),
)
points1 = [
        [0, 0, 0],
        [1, 0, 0],
        [0, 1, 0],
        [1, 1, 0],
        [0, 0, 1],
        [1, 0, 1],
        [0, 1, 1],
        [2, 2, 2],
]
lines1 = [
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
line_set1 = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(points1),
    lines=o3d.utility.Vector2iVector(lines1),
)
vis.add_geometry(line_set)



dt = 0.05
previous_t = time.time()

rotation = np.array([[0, 0, 0], [0, 0, 0],[0, 0, 0]], np.int64)
centre = np.array([0.5, 0.5, 0.5], np.int64)

rotation = line_set.get_rotation_matrix_from_xyz((np.pi / 24, 0, 0))

running = True
while running:

    if time.time() - previous_t > dt:
        line_set.rotate(rotation)
        vis.update_geometry(line_set)
        previous_t = time.time()


    running = vis.poll_events() # Returns false on window close
    vis.update_renderer()

vis.destroy_window()
