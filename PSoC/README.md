3D Lidar Scanner project using a lidar and IMU. Will it work...?\
\
Control system: PSOC 5LP\
Lidar: TFmini plus\
IMU: ICM-20948\
\
Features:
- ICM-20948 library for initialisation and data reading
- Communication protocol for sending lidar and IMU data to computer using bluetooth
- Python scripts for: 
  - calibrating magnetometer and alligning axes with accelerometer axes (not working yet),
  - graphing IMU data using matplotlib,
  - implementation and visualisation of IMU sensor fusion using imufusion and open3D
 
The final goal is to generate and display point clouds in real time.

IMU calibration algorithms are from https://ieeexplore.ieee.org/abstract/document/8723161
  
  
