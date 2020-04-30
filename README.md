# MPU6050_Arduino
Code for Arduino and the MPU6050 IMU

This repo contains 3 folders:
1) The callibration code can be used once to find the IMU offsets, unique to each physical IMU unit.

2) The simple weights folder, contains code that runs in the Arduino and implements a simple weighted average between the accelerometer and the gyro data.

3) The EKF folder contains code for both the Arduino the python. 
By running the Arduino script "MPU6050_EKF.ino", data from the sensors are communicated through the serial to the PC.
This data can be read and stored in the "Data" subfolder, by executing the "store_raw.py" script. In reality, this data is already transmitted in physical units from the Arduino, so they are not exactly "raw".
Finally, the script "EKF.py" reads a specified file with IMU data and performs the EKF estimation algorithm to find the best possible estimation of the gravity acceleration components on the 3 local axis. Results are plotted.
