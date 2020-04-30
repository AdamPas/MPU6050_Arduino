# MPU6050_Arduino
Code for Arduino and the MPU6050 IMU

The master branch contains code for running the EKF in the Arduino. The ekf data is communicated through Serial in real time,
stored and loaded, using python scripts.

The python_EKF branch has the arduino script communicate the raw sensors data. Once this is stored, using a python script,
it is used by the 
