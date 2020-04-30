'''
Author: adam

A script to read data from the IMU sensor 6050, through the serial.
The data is saved in a .csv file.
Format: N rows, 7 columns
		Each row contains a different timestamp
        First 3 columns contain the accelerometer data, the next 3 columns the gyro, last column contains the dt for this timestep
        Units for acceleration are in m/sec² and for angular velocity in deg/sec
'''

import serial
import csv
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("num_data", type=int, help="number of data to collect", default=1000)
parser.add_argument("write_file", help="the file to write in")

args = parser.parse_args()


# Open serial
ser = serial.Serial('/dev/ttyACM0', 38400, timeout=1)


# When this script runs, there is some leftover data being transmitted
# So, start considering data, only after the Arduino script sends initialization messages
while True:
	s = str(ser.readline())	# the read object is a bytes object, so convert to string
	if 'successful' in s:
		print('Start data recording.')
		break
	

# Initialize empty list
data = []

# Repeat for a specified amout of measurements
counter = 0;
while counter < args.num_data:
	s = ser.readline().decode('utf-8')
	
	# Parse values and convert to float
	data_list = s.split(',')
	print(data_list)
	for i in range(len(data_list)):
		data_list[i] = float(data_list[i])

	# Append to total list
	data.append(data_list)

	# For debugging purposes, print the last (most current) row
	# print(data[-1])

	counter += 1


print('Closing python serial port!')
ser.close()

# save data to csv file for later processing
filepath = './Data/' + args.write_file
print('Writting data to file: ', filepath)

with open(filepath, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerows(data)

