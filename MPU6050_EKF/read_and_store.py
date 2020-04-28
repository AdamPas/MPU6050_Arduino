'''
A script to read data from the IMU sensor 6050, through the serial.
The data is saved in a .csv file.
Format: N rows, 9 columns
The first column contains the accelerometer measurements for the X axis.
The second the gyro estimation for the X axis.
The third column contains the EKF resulting estimation for the X axis.
etc. for Y and Z
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
	for i in range(len(data_list)):
		data_list[i] = float(data_list[i])

	# Append to total list
	data.append(data_list)

	print(data[-1])

	counter += 1


print('Closing python serial port!')
print('Writting data to file: ', args.write_file)

# save data to csv file for later processing
with open(args.write_file, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerows(data)

ser.close()