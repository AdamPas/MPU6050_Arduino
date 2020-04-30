'''

Author: adam

 * Using the MPU6050 IMU and an EKF, orientation of the sensor is estimated as the components of the gravity vector (referred to as R)
 * Specifically, the state vector consists of the 3 components of the gravity vector in the local IMU axis
 * 
 * For the theoretical analysis, have a look here:
 * http:#www.starlino.com/imu_guide.html
 * 
 * For a simplified fusion with fixed weight, have a look here: 
 * http:#www.starlino.com/imu_kalman_arduino.html
 * 
 * Extended Kalman Filter:
 * Prediction step: Previous state + gyro measurements
 * Correction step: Accelerometer measurement

 This script considers pre-acquired IMU sensor data, performs the EKF algorithm and plots the results
'''

import matplotlib.pyplot as plt
import numpy as np
import math
import csv
import argparse

# global variables to hold the sensor data
accel_data = []	# (N,3)
gyro_data = []  # (N,3)
dt_data = []    # (N,1)


def parse_arguments():
	# Parse command line arguments

	parser = argparse.ArgumentParser()
	parser.add_argument("file", help="csv file containing the data")
	args = parser.parse_args()
	filepath = './Data/' + args.file

	return filepath

def data_loader(filepath):
	# Read data as a list of lists (same convention as in "read_and_store.py")

	print('Loading sensor data from file:',filepath)
	
	# Collect data from .csv file
	with open(filepath) as csvDataFile:
		csvReader = csv.reader(csvDataFile)
		for row in csvReader:
			accel_data.append(row[0:3])
			gyro_data.append(row[3:6])
			dt_data.append(row[6])

	# typecast data to float
	for i in range(len(accel_data)):
		for j in range(3):	# known data row structure
			accel_data[i][j] = float(accel_data[i][j])
			gyro_data[i][j] = float(gyro_data[i][j])
		dt_data[i] = float(dt_data[i])

	print('Loaded data successfully!')

def plot(accel, gyro, ekf):
# Plot data

	plt.figure(1)
	plt.xlabel("time")
	plt.ylabel("Rx")
	plt.title("Gravity x component")
	plt.plot(accel[0,:],label = 'accel')
	plt.plot(gyro[0,:],label = 'gyro')
	plt.plot(ekf[0,:],label = 'estim')
	plt.legend()


	plt.figure(2)
	plt.xlabel("time")
	plt.ylabel("Ry")
	plt.title("Gravity y component")
	plt.plot(accel[1,:],label = 'accel')
	plt.plot(gyro[1,:],label = 'gyro')
	plt.plot(ekf[1,:],label = 'estim')
	plt.legend()

	plt.figure(3)
	plt.xlabel("time")
	plt.ylabel("Rz")
	plt.title("Gravity z component")
	plt.plot(accel[2,:],label = 'accel')
	plt.plot(gyro[2,:],label = 'gyro')
	plt.plot(ekf[2,:],label = 'estim')
	plt.legend()

	plt.show()


def ekf(gyro_noise,accel_noise):
	# Performs EKF estimation
	# gyro_noise: The gyro noise covariance terms (for simplicity it is used in the main diagonal)
	# accel_noise: The accelerometer noise covariance terms

	# Following arrays in format (3,N)
	# The entire accelerometer measurements data
	R_Accel = np.array(accel_data).T # (3,N)
	# Initialize algorithm with accelerometer measurement
	R_Gyro = np.array(R_Accel[:,0]).reshape(3,1)
	R_Estim = np.array(R_Accel[:,0]).reshape(3,1)

	### EKF arrays ###
	K = np.eye(3)  			# Kalman gain (updated in every iteration)
	P = 0.1 * np.eye(3)  	# state covariance matrix
	F = np.eye(3)  			# model Jacobian (updated in every iteration)
	H = np.eye(3)  			# measurement array and its Jacobian
	# model and gyro noise covariance
	Q = np.array([[gyro_noise, 0.0, 0.0], [0.0, gyro_noise, 0.0], [0.0, 0.0, gyro_noise]])
	# accelerometer noise covariance
	R = np.array([[accel_noise, 0.0, 0.0], [0.0, accel_noise, 0.0], [0.0, 0.0, accel_noise]])
	###################

	### EKF algorithm ###
	# repeat for all measurements
	for i in range(1,len(accel_data)):

		### Prediction step ###
		# Update with gyro data and previous estimate, only if Rz is not too small.
		if abs(R_Estim[2,-1]) > 0.1:
			
			# first calculate Jacobian of update function (linearization at previous state)
			update_F(F,R_Estim[:,-1:],i)

			# then evaluate the update function, using previous estimation and gyro measurement
			R_new = evaluate_update(R_Estim[:,-1:],i)

			# update state covariance
			P = np.dot(F, np.dot(P, F.T)) + Q

		# Else, skip prediction step --> prediction = previous filter estimation
		else:
			R_new = R_Estim[:,-1:]

		# normalize gyro estimation
		normalize(R_new)

		# append the new estimation to both the ekf and the gyro vectors
		R_Estim = np.append(R_Estim,R_new,axis=1)
		R_Gyro = np.append(R_Gyro,R_new,axis=1)

		########################

		### EKF Correction step ###
		# compute innovation covariance as a middle step
		inn_covariance = R + np.dot(H,np.dot(P,H.T))

		# compute kalman gain
		K = np.dot(P, np.dot(H.T, np.linalg.inv(inn_covariance)))

		# correct state vector (with innovation)
		R_Estim[:,-1:] += np.dot(K, (R_Accel[:,i:i+1]-np.dot(H, R_Estim[:,-1:])))

		# normalize estimation vector (again)
		normalize(R_Estim[:,-1:])

		# update covariance matrix
		P -= np.dot(K, np.dot(H, P))

		#########################

	return R_Accel, R_Gyro, R_Estim
	##########################


def normalize(vec):
	# Normalizes a 3d-vector

	vec /= np.sqrt(vec[0]**2 + vec[1]**2 + vec[2]**2)

def evaluate_update(R,curr_iter):
	# Evaluates the prediction function of the EKF, at the given R[n-1] and gives the result in R_result
	# current iteration: this is the step in EKF algorithm AND index to the data arrays

	R_result = np.zeros((3,1))

	# get angles between projection of R on ZX/ZY plane and Z axis, based on last R_Estim and gyro measurement
	Axz = (math.degrees(math.atan2(R[0], R[2]))) + gyro_data[curr_iter][0] * dt_data[curr_iter]
	Ayz = (math.degrees(math.atan2(R[1], R[2]))) + gyro_data[curr_iter][1] * dt_data[curr_iter]

	# estimate sign of RzGyro by looking in what qudrant the angle Axz is: pozitive if Axz in range -90 ..90 => cos(Axz) >= 0
	if math.cos(math.radians(Axz)) >=0:
		signRz = 1
	else:
		signRz = -1

	R_result[0] = math.sin(math.radians(Axz)) \
				  / math.sqrt( 1 + (math.cos(math.radians(Axz))**2) * (math.tan(math.radians(Ayz))**2) )
	R_result[1] = math.sin(math.radians(Ayz)) \
				  / math.sqrt( 1 + (math.cos(math.radians(Ayz))**2) * (math.tan(math.radians(Axz))**2) )
	R_result[2] = signRz * math.sqrt(1 - R_result[0]**2 - R_result[1]**2)

	return R_result


def update_F(F,last_estim,curr_iter):
	# Approximates the prediction Jacobian with central finite differences
	# current iteration: this is the step in EKF algorithm AND index to the data arrays

	step = 1e-5	# the finite difference step

	# for each of the 3 state variables
	for i in range(3):
		# forward step
		Rtemp = last_estim				     		# reset Rtemp to last R_Estim
		Rtemp[i] += step            			 	# forward step only for the state variable under perturbation
		R_step_forward = evaluate_update(Rtemp,curr_iter)       # evaluate update function one step ahead

		# backward step
		Rtemp = last_estim							# reset Rtemp to last R_Estim
		Rtemp[i] -= step							# backward step only for the state variable under perturbation
		R_step_back = evaluate_update(Rtemp,curr_iter)          # evaluate update function one step ahead

		# fill the corresponding column of the update Jacobian F
		F[:,i:] = ((R_step_forward[:] - R_step_back[:]) / (2*step))


if __name__ == '__main__':

	# parse command line arguments
	filepath = parse_arguments();

	# load sensor data
	data_loader(filepath);
	
	# EKF function
	accel, gyro, ekf = ekf(1e-5,1e-4)

	# plotting
	plot(accel, gyro, ekf)

