'''
This script reads a .csv file with IMU data and plots it
'''
import matplotlib.pyplot as plt
import csv
import argparse

# Parse command line arguments
parser = argparse.ArgumentParser()
parser.add_argument("file", help="csv file containing the data")
args = parser.parse_args()

# Read data as a list of lists (same convention as in "read_and_store.py")
data = []
with open(args.file) as csvDataFile:
    csvReader = csv.reader(csvDataFile)
    for row in csvReader:
        data.append(row)

for i in range(len(data)):
	for j in range(9):
		data[i][j] = float(data[i][j])

t = range(len(data))


# Plot data

plt.figure(1)
plt.xlabel("time")
plt.ylabel("Rx")
plt.title("Gravity x component")
plt.plot(t,[data[i][0] for i in t],label = 'accel')
plt.plot(t,[data[i][1] for i in t],label = 'gyro')
plt.plot(t,[data[i][2] for i in t],label = 'estim')
plt.legend()


plt.figure(2)
plt.xlabel("time")
plt.ylabel("Ry")
plt.title("Gravity y component")
plt.plot([data[i][3] for i in t],label = 'accel')
plt.plot([data[i][4] for i in t],label = 'gyro')
plt.plot([data[i][5] for i in t],label = 'estim')
plt.legend()

plt.figure(3)
plt.xlabel("time")
plt.ylabel("Rz")
plt.title("Gravity z component")
plt.plot([data[i][6] for i in t],label = 'accel')
plt.plot([data[i][7] for i in t],label = 'gyro')
plt.plot([data[i][8] for i in t],label = 'estim')
plt.legend()

plt.show()


