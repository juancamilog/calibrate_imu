#!/usr/bin/env python2

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import pickle
import argparse

parser = argparse.ArgumentParser("Plot the results from calibrating the magnetometer")
parser.add_argument('filename', metavar='filename', type=str, nargs='+',help='path to the file obtained from the calibrate_imu node')

args = parser.parse_args()
f = open(args.filename[0])
x0 = pickle.load(f)
L = pickle.load(f)
data = pickle.load(f)
normalized_data = (L*(data.T-x0)).T

fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')

print data.shape
print normalized_data.shape

ax.scatter(np.array(data[:,0]),np.array(data[:,1]),np.array(data[:,2]),c='b',marker='^')

fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')
ax.scatter(np.array(normalized_data[:,0]),np.array(normalized_data[:,1]),np.array(normalized_data[:,2]),c='r',marker='o')

plt.show()
