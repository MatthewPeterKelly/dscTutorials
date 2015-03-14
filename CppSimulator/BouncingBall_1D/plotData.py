#!/usr/bin/python

import numpy
from matplotlib import pyplot as plt

data = numpy.genfromtxt('data.csv',skiprows=0,delimiter=',')

time = data[:,0]
pos = data[:,1]

plt.plot(time,pos)
plt.title('bouncing ball')
plt.xlabel('time (s)')
plt.ylabel('height (m)')
plt.show()