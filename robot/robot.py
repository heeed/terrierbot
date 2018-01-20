#!/bin/python
#
#terrierbot control 
#
#v0.01
#

import sys, smbus, time, math, datetime, math, datetime, imu
from time import sleep
bus = smbus.SMBus(1)

try:
	imu = imu.imuSystem(bus)
	imu.getBearing()
except:
	print "IMU sensor not found"
	sys.exit()

while True:
	print(imu.getBearing())
	sleep(0.1)
