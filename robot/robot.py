#!/bin/python
#
#terrierbot control 
#
#v0.01
#

import sys, smbus, time, math, datetime, math, datetime, imu, piconzero as pz
from time import sleep
bus = smbus.SMBus(1)
speed = 60


try:
        pz.init()
	imu = imu.imuSystem(bus)
	imu.getBearing()
except:
	print "IMU sensor not found"
	sys.exit()

try:
    while True:
        #keyp = pz.readkey()
#        if ord(keyp) == 3:
 #           break
  #      else:
         #   pz.moveRobot(keyp,speed)
        print(imu.getBearing())
        sleep(0.03)
         #   pz.spinRight(speed)
          #  sleep(1)
           # pz.stop()
           # sleep(2)

except KeyboardInterrupt:
    print("all off")

finally:
    pz.cleanup()
