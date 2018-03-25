#!/bin/python
#
#terrierbot control 
#
#v0.01
#

import os,sys, smbus, time, math, datetime, math, datetime, imu, piconzero as pz
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
     #   keyp = pz.readkey()
#       if ord(keyp) == 3:
 #           break
  #      else:
      #  pz.moveRobot(keyp,speed)
      #  print(imu.getBearing())
    #    sleep(0.03)
         #   pz.spinRight(speed)
          #  sleep(1)
           # pz.stop()
           # sleep(2)
        reqBearing = 50 
        leftBearingRange=reqBearing - 1
        rightBearingRange=reqBearing + 1
        
        if leftBearingRange < 0: #work out if left range is less then 0 & adjust to be 360 less amount
            leftBearingRange = 360 + leftBearingRange 
       
        if rightBearingRange > 360: #opposite of above tuned for right bearing
            rightBearingRange = 0+(rightBearingRange-360)
            leftBearingRange = rightBearingRange - 1 

        print("\nRB: "+str(reqBearing)+"| LB: "+str(leftBearingRange)+" |  RB:"+str(rightBearingRange))
        bearing = imu.getBearing()
        #print(bearing)
       
        #angle = 180 - abs(abs(bearing - reqBearing) - 180); 
        #print("Angle: "+str(angle))

        #if 1 >= reqBearing <= 180:
        if reqBearing >= 1 and reqBearing <= 180:
            print("Going Right")
            print("Right Turn: "+str(bearing))
            if leftBearingRange >= bearing and rightBearingRange >= bearing:
                print("inside turn loop")
                pz.spinRight(speed)
                sleep(0.1)
                pz.stop()
                sleep(0.1)
        elif reqBearing >= 181 and reqBearing <= 359:
            print("Going Left")
            print("Left Turn: "+str(bearing))
            if leftBearingRange <= bearing and rightBearingRange <= bearing:
                #print(imu.getBearing())
                pz.spinLeft(speed)
                sleep(0.1)
                pz.stop()
                sleep(0.1)
        else:
            print("Nope stopping")
            pz.stop()
except KeyboardInterrupt:
    print("all off")

finally:
    pz.cleanup()
