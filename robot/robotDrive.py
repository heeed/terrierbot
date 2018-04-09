#!/bin/python
#
#terrierbot control 
#
#v0.01
#

import sys, smbus, time, math, datetime, math, datetime, imu, piconzero as pz, bluetooth
from time import sleep
bus = smbus.SMBus(1)
speed = 60
bd_addr = "00:14:03:06:1E:DF" 
port = 1
sock = bluetooth.BluetoothSocket (bluetooth.RFCOMM)
sock.connect((bd_addr,port))

print('Connected')

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
#       if ord(keyp) == 3:
 #           break
  #      else:
	try:
        	data = sock.recv(7)
    	except bluetooth.BluetoothError as err:
        	print("Receive error")
       		break;
    	if len(data) > 0:
        	store = ""
        	sleep(0.2)
        	datatemp=str(data)
        	print(datatemp)
        if len(datatemp)==7:
            store=store+datatemp
            output = store+"\n"
    	
	x,y = output.split(':')
    	print(x)
    	print(y)
	if y <= 200:
		pz.forward(speed)
	else:
		pz.stop()
    	#sleep(0.1)
        
        #pz.moveRobot(keyp,speed)
        #print(imu.getBearing())
        #sleep(0.03)
         #   pz.spinRight(speed)
          #  sleep(1)
           # pz.stop()
           # sleep(2)

except KeyboardInterrupt:
    print("all off")

finally:
    pz.cleanup()
    sock.close()
