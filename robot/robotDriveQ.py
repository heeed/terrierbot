#!/bin/python
#
#terrierbot control 
#
#v0.01
#

import sys, smbus, time, math, datetime, math, datetime, imu, piconzero as pz,serial,threading
from time import sleep
bus = smbus.SMBus(1)
speed = 60
blueConn= serial.Serial(
        port='/dev/rfcomm0',
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0
        )


def blueConnection():
    while True:
        blueMsg=blueConn.readline()
        #print(blueMsg)
	if blueMsg == "up":
            print("Up")
            pz.forward(speed)
            sleep(0.5)
            pz.stop()
            #break
        elif blueMsg == "down":
            print("Down")
            pz.reverse(speed)
            sleep(0.5)
            pz.stop

        elif blueMsg=="right":
            print("right")
            pz.spinRight(speed)
            sleep(0.5)
            pz.stop()

        elif blueMsg=="left":
            print("left")
            pz.spinLeft(speed)
            sleep(0.5)
            pz.stop()
        else:
            pz.stop()
        #       pz.stop()
        sleep(0.1)


print('Connected')
d = threading.Thread(name='daemon', target=blueConnection)
d.setDaemon(True)


try:
        pz.init()
	imu = imu.imuSystem(bus)
	imu.getBearing()
except:
	print "IMU sensor not found"
	sys.exit()

try:
	d.start()
	
   	while True:
		print("Still running")
#	d = threading.Thread(name='daemon', target=daemon)
		
#        keyp  = pz.readkey()
#       if ord(keyp) == 3:
 #           break
  #      else:
#	try:
      	#lueMsg=blueConn.readline()
 #   	except:
  #      	print("Receive error")
   #    		break;
	
	#	pz.stop()
    	#sleep(0.1)
        
        #pz.moveRobot(keyp,speed)
        #print(imu.getBearing())
        #sleep(0.03)
         #   pz.spinRight(speed)
          #  sleep(1)
           # pz.stop()
                sleep(0.5)

except KeyboardInterrupt:
    print("all off")

finally:
    pz.cleanup()
    #sock.close()
