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
        if len(blueMsg)>0:
            screenMessage.printMsg(blueMsg)
	if blueMsg == "up":
            pz.forward(speed)
            sleep(0.5)
            pz.stop()
            #break
        elif blueMsg == "down":
            pz.reverse(speed)
            sleep(0.5)
            pz.stop

        elif blueMsg=="right":
            pz.spinRight(speed)
            sleep(0.5)
            pz.stop()

        elif blueMsg=="left":
            pz.spinLeft(speed)
            sleep(0.5)
            pz.stop()
        else:
            pz.stop()
        sleep(0.1)

class keyBoardControl(threading.Thread):
    def run(self):
        return
    def readKey(self,target,controlTarget):
        while True:
            try:
                keyp=pz.readkey()
                target.printMsg(keyp)
	        if keyp == 'w' or ord(keyp) == 16:
        		controlTarget.forward(speed)
            	#	print('Forward', speed)
        	elif keyp == 'z' or ord(keyp) == 17:
            		controlTarget.reverse(speed)
            	#	print('Reverse', speed)
        	elif keyp == 's' or ord(keyp) == 18:
            		controlTarget.right(speed)
            	#	print('Spin Right', speed)
        	elif keyp == 'a' or ord(keyp) == 19:
            		controlTarget.left(speed)
            	#	print('Spin Left', speed)
        	#elif keyp == '.' or keyp == '>':
           	#	speed = min(100, speed+10)
            	#	print('Speed+', speed)
        	#elif keyp == ',' or keyp == '<':
            	#	speed = max (0, speed-10)
            	#	print('Speed-', speed)
        	#elif keyp == ' ':
            	#	pz.stop()
            	#	print('Stop')
        	elif ord(keyp) == 3:
            		break
               
 
            except KeyboardInterrupt:
                break

class displayMsg(threading.Thread):
    def run(self):
        return
        
    def printMsg(self,msg):
        print(msg)

class robotMove():
    def forward(self,speed):
        pz.forward(speed)
    def reverse(self, speed):
        pz.reverse(speed)
    def left(self,speed):
        pz.spinLeft(speed)
    def right(self,speed):
        pz.spinRight(speed)
    def stop():
        pz.stop()


print('Connected')
d = threading.Thread(name='daemon', target=blueConnection)
d.setDaemon(True)
screenMessage=displayMsg()
kb=keyBoardControl()
conn=robotMove()

try:
        pz.init()
	imu = imu.imuSystem(bus)
	imu.getBearing()
        
except:
	print "IMU sensor not found"
	sys.exit()

try:
	d.start()
	screenMessage.start()
        kb.start()
        kb.readKey(screenMessage,conn)

   	while True:
	#	("Still running")
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
