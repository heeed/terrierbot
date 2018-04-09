import bluetooth
import smbus,imu
bus=smbus.SMBus(1)
imu=imu.imuSystem(bus)
bd_addr = "00:14:03:06:1E:DF" 
port = 1
sock = bluetooth.BluetoothSocket (bluetooth.RFCOMM)
sock.connect((bd_addr,port))

while 1:
    tosend = str(imu.getBearing()) 
    print(str(imu.getBearing()))
    if tosend != 'q':
        sock.send(tosend)
    else:
        break
        
