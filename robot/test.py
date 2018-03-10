import smbus,imu
bus=smbus.SMBus(1)
imu=imu.imuSystem(bus)
imu.getBearing()

