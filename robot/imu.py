#imu.py
#
# holding class and settings for LSM9DS0 chip on ozzmaker IMU board
#
# Contents sourced from ozzmaker examples

import math,datetime
from time import sleep

MAG_ADDRESS	=	0x1E			
ACC_ADDRESS	=	0x1E			
GYR_ADDRESS     =       0x6A

#LSM9DS0 Gyro Registers	
WHO_AM_I_G	=	0x0F			
CTRL_REG1_G	=	0x20			
CTRL_REG2_G	=	0x21			
CTRL_REG3_G	=	0x22			
CTRL_REG4_G	=	0x23			
CTRL_REG5_G	=	0x24			
REFERENCE_G	=	0x25			
STATUS_REG_G	=	0x27			
OUT_X_L_G	=	0x28			
OUT_X_H_G	=	0x29			
OUT_Y_L_G	=	0x2A			
OUT_Y_H_G	=	0x2B			
OUT_Z_L_G	=	0x2C			
OUT_Z_H_G	=	0x2D			
FIFO_CTRL_REG_G	=	0x2E			
FIFO_SRC_REG_G	=	0x2F			
INT1_CFG_G	=	0x30			
INT1_SRC_G	=	0x31			
INT1_THS_XH_G	=	0x32			
INT1_THS_XL_G	=	0x33			
INT1_THS_YH_G	=	0x34			
INT1_THS_YL_G	=	0x35			
INT1_THS_ZH_G	=	0x36			
INT1_THS_ZL_G	=	0x37			
INT1_DURATION_G	=	0x38			

#LSM9DS0 Accel and Magneto Registers
OUT_TEMP_L_XM	=	0x05			
OUT_TEMP_H_XM	=	0x06			
STATUS_REG_M	=	0x07			
OUT_X_L_M	=	0x08			
OUT_X_H_M	=	0x09			
OUT_Y_L_M	=	0x0A			
OUT_Y_H_M	=	0x0B			
OUT_Z_L_M	=	0x0C			
OUT_Z_H_M	=	0x0D			
WHO_AM_I_XM	=	0x0F			
INT_CTRL_REG_M	=	0x12			
INT_SRC_REG_M	=	0x13			
INT_THS_L_M	=	0x14			
INT_THS_H_M	=	0x15			
OFFSET_X_L_M	=	0x16			
OFFSET_X_H_M	=	0x17			
OFFSET_Y_L_M	=	0x18			
OFFSET_Y_H_M	=	0x19			
OFFSET_Z_L_M	=	0x1A			
OFFSET_Z_H_M	=	0x1B			
REFERENCE_X	=	0x1C			
REFERENCE_Y	=	0x1D			
REFERENCE_Z	=	0x1E			
CTRL_REG0_XM	=	0x1F			
CTRL_REG1_XM	=	0x20			
CTRL_REG2_XM	=	0x21			
CTRL_REG3_XM	=	0x22			
CTRL_REG4_XM	=	0x23			
CTRL_REG5_XM	=	0x24			
CTRL_REG6_XM	=	0x25			
CTRL_REG7_XM	=	0x26			
STATUS_REG_A	=	0x27			
OUT_X_L_A	=	0x28			
OUT_X_H_A	=	0x29			
OUT_Y_L_A	=	0x2A			
OUT_Y_H_A	=	0x2B			
OUT_Z_L_A	=	0x2C			
OUT_Z_H_A	=	0x2D			
FIFO_CTRL_REG	=	0x2E			
FIFO_SRC_REG	=	0x2F			
INT_GEN_1_REG	=	0x30			
INT_GEN_1_SRC	=	0x31			
INT_GEN_1_THS	=	0x32			
INT_GEN_1_DURATION	=	0x33			
INT_GEN_2_REG	=	0x34			
INT_GEN_2_SRC	=	0x35			
INT_GEN_2_THS	=	0x36			
INT_GEN_2_DURATION	=	0x37			
CLICK_CFG	=	0x38			
CLICK_SRC	=	0x39			
CLICK_THS	=	0x3A			
TIME_LIMIT	=	0x3B			
TIME_LATENCY	=	0x3C			
TIME_WINDOW	=	0x3D			
#Kalman filter variables
Q_angle = 0.02
Q_gyro = 0.0015
R_angle = 0.005
y_bias = 0.0
x_bias = 0.0
XP_00 = 0.0
XP_01 = 0.0
XP_10 = 0.0
XP_11 = 0.0
YP_00 = 0.0
YP_01 = 0.0
YP_10 = 0.0
YP_11 = 0.0
KFangleX = 0.0
KFangleY = 0.0
MAG_LPF_FACTOR = 0.4    # Low pass filter constant magnetometer
ACC_LPF_FACTOR = 0.4    # Low pass filter constant for accelerometer
ACC_MEDIANTABLESIZE = 9         # Median filter table size for accelerometer. Higher = smoother but a longer delay
MAG_MEDIANTABLESIZE = 9
RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
G_GAIN = 0.070
AA =  0.40
IMU_upside_down = 0
gyroXangle = 0.0
gyroYangle = 0.0
gyroZangle = 0.0
CFangleX = 0.0
CFangleY = 0.0
CFangleXFiltered = 0.0
CFangleYFiltered = 0.0
kalmanX = 0.0
kalmanY = 0.0
oldXMagRawValue = 0
oldYMagRawValue = 0
oldZMagRawValue = 0
oldXAccRawValue = 0
oldYAccRawValue = 0
oldZAccRawValue = 0
a = 0
#Setup the tables for the mdeian filter. Fill them all with '1' soe we dont get device by zero error
acc_medianTable1X = [1] * ACC_MEDIANTABLESIZE
acc_medianTable1Y = [1] * ACC_MEDIANTABLESIZE
acc_medianTable1Z = [1] * ACC_MEDIANTABLESIZE
acc_medianTable2X = [1] * ACC_MEDIANTABLESIZE
acc_medianTable2Y = [1] * ACC_MEDIANTABLESIZE
acc_medianTable2Z = [1] * ACC_MEDIANTABLESIZE
mag_medianTable1X = [1] * MAG_MEDIANTABLESIZE
mag_medianTable1Y = [1] * MAG_MEDIANTABLESIZE
mag_medianTable1Z = [1] * MAG_MEDIANTABLESIZE
mag_medianTable2X = [1] * MAG_MEDIANTABLESIZE
mag_medianTable2Y = [1] * MAG_MEDIANTABLESIZE
mag_medianTable2Z = [1] * MAG_MEDIANTABLESIZE


class imuSystem:

	def __init__(self,bus):

		self.bus = bus
                #initialise the accelerometer
                self.writeACC(CTRL_REG1_XM, 0b01100111) #z,y,x axis enabled, continuos update,  100Hz data rate
                self.writeACC(CTRL_REG2_XM, 0b00100000) #+/- 16G full scale

                #initialise the magnetometer
                self.writeMAG(CTRL_REG5_XM, 0b11110000) #Temp enable, M data rate = 50Hz
                self.writeMAG(CTRL_REG6_XM, 0b01100000) #+/-12gauss
                self.writeMAG(CTRL_REG7_XM, 0b00000000) #Continuous-conversion mode

                #initialise the gyroscope
                self.writeGRY(CTRL_REG1_G, 0b00001111) #Normal power mode, all axes enabled
                self.writeGRY(CTRL_REG4_G, 0b00110000) #Continuos update, 2000 dps full scale

	def kalmanFilterY (self,accAngle, gyroRate, DT):
        	y=0.0
        	S=0.0

        	global KFangleY
        	global Q_angle
        	global Q_gyro
        	global y_bias
        	global YP_00
        	global YP_01
        	global YP_10
        	global YP_11

        	KFangleY = KFangleY + DT * (gyroRate - y_bias)

        	YP_00 = YP_00 + ( - DT * (YP_10 + YP_01) + Q_angle * DT )
        	YP_01 = YP_01 + ( - DT * YP_11 )
        	YP_10 = YP_10 + ( - DT * YP_11 )
        	YP_11 = YP_11 + ( + Q_gyro * DT )

        	y = accAngle - KFangleY
        	S = YP_00 + R_angle
        	K_0 = YP_00 / S
        	K_1 = YP_10 / S

        	KFangleY = KFangleY + ( K_0 * y )
        	y_bias = y_bias + ( K_1 * y )

        	YP_00 = YP_00 - ( K_0 * YP_00 )
        	YP_01 = YP_01 - ( K_0 * YP_01 )
        	YP_10 = YP_10 - ( K_1 * YP_00 )
        	YP_11 = YP_11 - ( K_1 * YP_01 )

        	return KFangleY

	def kalmanFilterX (self,accAngle, gyroRate, DT):
	        x=0.0
        	S=0.0

        	global KFangleX
        	global Q_angle
        	global Q_gyro
        	global x_bias
        	global XP_00
        	global XP_01
        	global XP_10
        	global XP_11


        	KFangleX = KFangleX + DT * (gyroRate - x_bias)

        	XP_00 = XP_00 + ( - DT * (XP_10 + XP_01) + Q_angle * DT )
        	XP_01 = XP_01 + ( - DT * XP_11 )
        	XP_10 = XP_10 + ( - DT * XP_11 )
        	XP_11 = XP_11 + ( + Q_gyro * DT )

        	x = accAngle - KFangleX
        	S = XP_00 + R_angle
        	K_0 = XP_00 / S
        	K_1 = XP_10 / S

        	KFangleX = KFangleX + ( K_0 * x )
        	x_bias = x_bias + ( K_1 * x )

        	XP_00 = XP_00 - ( K_0 * XP_00 )
        	XP_01 = XP_01 - ( K_0 * XP_01 )
        	XP_10 = XP_10 - ( K_1 * XP_00 )
        	XP_11 = XP_11 - ( K_1 * XP_01 )

        	return KFangleX

	def writeACC(self,register,value):
        	self.bus.write_byte_data(ACC_ADDRESS , register, value)
        	return -1

	def writeMAG(self,register,value):
        	self.bus.write_byte_data(MAG_ADDRESS, register, value)
        	return -1

	def writeGRY(self,register,value):
        	self.bus.write_byte_data(GYR_ADDRESS, register, value)
        	return -1

	def readACCx(self):
        	acc_l = self.bus.read_byte_data(ACC_ADDRESS, OUT_X_L_A)
        	acc_h = self.bus.read_byte_data(ACC_ADDRESS, OUT_X_H_A)
        	acc_combined = (acc_l | acc_h <<8)

        	return acc_combined  if acc_combined < 32768 else acc_combined - 65536


	def readACCy(self):
        	acc_l = self.bus.read_byte_data(ACC_ADDRESS, OUT_Y_L_A)
        	acc_h = self.bus.read_byte_data(ACC_ADDRESS, OUT_Y_H_A)
        	acc_combined = (acc_l | acc_h <<8)

        	return acc_combined  if acc_combined < 32768 else acc_combined - 65536


	def readACCz(self):
        	acc_l = self.bus.read_byte_data(ACC_ADDRESS, OUT_Z_L_A)
        	acc_h = self.bus.read_byte_data(ACC_ADDRESS, OUT_Z_H_A)
        	acc_combined = (acc_l | acc_h <<8)

        	return acc_combined  if acc_combined < 32768 else acc_combined - 65536


	def readMAGx(self):
        	mag_l = self.bus.read_byte_data(MAG_ADDRESS, OUT_X_L_M)
        	mag_h = self.bus.read_byte_data(MAG_ADDRESS, OUT_X_H_M)
        	mag_combined = (mag_l | mag_h <<8)

        	return mag_combined  if mag_combined < 32768 else mag_combined - 65536

	def readMAGy(self):
        	mag_l = self.bus.read_byte_data(MAG_ADDRESS, OUT_Y_L_M)
        	mag_h = self.bus.read_byte_data(MAG_ADDRESS, OUT_Y_H_M)
        	mag_combined = (mag_l | mag_h <<8)

        	return mag_combined  if mag_combined < 32768 else mag_combined - 65536


	def readMAGz(self):
        	mag_l = self.bus.read_byte_data(MAG_ADDRESS, OUT_Z_L_M)
        	mag_h = self.bus.read_byte_data(MAG_ADDRESS, OUT_Z_H_M)
        	mag_combined = (mag_l | mag_h <<8)

        	return mag_combined  if mag_combined < 32768 else mag_combined - 65536

	def readGYRx(self):
        	gyr_l = self.bus.read_byte_data(GYR_ADDRESS, OUT_X_L_G)
        	gyr_h = self.bus.read_byte_data(GYR_ADDRESS, OUT_X_H_G)
        	gyr_combined = (gyr_l | gyr_h <<8)

        	return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536

	def readGYRy(self):
        	gyr_l = self.bus.read_byte_data(GYR_ADDRESS, OUT_Y_L_G)
        	gyr_h = self.bus.read_byte_data(GYR_ADDRESS, OUT_Y_H_G)
        	gyr_combined = (gyr_l | gyr_h <<8)

        	return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536

	def readGYRz(self):
        	gyr_l = self.bus.read_byte_data(GYR_ADDRESS, OUT_Z_L_G)
        	gyr_h = self.bus.read_byte_data(GYR_ADDRESS, OUT_Z_H_G)
        	gyr_combined = (gyr_l | gyr_h <<8)

        	return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536	

	def readMagneometer(self):
		MAGx = self.readMAGx()
        	MAGy = self.readMAGy()
       		MAGz = self.readMAGz()
		return MAGx

	def readACCx(self):
        	acc_l = self.bus.read_byte_data(ACC_ADDRESS, OUT_X_L_A)
                acc_h = self.bus.read_byte_data(ACC_ADDRESS, OUT_X_H_A)
                acc_combined = (acc_l | acc_h <<8)

                return acc_combined  if acc_combined < 32768 else acc_combined - 65536

	def readACCy(self):
                acc_l = self.bus.read_byte_data(ACC_ADDRESS, OUT_Y_L_A)
                acc_h = self.bus.read_byte_data(ACC_ADDRESS, OUT_Y_H_A)
                acc_combined = (acc_l | acc_h <<8)

                return acc_combined  if acc_combined < 32768 else acc_combined - 65536

	def readACCx(self):
                acc_l = self.bus.read_byte_data(ACC_ADDRESS, OUT_X_L_A)
                acc_h = self.bus.read_byte_data(ACC_ADDRESS, OUT_X_H_A)
                acc_combined = (acc_l | acc_h <<8)

                return acc_combined  if acc_combined < 32768 else acc_combined - 65536

        def readACCy(self):
        	acc_l = self.bus.read_byte_data(ACC_ADDRESS, OUT_Y_L_A)
                acc_h = self.bus.read_byte_data(ACC_ADDRESS, OUT_Y_H_A)
                acc_combined = (acc_l | acc_h <<8)

                return acc_combined  if acc_combined < 32768 else acc_combined - 65536

	def test(self):
		print("This is a test message")
	
	def getBearing(self):

		global a
	        global MAG_LPF_FACTOR 
		global ACC_LPF_FACTOR 

                global acc_medianTable1X
                global acc_medianTable1Y
                global acc_medianTable1Z
                global acc_medianTable2X
                global acc_medianTable2Y
                global acc_medianTable2Z
                global mag_medianTable1X
                global mag_medianTable1Y
                global mag_medianTable1Z
                global mag_medianTable2X
                global mag_medianTable2Y
                global mag_medianTable2Z


		a = datetime.datetime.now()

		ACCx = self.readACCx()
        	ACCy = self.readACCy()
		ACCz = self.readACCz()
        	GYRx = self.readGYRx()
        	GYRy = self.readGYRy()
        	GYRz = self.readGYRz()
                MAGx = self.readMAGx()
                MAGy = self.readMAGy()
                MAGz = self.readMAGz()

		
		##Calculate loop Period(LP). How long between Gyro Reads
	        b = datetime.datetime.now() - a
        	a = datetime.datetime.now()
        	LP = b.microseconds/(1000000*1.0)
        	print "Loop Time | %5.2f|" % ( LP ),

		ACC_MEDIANTABLESIZE = 9         # Median filter table size for accelerometer. Higher = smoother but a longer delay
        	MAG_MEDIANTABLESIZE = 9
		RAD_TO_DEG = 57.29578
	        M_PI = 3.14159265358979323846
		G_GAIN = 0.070
		AA =  0.40
		IMU_upside_down = 0
		gyroXangle = 0.0
		gyroYangle = 0.0
		gyroZangle = 0.0
		CFangleX = 0.0
		CFangleY = 0.0
		CFangleXFiltered = 0.0
		CFangleYFiltered = 0.0
		kalmanX = 0.0
		kalmanY = 0.0
		oldXMagRawValue = 0
		oldYMagRawValue = 0
		oldZMagRawValue = 0
		oldXAccRawValue = 0
		oldYAccRawValue = 0
		oldZAccRawValue = 0
		
		############################################ 
        	### Apply low pass filter ####
        	##############################################
		MAGx =  MAGx  * MAG_LPF_FACTOR + oldXMagRawValue*(1 - MAG_LPF_FACTOR);
		MAGy =  MAGy  * MAG_LPF_FACTOR + oldYMagRawValue*(1 - MAG_LPF_FACTOR);
		MAGz =  MAGz  * MAG_LPF_FACTOR + oldZMagRawValue*(1 - MAG_LPF_FACTOR);
		ACCx =  ACCx  * ACC_LPF_FACTOR + oldXAccRawValue*(1 - ACC_LPF_FACTOR);
		ACCy =  ACCy  * ACC_LPF_FACTOR + oldYAccRawValue*(1 - ACC_LPF_FACTOR);
		ACCz =  ACCz  * ACC_LPF_FACTOR + oldZAccRawValue*(1 - ACC_LPF_FACTOR);

		oldXMagRawValue = MAGx
        	oldYMagRawValue = MAGy
        	oldZMagRawValue = MAGz
        	oldXAccRawValue = ACCx
        	oldYAccRawValue = ACCy
        	oldZAccRawValue = ACCz

		#########################################
        	#### Median filter for accelerometer ####
        	#########################################
        	# cycle the table
        	for x in range (ACC_MEDIANTABLESIZE-1,0,-1 ):
                	acc_medianTable1X[x] = acc_medianTable1X[x-1]
                	acc_medianTable1Y[x] = acc_medianTable1Y[x-1]
                	acc_medianTable1Z[x] = acc_medianTable1Z[x-1]

        	# Insert the lates values
        	acc_medianTable1X[0] = ACCx
        	acc_medianTable1Y[0] = ACCy
        	acc_medianTable1Z[0] = ACCz

        	# Copy the tables
        	acc_medianTable2X = acc_medianTable1X[:]
        	acc_medianTable2Y = acc_medianTable1Y[:]
        	acc_medianTable2Z = acc_medianTable1Z[:]

       		# Sort table 2
        	acc_medianTable2X.sort()
        	acc_medianTable2Y.sort()
        	acc_medianTable2Z.sort()
	
		# The middle value is the value we are interested in
        	ACCx = acc_medianTable2X[ACC_MEDIANTABLESIZE/2];
        	ACCy = acc_medianTable2Y[ACC_MEDIANTABLESIZE/2];
        	ACCz = acc_medianTable2Z[ACC_MEDIANTABLESIZE/2];

		 #Normalize accelerometer raw values.
        	accXnorm = ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
        	accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)

		   # Change the rotation value of the accelerometer to -/+ 180 and move the Y axis '0' point to up

 	#Convert Gyro raw to degrees per second
        	rate_gyr_x =  GYRx * G_GAIN
        	rate_gyr_y =  GYRy * G_GAIN
        	rate_gyr_z =  GYRz * G_GAIN


        #Calculate the angles from the gyro. 
        	gyroXangle+=rate_gyr_x*LP
        	gyroYangle+=rate_gyr_y*LP
        	gyroZangle+=rate_gyr_z*LP


        ##Convert Accelerometer values to degrees
        	AccXangle =  (math.atan2(ACCy,ACCz)+M_PI)*RAD_TO_DEG
       		AccYangle =  (math.atan2(ACCz,ACCx)+M_PI)*RAD_TO_DEG

		
        # Change the rotation value of the accelerometer to -/+ 180 and move the Y axis '0' point to up
        # Two different pieces of code are used depending on how your IMU is mounted
		if IMU_upside_down :            # If IMU is upside down E.g Skull logo is facing up.
                	if AccXangle >180:
                        	AccXangle -= 360.0
                        	AccYangle-=90
                	if AccYangle >180:
                        	AccYangle -= 360.0

        	else :                                          # If IMU is up the correct way E.g Skull logo is facing down.
                	AccXangle -= 180.0
                	if AccYangle > 90:
                        	AccYangle -= 270.0
                	else:
                        	AccYangle += 90.0



        #Complementary filter used to combine the accelerometer and gyro values.
        	CFangleX=AA*(CFangleX+rate_gyr_x*LP) +(1 - AA) * AccXangle
        	CFangleY=AA*(CFangleY+rate_gyr_y*LP) +(1 - AA) * AccYangle

        #Kalman filter used to combine the accelerometer and gyro values.
        	kalmanY = self.kalmanFilterY(AccYangle, rate_gyr_y,LP)
        	kalmanX = self.kalmanFilterX(AccXangle, rate_gyr_x,LP)


		#Calculate pitch and roll
	        if IMU_upside_down :
        	        accXnorm = -accXnorm                            #flip Xnorm as the IMU is upside down
                	accYnorm = -accYnorm                            #flip Ynorm as the IMU is upside down
                	pitch = math.asin(accXnorm)
                	roll = math.asin(accYnorm/math.cos(pitch))
        	else :
                	pitch = math.asin(accXnorm)
                	roll = -math.asin(accYnorm/math.cos(pitch))


                if IMU_upside_down :
                	MAGy = -MAGy

                #Calculate heading
                heading = 180 * math.atan2(MAGy,MAGx)/M_PI

                #Only have our heading between 0 and 360
                if heading < 0:
                    heading += 360


		#Calculate the new tilt compensated values
	        magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
        	magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)

       		#Calculate tilt compensated heading
        	tiltCompensatedHeading = 180 * math.atan2(magYcomp,magXcomp)/M_PI

        	#Only have our heading between 0 and 360
        	if tiltCompensatedHeading < 0:
                	tiltCompensatedHeading += 360
	        sleep(0.1)	
		return(int(tiltCompensatedHeading))
                #return(int(heading))
