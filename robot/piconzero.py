# Python library for 4tronix Picon Zero
# Note that all I2C accesses are wrapped in try clauses with repeats

import smbus, time
from time import sleep

bus = smbus.SMBus(1) # For revision 1 Raspberry Pi, change to bus = smbus.SMBus(0)

pzaddr = 0x22 # I2C address of Picon Zero

#---------------------------------------------
# Definitions of Commands to Picon Zero
MOTORA = 0
OUTCFG0 = 2
OUTPUT0 = 8
INCFG0 = 14
SETBRIGHT = 18
UPDATENOW = 19
RESET = 20
INPERIOD0 = 21
#---------------------------------------------

#---------------------------------------------
# General variables
DEBUG = False
RETRIES = 10   # max number of retries for I2C calls
#---------------------------------------------

#======================================================================
# Reading single character by forcing stdin to raw mode
import sys
import tty
import termios
import imu

imu=imu.imuSystem(bus)

def readchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    if ch == '0x03':
        raise KeyboardInterrupt
    return ch

def readkey(getchar_fn=None):
    getchar = getchar_fn or readchar
    c1 = getchar()
    if ord(c1) != 0x1b:
        return c1
    c2 = getchar()
    if ord(c2) != 0x5b:
        return c1
    c3 = getchar()
    return chr(0x10 + ord(c3) - 65)  # 16=Up, 17=Down, 18=Right, 19=Left arrows

# End of single character reading
#======================================================================

def moveRobot(keyp,speed):
    if keyp == 'w' or ord(keyp) == 16:
        forward(speed)
        print('Forward', speed)
    elif keyp == 'z' or ord(keyp) == 17:
        reverse(speed)
        print('Reverse', speed)
    elif keyp == 's' or ord(keyp) == 18:
        spinRight(speed)
        print('Spin Right', speed,imu.getBearing())
    elif keyp == 'a' or ord(keyp) == 19:
        spinLeft(speed)
        print('Spin Left', speed)
    elif keyp == '.' or keyp == '>':
        speed = min(100, speed+10)
        print('Speed+', speed)
    elif keyp == ',' or keyp == '<':
        speed = max (0, speed-10)
        print('Speed-', speed)
    elif keyp == ' ':
        stop()
        print('Stop')
    elif keyp == "x":
        setHeading(90,speed)
    elif keyp == "b":
        print(int(imu.getBearing()))
    #lif ord(keyp) == 3:
      # break



def setHeading(requiredHeading,speed):
    while True:
        bearing = int(imu.getBearing())
        print(bearing)
        
        if bearing != requiredHeading:
            spinRight(50)
            sleep(0.01)
            stop()
            #sleep(0.1)
        else:
            break
        #sleep(0.1)


#---------------------------------------------
# Get Version and Revision info
def getRevision():
    for i in range(RETRIES):
        try:
            rval = bus.read_word_data (pzaddr, 0)
            return [rval/256, rval%256]
        except:
            if (DEBUG):
                print("Error in getRevision(), retrying")
#---------------------------------------------


#---------------------------------------------
# motor must be in range 0..1
# value must be in range -128 - +127
# values of -127, -128, +127 are treated as always ON,, no PWM
def setMotor (motor, value):
    if (motor>=0 and motor<=1 and value>=-128 and value<128):
        for i in range(RETRIES):
            try:
                bus.write_byte_data (pzaddr, motor, value)
                break
            except:
                if (DEBUG):
                    print("Error in setMotor(), retrying")

def forward (speed):
    setMotor (0, speed)
    setMotor (1, speed)

def reverse (speed):
    setMotor (0, -speed)
    setMotor (1, -speed)

def spinLeft (speed):
    setMotor (0, speed)
    setMotor (1, -speed)

def spinRight (speed):
    setMotor (0, -speed)
    setMotor (1, speed)

def stop():
    setMotor (0, 0)
    setMotor (1, 0)

#---------------------------------------------

#---------------------------------------------
# Read data for selected input channel (analog or digital)
# Channel is in range 0 to 3
def readInput (channel):
    if (channel>=0 and channel <=3):
        for i in range(RETRIES):
            try:
                return bus.read_word_data (pzaddr, channel + 1)
            except:
                if (DEBUG):
                    print("Error in readChannel(), retrying")

#---------------------------------------------

#---------------------------------------------
# Set configuration of selected output
# 0: On/Off, 1: PWM, 2: Servo, 3: WS2812B
def setOutputConfig (output, value):
    if (output>=0 and output<=5 and value>=0 and value<=3):
        for i in range(RETRIES):
            try:
                bus.write_byte_data (pzaddr, OUTCFG0 + output, value)
                break
            except:
                if (DEBUG):
                    print("Error in setOutputConfig(), retrying")
#---------------------------------------------

#---------------------------------------------
# Set configuration of selected input channel
# 0: Digital, 1: Analog, 2: DS18B20, 4: DutyCycle 5: Pulse Width
def setInputConfig (channel, value, pullup = False, period = 2000):
    if (channel >= 0 and channel <= 3 and value >= 0 and value <= 5):
        if (value == 0 and pullup == True):
            value = 128
        for i in range(RETRIES):
            try:
                bus.write_byte_data (pzaddr, INCFG0 + channel, value)
                if (value == 4 or value == 5):
                    bus.write_word_data (pzaddr, INPERIOD0 + channel, period)
                break
            except:
                if (DEBUG):
                    print("Error in setInputConfig(), retrying")
#---------------------------------------------

#---------------------------------------------
# Set output data for selected output channel
# Mode  Name    Type    Values
# 0     On/Off  Byte    0 is OFF, 1 is ON
# 1     PWM     Byte    0 to 100 percentage of ON time
# 2     Servo   Byte    -100 to + 100 Position in degrees
# 3     WS2812B 4 Bytes 0:Pixel ID, 1:Red, 2:Green, 3:Blue
def setOutput (channel, value):
    if (channel>=0 and channel<=5):
        for i in range(RETRIES):
            try:
                bus.write_byte_data (pzaddr, OUTPUT0 + channel, value)
                break
            except:
                if (DEBUG):
                    print("Error in setOutput(), retrying")
#---------------------------------------------

#---------------------------------------------
# Set the colour of an individual pixel (always output 5)
def setPixel (Pixel, Red, Green, Blue, Update=True):
    pixelData = [Pixel, Red, Green, Blue]
    for i in range(RETRIES):
        try:
            bus.write_i2c_block_data (pzaddr, Update, pixelData)
            break
        except:
            if (DEBUG):
                print("Error in setPixel(), retrying")

def setAllPixels (Red, Green, Blue, Update=True):
    pixelData = [100, Red, Green, Blue]
    for i in range(RETRIES):
        try:
            bus.write_i2c_block_data (pzaddr, Update, pixelData)
            break
        except:
            if (DEBUG):
                print("Error in setAllPixels(), retrying")

def updatePixels ():
    for i in range(RETRIES):
        try:
            bus.write_byte_data (pzaddr, UPDATENOW, 0)
            break
        except:
            if (DEBUG):
                print("Error in updatePixels(), retrying")

#---------------------------------------------

#---------------------------------------------
# Set the overall brightness of pixel array
def setBrightness (brightness):
    for i in range(RETRIES):
        try:
            bus.write_byte_data (pzaddr, SETBRIGHT, brightness)
            break
        except:
            if (DEBUG):
                print("Error in setBrightness(), retrying")
#---------------------------------------------

#---------------------------------------------
# Initialise the Board (same as cleanup)
def init (debug=False):
    DEBUG = debug
    for i in range(RETRIES):
        try:
            bus.write_byte_data (pzaddr, RESET, 0)
            break
        except:
            if (DEBUG):
                print("Error in init(), retrying")
    time.sleep(0.01)  #1ms delay to allow time to complete
    if (DEBUG):
        print("Debug is", DEBUG)
#---------------------------------------------

#---------------------------------------------
# Cleanup the Board (same as init)
def cleanup ():
    for i in range(RETRIES):
        try:
            bus.write_byte_data (pzaddr, RESET, 0)
            break
        except:
            if (DEBUG):
                print("Error in cleanup(), retrying")
    time.sleep(0.001)   # 1ms delay to allow time to complete
#---------------------------------------------
