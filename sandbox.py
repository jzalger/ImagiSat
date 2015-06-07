"""
sandbox.py is a script for testing and prototyping ImagiSat functionality.
"""
__author__ = 'jzalger'

import mraa
import serial
from pynmea import nmea

# Read a GPIO Pin
pin = mraa.Gpio(31)
pin.dir(mraa.DIR_IN)
pwrStatus = pin.read()

##################################################
# Serial
##################################################
# Query GPS
gpsSerial = serial.Serial('/dev/ttyMFD1', 9600)  # MFD1 corresponds to UART1
msg = gpsSerial.readline()  # Reads some characters

# Parse the GPS coorginates.
# Borrowed from https://www.sparkfun.com/tutorials/403

gpgga = nmea.GPGGA()

# Check if its a location message
while True:
    msg = gpsSerial.readline()
    if 'GPGGA' in msg:
        gpgga.parse(msg)

        lats = gpgga.latitude
        longs = gpgga.longitude

        # convert degrees,decimal minutes to decimal degrees
        lat1 = (float(lats[2]+lats[3]+lats[4]+lats[5]+lats[6]+lats[7]+lats[8]))/60
        lat = (float(lats[0]+lats[1])+lat1)
        long1 = (float(longs[3]+longs[4]+longs[5]+longs[6]+longs[7]+longs[8]+longs[9]))/60
        long = (float(longs[0]+longs[1]+longs[2])+long1)

        # Do something with this usefulness

##################################################
# PWM
##################################################
pwmPin = mraa.Pwm(21)  # MRAA pin number, eg 21 = PWM 3
pwmPin.period_us(700)  # Define the duty cycle period in microseconds
pwmPin.enable(True)    # Activate the pin
pwmPin.write(0.5)      # Write value betwen 0 and 1
pwmPin.enable(False)

##################################################
# I2C
##################################################

# BMP085 Sensor Data
import Adafruit_BMP.BMP085 as BMP085
sensor = BMP085.BMP085()
sensor.read_temperature()
sensor.read_pressure()

# MPR121
import Adafruit_MPR121.MPR121 as MPR121
cap = MPR121.MPR121()
if not cap.begin():
    print "Badness"
while True:
    if cap.is_touched(5):
        print "Awesome"

# ADXL345 using UPM
import pyupm_adxl345
acclSensor = pyupm_adxl345.Adxl345(1) # 1 is the i2c bus number
acclSensor.update()
a = acclSensor.getAcceleration()  # Returns a list of floats [x, y, z]

# HMC58831 Digital Compas
import pyupm_hmc5883l
compas = pyupm_hmc5883l.Hmc5883l(1)  # 1 is the i2c bus number
compas.update()
heading = compas.heading()
direction = compas.direction()
coordinates = compas.coordinates()  # list of ints [x, y, z] with

# SHT 75
import Sht1x
sht1x = Sht1x.Sht1x(7, 19)       # Using the 3.3V I2C bus
temp = sht1x.read_temperature_C()
humidity = sht1x.read_humidity()

# 16ch PCA9685 PWM Board
import pyupm_pca9685 as pca
pwm = pca.PCA9685(1, 0x40)      # (bus number, device address)
pwm.ledFullOn(1, False)         # The boolean logic was reversed for some reason
pwm.ledFullOff(1, False)