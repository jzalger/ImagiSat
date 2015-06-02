'''
Created on Oct 5, 2012

@author: Luca Nobili

This modules reads Humidity and Temperature from a Sensirion SHT1x sensor. I has been tested
both with an SHT11 and an SHT15.

It is meant to be used in a Raspberry Pi and depends on this module (http://code.google.com/p/raspberry-gpio-python/).

The module raspberry-gpio-python requires root privileges, therefore, to run this module you need to run your script as root.


Example Usage:

>>> from sht1x.Sht1x import Sht1x as SHT1x
>>> sht1x = SHT1x(11,7)
>>> sht1x.read_temperature_C()
25.22
>>> sht1x.read_humidity()     
52.6564216

###
Updated March 2015 by jzalger. Adapted for Intel Edison using the MRAA library
###

'''
import traceback
import sys
import time
import logging
import math

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

import mraa
 
#   Conversion coefficients from SHT15 datasheet
D1 = -40.0  # for 14 Bit @ 5V
D2 = 0.01   # for 14 Bit DEGC

C1 = -2.0468        # for 12 Bit
C2 = 0.0367         # for 12 Bit
C3 = -0.0000015955  # for 12 Bit
T1 = 0.01      # for 14 Bit @ 5V
T2 = 0.00008   # for 14 Bit @ 5V
    
class Sht1x(object):

    def __init__(self, dataPin, sckPin):
        self.dataPin = mraa.Gpio(dataPin)
        self.sckPin = mraa.Gpio(sckPin)

    def read_temperature_C(self):
        temperatureCommand = 0b00000011

        self.__sendCommand(temperatureCommand)
        self.__waitForResult()
        rawTemperature = self.__getData16Bit()
        self.__skipCrc()
        # GPIO.cleanup() Not sure what this does
        return rawTemperature * D2 + D1

    def read_humidity(self):
        """Get current temperature for humidity correction"""
        temperature = self.read_temperature_C()
        return self._read_humidity(temperature)
    
    def _read_humidity(self, temperature):
        humidityCommand = 0b00000101
        self.__sendCommand(humidityCommand)
        self.__waitForResult()
        rawHumidity = self.__getData16Bit()
        self.__skipCrc()
        # GPIO.cleanup()

        # Apply linear conversion to raw value
        linearHumidity = C1 + C2 * rawHumidity + C3 * rawHumidity * rawHumidity
        # Correct humidity value for current temperature
        return (temperature - 25.0) * (T1 + T2 * rawHumidity) + linearHumidity

    def calculate_dew_point(self, temperature, humidity):
        if temperature > 0:
            tn = 243.12
            m = 17.62
        else:
            tn = 272.62
            m = 22.46
        return tn * (math.log(humidity / 100.0) + (m * temperature) / (tn + temperature)) / (m - math.log(humidity / 100.0) - m * temperature / (tn + temperature))

    def __sendCommand(self, command):
        # Transmission start
        self.dataPin.dir(mraa.DIR_OUT)
        self.sckPin.dir(mraa.DIR_OUT)

        self.dataPin.write(1)
        self.__clockTick(1)
        self.dataPin.write(0)
        self.__clockTick(0)
        self.__clockTick(1)
        self.dataPin.write(1)
        self.__clockTick(0)

        for i in range(8):
            # GPIO.output(self.dataPin, command & (1 << 7 - i))
            self.dataPin.write(command & (1 << 7 - i))
            self.__clockTick(1)
            self.__clockTick(0)

        self.__clockTick(1)

        self.dataPin.dir(mraa.DIR_IN)

        ack = self.dataPin.read()
        logger.debug("ack1: %s", ack)
        if ack != 0:
            logger.error("nack1")

        self.__clockTick(0)

        ack = self.dataPin.read()
        logger.debug("ack2: %s", ack)
        if ack != 1:
            logger.error("nack2")

    def __clockTick(self, value):
        self.sckPin.write(value)
        # 100 nanoseconds
        time.sleep(.0000001)

    def __waitForResult(self):
        self.dataPin.dir(mraa.DIR_IN)

        for i in range(100):
            # 10 milliseconds
            time.sleep(.01)
            ack = self.dataPin.read()
            if ack == 0:
                break
        if ack == 1:
            raise SystemError

    def __getData16Bit(self):
        # GPIO.setup(self.dataPin, GPIO.IN)
        # GPIO.setup(self.sckPin, GPIO.OUT)
        self.dataPin.dir(mraa.DIR_IN)
        self.sckPin.dir(mraa.DIR_OUT)

        # Get the most significant bits
        value = self.__shiftIn(8)
        value *= 256

        # Send the required ack
        self.dataPin.dir(mraa.DIR_OUT)
        self.dataPin.write(1)
        self.dataPin.write(0)
        self.__clockTick(1)
        self.__clockTick(0)

        # Get the least significant bits
        self.dataPin.dir(mraa.DIR_IN)
        value |= self.__shiftIn(8)

        return value

    def __shiftIn(self, bitNum):
        value = 0
        for i in range(bitNum):
            self.__clockTick(1)
            value = value * 2 + self.dataPin.read()
            self.__clockTick(0)
        return value

    def __skipCrc(self):
        # Skip acknowledge to end trans (no CRC)
        self.dataPin.dir(mraa.DIR_OUT)
        self.sckPin.dir(mraa.DIR_OUT)

        self.dataPin.write(1)
        self.__clockTick(1)
        self.__clockTick(0)

    def __connectionReset(self):
        self.dataPin.dir(mraa.DIR_OUT)
        self.sckPin.dir(mraa.DIR_OUT)
        self.dataPin.write(1)
        for i in range(10):
            self.__clockTick(1)
            self.__clockTick(0)


class WaitingSht1x(Sht1x):
    def __init__(self, dataPin, sckPin):
        super(WaitingSht1x, self).__init__(dataPin, sckPin)
        self.__lastInvocationTime = 0

    def read_temperature_C(self):
        self.__wait()        
        return super(WaitingSht1x, self).read_temperature_C()
    
    def read_humidity(self):
        temperature = self.read_temperature_C()
        self.__wait()
        return super(WaitingSht1x, self)._read_humidity(temperature)
    
    def read_temperature_and_Humidity(self):
        temperature = self.read_temperature_C()
        self.__wait()
        humidity = super(WaitingSht1x, self)._read_humidity(temperature)
        return temperature, humidity
            
    def __wait(self):
        lastInvocationDelta = time.time() - self.__lastInvocationTime
        # if we queried the sensor less then a second ago, wait until a second is passed
        if lastInvocationDelta < 1:
            time.sleep(1 - lastInvocationDelta)
        self.__lastInvocationTime = time.time()
        
def main():
    sht1x = WaitingSht1x(11, 7)
    print(sht1x.read_temperature_C())
    print(sht1x.read_humidity())
    aTouple = sht1x.read_temperature_and_Humidity()
    print("Temperature: {} Humidity: {}".format(aTouple[0], aTouple[1]))
    print(sht1x.calculate_dew_point(20, 50))
    
if __name__ == '__main__':
    main()