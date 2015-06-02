ImagiSat
========
ImagiSat is a meteorologist in your pocket. It delivers supercomputed weather forecasts for anywhere to anywhere.

Hardware
========
The core of the ImagiSat is the Intel Edison Microcontroller. This is a powerful yet energy efficient X86 based
chip with on-board WiFI and Bluetooth LE running a Linux kernel. It has a dual core 500MHz Atom processor, 4GB flash 
storage, and 1GB ram.

Software
========
All core logic is written in Python. 

I2C Support
-----------

For i2C support using the Adafruit based I2C Python libraries, the following steps were required:
1. Install i2c-tools and python-smbus on Edison Linux.
2. Adafruit GPIO.I2C library was modified to return 1 as the bus when querying the device. Also require_repeated_start() 
was bypassed as this was implemented for the RPI and does not seem necessary for the Edison.

3. The modified Adafruit GPIO.I2C was installed, followed by the necessary I2C device modules.