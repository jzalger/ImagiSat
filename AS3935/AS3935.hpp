/*
AS3935.h - AS3935 Franklin Lightning Sensor™ IC by AMS library
Copyright (c) 2012 Raivis Rengelis (raivis [at] rrkb.lv). All rights reserved.
Portée sur MBED par Valentin

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#ifndef _AS3935_HPP
#define _AS3935_HPP

#include "mbed.h"
//#include "i2c.hpp"

// register access macros - register address, bitmask
#define AS3935_AFE_GB 0x00, 0x3E
#define AS3935_PWD 0x00, 0x01
#define AS3935_NF_LEV 0x01, 0x70
#define AS3935_WDTH 0x01, 0x0F
#define AS3935_CL_STAT 0x02, 0x40
#define AS3935_MIN_NUM_LIGH 0x02, 0x30
#define AS3935_SREJ 0x02, 0x0F
#define AS3935_LCO_FDIV 0x03, 0xC0
#define AS3935_MASK_DIST 0x03, 0x20
#define AS3935_INT 0x03, 0x0F
#define AS3935_DISTANCE 0x07, 0x3F
#define AS3935_DISP_LCO 0x08, 0x80
#define AS3935_DISP_SRCO 0x08, 0x40
#define AS3935_DISP_TRCO 0x08, 0x20
#define AS3935_TUN_CAP 0x08, 0x0F

// other constants
#define AS3935_AFE_INDOOR 0x12
#define AS3935_AFE_OUTDOOR 0x0E

class AS3935 {
    
    public:
        /*
         * Initializes I2C interface and IRQ pin
         */
        AS3935(PinName sda, PinName scl, int adresse);
        
        //destruction         
        //~AS3935();   
        
        //write to specified register specified data using specified bitmask,     
        //the rest of the register remains intact
        void registerWrite(char reg, char mask, char data);
        
        //read specified register using specified bitmask and return value aligned     
        //to lsb, i.e. if value to be read is in a middle of register, function     
        //reads register and then aligns lsb of value to lsb of byte
        char registerRead(char reg, char mask);
        
        //reset all the registers on chip to default values
        void reset();
        
        //put chip into power down mode
        void powerDown();
        
        //bring chip out of power down mode and perform RCO calibration
        void powerUp();
        
        //return interrupt source, bitmask, 0b0001 - noise, 0b0100 - disturber,     
        //0b1000 - lightning
        int interruptSource();
        
        //disable indication of disturbers
        void disableDisturbers();
        
        //enable indication of distrubers
        void enableDisturbers();
        
        //return number of lightnings that need to be detected in 17 minute period     
        //before interrupt is issued
        int getMinimumLightnings();
        
        //set number of lightnings that need to be detected in 17 minute period     
        //before interrupt is issued
        int setMinimumLightnings(int minlightning);
        
        //return distance to lightning in kilometers, 1 means storm is overhead,     
        //63 means it is too far to reliably calculate distance
        int lightningDistanceKm();
        
        // load gain preset to operate indoors
        void setIndoors();
        
        //load gain preset to operate outdoors
        void setOutdoors();
        
        //return noise floor setting - refer to datasheet for meaning and range
        int getNoiseFloor();
        
        //set noise floor setting
        int setNoiseFloor(int noisefloor);
        
        //return spike rejection value - refer to datasheet for meaning and range
        int getSpikeRejection();
        
        //set spike rejection value
        int setSpikeRejection(int srej);
        
        //return watchdog threshold value - refer to datasheet for meaning and range
        int getWatchdogThreshold();
        
        //set watchdog threshold value
        int setWatchdogThreshold(int wdth);
        
        //return tune Capacity value 
        int getTuneCap();
        
        //set tune Capacity value
        int setTuneCap(int cap);
        
        //clear internal accumulated lightning statistics
        void clearStats();
    
    private:
        I2C i2c;   
        //DigitalOut _irq;
        int _adress;
        char _rawRegisterRead(char reg);
        char _ffsz(char mask);
};

/* !_AS3935_HPP_ */
#endif