/*
AS3935.cpp - AS3935 Franklin Lightning Sensor™ IC by AMS library
Copyright (c) 2012 Raivis Rengelis (raivis [at] rrkb.lv). All rights reserved.
Porté sur MBED par Valentin, version I2C

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

#include "AS3935.hpp"

AS3935::AS3935(PinName sda, PinName scl, int adresse): i2c(sda, scl), _adress(adresse) {        
        
        wait_ms(11);
    }

    //~AS3935(){
        
    //}
        
char AS3935::_rawRegisterRead(char reg)
{
    char data;
    i2c.write( _adress, &reg, 1, true);
    i2c.read(_adress, &data, 1);
    return data;
    
}

char AS3935::_ffsz(char mask)
{
    char i = 0;
    //char temp;
    
    while(!(mask & 1)) {
       //if (!(mask & 1)) {
          mask >>= 1;
          i++;
       //}
    }
    //if (mask){
      //  for (i = 1; ~mask & 1 ; i++) {
        //    mask >>= 1;
          //  }
       // }
    return i;
}

void AS3935::registerWrite(char reg, char mask, char data)
{
    char cmd[2];
    char regval;
    regval = _rawRegisterRead(reg);
    regval &= ~(mask);
    //if (mask){
        regval |= (data << (_ffsz(mask)));
      //  }
    //else {
      //  regval |= data;
       // }
    cmd[0] = reg;
    cmd[1] = regval;
    i2c.write( _adress, cmd, 2);

}

char AS3935::registerRead(char reg, char mask)
    {
    char regval;
    regval = _rawRegisterRead(reg);
    regval = regval & mask;
    //if (mask){
        regval >>= (_ffsz(mask));
      //  }
    return regval;
}

void AS3935::reset()
    {
    char cmd[2];
    cmd[0] = 0x3C;
    cmd[1] = 0x96;
    i2c.write( _adress, cmd, 2);
    wait_ms(2);
    }



void AS3935::powerDown()
    {
    registerWrite(AS3935_PWD,1);
    }

void AS3935::powerUp()
    {
    char cmd[2];
    cmd[0] = 0x3D;
    cmd[1] = 0x96;
    registerWrite(AS3935_PWD,0);
    i2c.write( _adress, cmd, 2);
    wait_ms(3);
    registerWrite(AS3935_DISP_TRCO,1);
    wait_ms(2);
    registerWrite(AS3935_DISP_TRCO,0);
    }

int AS3935::interruptSource()
    {
    return registerRead(AS3935_INT);
    }

void AS3935::disableDisturbers()
    {
    registerWrite(AS3935_MASK_DIST,1);
    }

void AS3935::enableDisturbers()
    {
    registerWrite(AS3935_MASK_DIST,0);
    }

int AS3935::getMinimumLightnings()
    {
    return registerRead(AS3935_MIN_NUM_LIGH);
    }

int AS3935::setMinimumLightnings(int minlightning)
    {
    registerWrite(AS3935_MIN_NUM_LIGH,minlightning);
    return getMinimumLightnings();
    }

int AS3935::lightningDistanceKm()
    {
    return registerRead(AS3935_DISTANCE);
    }

void AS3935::setIndoors()
    {
    registerWrite(AS3935_AFE_GB,AS3935_AFE_INDOOR);
    }

void AS3935::setOutdoors()
    {
    registerWrite(AS3935_AFE_GB,AS3935_AFE_OUTDOOR);
    }

int AS3935::getNoiseFloor()
    {
    return registerRead(AS3935_NF_LEV);
    }

int AS3935::setNoiseFloor(int noisefloor)
    {
    registerWrite(AS3935_NF_LEV,noisefloor);
    return getNoiseFloor();
    }

int AS3935::getSpikeRejection()
    {
    return registerRead(AS3935_SREJ);
    }

int AS3935::setSpikeRejection(int srej)
    {
    registerWrite(AS3935_SREJ, srej);
    return getSpikeRejection();
    }

int AS3935::getWatchdogThreshold()
    {
    return registerRead(AS3935_WDTH);
    }

int AS3935::setWatchdogThreshold(int wdth)
    {
    registerWrite(AS3935_WDTH,wdth);
    return getWatchdogThreshold();
    }

int AS3935::getTuneCap()
    {
    return registerRead(AS3935_TUN_CAP);    
    }
        
int AS3935::setTuneCap(int cap)
    {
    registerWrite(AS3935_TUN_CAP,cap);
    return getTuneCap();    
    }

void AS3935::clearStats()
    {
    registerWrite(AS3935_CL_STAT,1);
    registerWrite(AS3935_CL_STAT,0);
    registerWrite(AS3935_CL_STAT,1);
    }