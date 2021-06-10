/* MAX17048 Driver Library
 * Copyright (c) 2013 Neil Thiessen
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "MAX17048.h"

const char MAX17048::RCOMP0 = 0x97;
const int MAX17048::m_ADDR = (0x36 << 1);

MAX17048::MAX17048(PinName sda, PinName scl, int hz) : m_I2C(sda, scl)
{
    //Set the I2C bus frequency
    m_I2C.frequency(hz);
}

bool MAX17048::open()
{
    //Probe for the MAX17048 using a Zero Length Transfer
    if (!m_I2C.write(m_ADDR, NULL, 0)) {
        //Return success
        return true;
    } else {
        //Return failure
        return false;
    }
}

void MAX17048::reset()
{
    //Write the POR command
    write(REG_CMD, 0x5400);
}

void MAX17048::quickStart()
{
    //Read the current 16-bit register value
    unsigned short value = read(REG_MODE);

    //Set the QuickStart bit
    value |= (1 << 14);

    //Write the value back out
    write(REG_MODE, value);
}

bool MAX17048::sleepEnabled()
{
    //Read the 16-bit register value
    unsigned short value = read(REG_MODE);

    //Return the status of the EnSleep bit
    if (value & (1 << 13))
        return true;
    else
        return false;
}

void MAX17048::sleepEnabled(bool enabled)
{
    //Read the current 16-bit register value
    unsigned short value = read(REG_MODE);

    //Set or clear the EnSleep bit
    if (enabled)
        value |= (1 << 13);
    else
        value &= ~(1 << 13);

    //Write the value back out
    write(REG_MODE, value);
}

bool MAX17048::hibernating()
{
    //Read the 16-bit register value
    unsigned short value = read(REG_MODE);

    //Return the status of the HibStat bit
    if (value & (1 << 12))
        return true;
    else
        return false;
}

float MAX17048::hibernateThreshold()
{
    //Read the 16-bit register value
    unsigned short value = read(REG_HIBRT);

    //Extract the hibernate threshold
    return (value >> 8) * 0.208;
}

void MAX17048::hibernateThreshold(float threshold)
{
    //Read the current 16-bit register value
    unsigned short value = read(REG_HIBRT);

    //Mask off the old value
    value &= 0x00FF;

    //Do a smart update
    if (threshold > 0.0) {
        if (threshold < 53.04)
            value |= (unsigned short)(threshold / 0.208) << 8;
        else
            value |= 0xFF00;
    }

    //Write the 16-bit register
    write(REG_HIBRT, value);
}

float MAX17048::activeThreshold()
{
    //Read the 16-bit register value
    unsigned short value = read(REG_HIBRT);

    //Extract the active threshold
    return (value & 0x00FF) * 0.00125;
}

void MAX17048::activeThreshold(float threshold)
{
    //Read the current 16-bit register value
    unsigned short value = read(REG_HIBRT);

    //Mask off the old value
    value &= 0xFF00;

    //Do a smart update
    if (threshold > 0.0) {
        if (threshold < 0.31875)
            value |= (char)(threshold / 0.00125);
        else
            value |= 0x00FF;
    }

    //Write the 16-bit register
    write(REG_HIBRT, value);
}

unsigned short MAX17048::version()
{
    //Return the 16-bit production version
    return read(REG_VERSION);
}

char MAX17048::compensation()
{
    //Read the 16-bit register value
    unsigned short value = read(REG_CONFIG);

    //Return only the upper byte
    return (char)(value >> 8);
}

void MAX17048::compensation(char rcomp)
{
    //Read the current 16-bit register value
    unsigned short value = read(REG_CONFIG);

    //Update the register value
    value &= 0x00FF;
    value |= rcomp << 8;

    //Write the value back out
    write(REG_CONFIG, value);
}

void MAX17048::tempCompensation(float temp)
{
    //Calculate the new RCOMP value
    char rcomp;
    if (temp > 20.0) {
        rcomp = RCOMP0 + (temp - 20.0) * -0.5;
    } else {
        rcomp = RCOMP0 + (temp - 20.0) * -5.0;
    }

    //Update the RCOMP value
    compensation(rcomp);
}

bool MAX17048::sleeping()
{
    //Read the 16-bit register value
    unsigned short value = read(REG_CONFIG);

    //Return the status of the SLEEP bit
    if (value & (1 << 7))
        return true;
    else
        return false;
}

void MAX17048::sleep(bool sleep)
{
    //Read the current 16-bit register value
    unsigned short value = read(REG_CONFIG);

    //Set or clear the SLEEP bit
    if (sleep)
        value |= (1 << 7);
    else
        value &= ~(1 << 7);

    //Write the value back out
    write(REG_CONFIG, value);
}

bool MAX17048::socChangeAlertEnabled()
{
    //Read the 16-bit register value
    unsigned short value = read(REG_CONFIG);

    //Return the status of the ALSC bit
    if (value & (1 << 6))
        return true;
    else
        return false;
}

void MAX17048::socChangeAlertEnabled(bool enabled)
{
    //Read the current 16-bit register value
    unsigned short value = read(REG_CONFIG);

    //Set or clear the ALSC bit
    if (enabled)
        value |= (1 << 6);
    else
        value &= ~(1 << 6);

    //Write the value back out
    write(REG_CONFIG, value);
}

bool MAX17048::alerting()
{
    //Read the 16-bit register value
    unsigned short value = read(REG_CONFIG);

    //Return the status of the ALRT bit
    if (value & (1 << 5))
        return true;
    else
        return false;
}

void MAX17048::clearAlert()
{
    //Read the current 16-bit register value
    unsigned short value = read(REG_CONFIG);

    //Clear the ALRT bit
    value &= ~(1 << 5);

    //Write the value back out
    write(REG_CONFIG, value);
}

char MAX17048::emptyAlertThreshold()
{
    //Read the 16-bit register value
    unsigned short value = read(REG_CONFIG);

    //Extract the threshold
    return 32 - (value & 0x001F);
}

void MAX17048::emptyAlertThreshold(char threshold)
{
    //Read the current 16-bit register value
    unsigned short value = read(REG_CONFIG);

    //Range check threshold
    if (threshold < 1)
        threshold = 1;
    else if (threshold > 32)
        threshold = 32;

    //Update the register value
    value &= 0xFFE0;
    value |= 32 - threshold;

    //Write the 16-bit register
    write(REG_CONFIG, value);
}

float MAX17048::vAlertMinThreshold()
{
    //Read the 16-bit register value
    unsigned short value = read(REG_VALRT);

    //Extract the alert threshold
    return (value >> 8) * 0.02;
}

void MAX17048::vAlertMinThreshold(float threshold)
{
    //Read the current 16-bit register value
    unsigned short value = read(REG_VALRT);

    //Mask off the old value
    value &= 0x00FF;

    //Do a smart update
    if (threshold > 0.0) {
        if (threshold < 5.1)
            value |= (unsigned short)(threshold / 0.02) << 8;
        else
            value |= 0xFF00;
    }

    //Write the 16-bit register
    write(REG_VALRT, value);
}

float MAX17048::vAlertMaxThreshold()
{
    //Read the 16-bit register value
    unsigned short value = read(REG_VALRT);

    //Extract the active threshold
    return (value & 0x00FF) * 0.02;
}

void MAX17048::vAlertMaxThreshold(float threshold)
{
    //Read the current 16-bit register value
    unsigned short value = read(REG_VALRT);

    //Mask off the old value
    value &= 0xFF00;

    //Do a smart update
    if (threshold > 0.0) {
        if (threshold < 5.1)
            value |= (char)(threshold / 0.02);
        else
            value |= 0x00FF;
    }

    //Write the 16-bit register
    write(REG_VALRT, value);
}

float MAX17048::vResetThreshold()
{
    //Read the 16-bit register value
    unsigned short value = read(REG_VRESET_ID);

    //Extract the threshold
    return (value >> 9) * 0.04;
}

void MAX17048::vResetThreshold(float threshold)
{
    //Read the current 16-bit register value
    unsigned short value = read(REG_VRESET_ID);

    //Mask off the old value
    value &= 0x01FF;

    //Do a smart update
    if (threshold > 0.0) {
        if (threshold < 5.08)
            value |= (unsigned short)(threshold / 0.04) << 9;
        else
            value |= 0xFE00;
    }

    //Write the 16-bit register
    write(REG_VRESET_ID, value);
}

bool MAX17048::comparatorEnabled()
{
    //Read the 16-bit register value
    unsigned short value = read(REG_VRESET_ID);

    //Return the status of the Dis bit
    if (value & (1 << 8))
        return false;
    else
        return true;
}

void MAX17048::comparatorEnabled(bool enabled)
{
    //Read the current 16-bit register value
    unsigned short value = read(REG_VRESET_ID);

    //Set or clear the Dis bit
    if (enabled)
        value &= ~(1 << 8);
    else
        value |= (1 << 8);

    //Write the value back out
    write(REG_VRESET_ID, value);
}

char MAX17048::id()
{
    //Read the 16-bit register value
    unsigned short value = read(REG_VRESET_ID);

    //Return only the ID bits
    return value;
}

bool MAX17048::vResetAlertEnabled()
{
    //Read the 16-bit register value
    unsigned short value = read(REG_STATUS);

    //Return the status of the EnVR bit
    if (value & (1 << 14))
        return true;
    else
        return false;
}

void MAX17048::vResetAlertEnabled(bool enabled)
{
    //Read the current 16-bit register value
    unsigned short value = read(REG_STATUS);

    //Set or clear the EnVR bit
    if (enabled)
        value |= (1 << 14);
    else
        value &= ~(1 << 14);

    //Write the value back out
    write(REG_STATUS, value);
}

char MAX17048::alertFlags()
{
    //Read the 16-bit register value
    unsigned short value = read(REG_STATUS);

    //Return only the flag bits
    return (value >> 8) & 0x3F;
}

void MAX17048::clearAlertFlags(char flags)
{
    //Read the current 16-bit register value
    unsigned short value = read(REG_STATUS);

    //Clear the specified flag bits
    value &= ~((flags & 0x3F) << 8);

    //Write the value back out
    write(REG_STATUS, value);
}

float MAX17048::vcell()
{
    //Read the 16-bit raw Vcell value
    unsigned short value = read(REG_VCELL);

    //Return Vcell in volts
    return value * 0.000078125;
}

float MAX17048::soc()
{
    //Read the 16-bit raw SOC value
    unsigned short value = read(REG_SOC);

    //Return SOC in percent
    return value * 0.00390625;
}

int MAX17048::soc_int()
{
    //Read the 16-bit raw SOC value
    unsigned short value = read(REG_SOC);

    //Return only the top byte
    return value >> 8;
}

float MAX17048::crate()
{
    //Read the 16-bit raw C/Rate value
    short value = read(REG_CRATE);

    //Return C/Rate in %/hr
    return value * 0.208;
}

#ifdef MBED_OPERATORS
MAX17048::operator float()
{
    //Return the current floating point SOC reading
    return soc();
}

MAX17048::operator int()
{
    //Return the current integer SOC reading
    return soc_int();
}
#endif

unsigned short MAX17048::read(char reg)
{
    //Create a temporary buffer
    char buff[2];

    //Select the register
    m_I2C.write(m_ADDR, &reg, 1, true);

    //Read the 16-bit register
    m_I2C.read(m_ADDR, buff, 2);

    //Return the combined 16-bit value
    return (buff[0] << 8) | buff[1];
}

void MAX17048::write(char reg, unsigned short data)
{
    //Create a temporary buffer
    char buff[3];

    //Load the register address and 16-bit data
    buff[0] = reg;
    buff[1] = data >> 8;
    buff[2] = data;

    //Write the data
    m_I2C.write(m_ADDR, buff, 3);
}
