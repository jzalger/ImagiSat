/*
 * mbed library to use a Bosch Sensortec BMP085/BMP180 sensor
 * Copyright (c) 2010 Hiroshi Suga
 * Released under the MIT License: http://mbed.org/license/mit
 */
 
/** @file BMP085.h
 * @brief mbed library to use a Bosch Sensortec BMP085/BMP180 sensor
 * barometric pressure sensor BMP085/BMP180 (Bosch Sensortec)
 * interface: I2C digital
 */
 
#ifndef BMP085_H
#define BMP085_H

#include "mbed.h"

/**
 * @brief over sampling setting
 */
enum BMP085_oss {
    BMP085_oss1 = 0, ///< ultra low power (1 time)
    BMP085_oss2 = 1, ///< standard (2 times)
    BMP085_oss4 = 2, ///< high resolution (4 times)
    BMP085_oss8 = 3  ///< ultra high resolution (8 times)
};

/**
 * @brief BMP085 class
 */
class BMP085 {
public:
    BMP085(PinName p_sda, PinName p_scl, BMP085_oss p_oss = BMP085_oss1);
    BMP085(I2C& p_i2c, BMP085_oss p_oss = BMP085_oss1);

    float get_temperature();
    float get_pressure();
    void update();

protected:
    void init(BMP085_oss);
    unsigned short twi_readshort (int, int);
    unsigned long twi_readlong (int, int);
    void twi_writechar (int, int, int);

    I2C i2c;
    float temperature;
    float pressure;

private:

    short ac1, ac2, ac3, b1, b2, mb, mc, md, oss;
    unsigned short ac4, ac5, ac6;
};

#endif