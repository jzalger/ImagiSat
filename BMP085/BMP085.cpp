/*
 * mbed library to use a Bosch Sensortec BMP085/BMP180 sensor
 * Copyright (c) 2010 Hiroshi Suga
 * Released under the MIT License: http://mbed.org/license/mit
 */

/** @file BMP085.cpp
 * @brief mbed library to use a Bosch Sensortec BMP085/BMP180 sensor
 * barometric pressure sensor BMP085/BMP180 (Bosch Sensortec)
 * interface: I2C digital
 */

#include "mbed.h"
#include "BMP085.h"

#define WEATHER_BMP085 0xee
#define xpow(x, y) ((long)1 << y)

/**
 * @brief Initializes interface (private I2C)
 * @param p_sda port of I2C SDA
 * @param p_scl port of I2C SCL
 * @param p_oss parameter of OSS
 */
BMP085::BMP085 (PinName p_sda, PinName p_scl, BMP085_oss p_oss) : i2c(p_sda, p_scl) {
    init(p_oss);
}

/**
 * @brief Initializes interface (public I2C)
 * @param p_i2c instance of I2C class
 * @param p_oss parameter of OSS
 */
BMP085::BMP085 (I2C& p_i2c, BMP085_oss p_oss) : i2c(p_i2c) { 
    init(p_oss);
}

/**
 * @brief Get temperature
 * @return temperature (`C)
 */
float BMP085::get_temperature() {
    return temperature;
}

/**
 * @brief Get pressure
 * @return pressure (hPa)
 */
float BMP085::get_pressure() {
    return pressure;
}

/**
 * @brief Update results
 */
void BMP085::update () {
    long t, p, ut, up, x1, x2, x3, b3, b5, b6;
    unsigned long b4, b7;

    twi_writechar(WEATHER_BMP085, 0xf4, 0x2e);
    wait(0.01);
    ut = twi_readshort(WEATHER_BMP085, 0xf6);

    twi_writechar(WEATHER_BMP085, 0xf4, 0x34 | (oss << 6));
    wait(0.05);
    up = twi_readlong(WEATHER_BMP085, 0xf6) >> (8 - oss);

    x1 = (ut - ac6) * ac5 / xpow(2, 15);
    x2 = (long)mc * xpow(2, 11) / (x1 + md);
    b5 = x1 + x2;
    t = (b5 + 8) / xpow(2, 4);
    temperature = (float)t / 10.0;

    b6 = b5 - 4000;
    x1 = (b2 * (b6 * b6 / xpow(2, 12))) / xpow(2, 11);
    x2 = ac2 * b6 / xpow(2, 11);
    x3 = x1 + x2;
    b3 = ((((unsigned long)ac1 * 4 + x3) << oss) + 2) / 4;
    x1 = ac3 * b6 / xpow(2, 13);
    x2 = (b1 * (b6 * b6 / xpow(2, 12))) / xpow(2, 16);
    x3 = ((x1 + x2) + 2) / xpow(2, 2);
    b4 = ac4 * (unsigned long)(x3 + 32768) / xpow(2, 15);
    b7 = ((unsigned long)up - b3) * (50000 >> oss);
    if (b7 < (unsigned long)0x80000000) {
        p = (b7 * 2) / b4;
    } else {
        p = (b7 / b4) * 2;
    }
    x1 = (p / xpow(2, 8)) * (p / xpow(2, 8));
    x1 = (x1 * 3038) / xpow(2, 16);
    x2 = (-7357 * p) / xpow(2, 16);
    p = p + (x1 + x2 + 3791) / xpow(2, 4);
    pressure = (float)p / 100.0;
}

void BMP085::init (BMP085_oss p_oss) {
    ac1 = twi_readshort(WEATHER_BMP085, 0xaa);
    ac2 = twi_readshort(WEATHER_BMP085, 0xac);
    ac3 = twi_readshort(WEATHER_BMP085, 0xae);
    ac4 = twi_readshort(WEATHER_BMP085, 0xb0);
    ac5 = twi_readshort(WEATHER_BMP085, 0xb2);
    ac6 = twi_readshort(WEATHER_BMP085, 0xb4);
    b1 = twi_readshort(WEATHER_BMP085, 0xb6);
    b2 = twi_readshort(WEATHER_BMP085, 0xb8);
    mb = twi_readshort(WEATHER_BMP085, 0xba);
    mc = twi_readshort(WEATHER_BMP085, 0xbc);
    md = twi_readshort(WEATHER_BMP085, 0xbe);
    oss = p_oss;
}

unsigned short BMP085::twi_readshort (int id, int addr) {
    unsigned short i;

    i2c.start();
    i2c.write(id);
    i2c.write(addr);

    i2c.start();
    i2c.write(id | 1);
    i = i2c.read(1) << 8;
    i |= i2c.read(0);
    i2c.stop();

    return i;
}

unsigned long BMP085::twi_readlong (int id, int addr) {
    unsigned long i;

    i2c.start();
    i2c.write(id);
    i2c.write(addr);

    i2c.start();
    i2c.write(id | 1);
    i = i2c.read(1) << 16;
    i |= i2c.read(1) << 8;
    i |= i2c.read(0);
    i2c.stop();

    return i;
}

void BMP085::twi_writechar (int id, int addr, int dat) {

    i2c.start();
    i2c.write(id);
    i2c.write(addr);
    i2c.write(dat);
    i2c.stop();
}
