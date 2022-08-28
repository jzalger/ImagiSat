#ifndef __DEVICE_H
#define __DEVICE_H
#include <neopixel.h>
#include <tuple>
#include "mbed.h"
#include "SerialGPS.h"
#include "Arduino.h"
#include "BLEPeripheral.h"
#include "stdlib.h"
#include <string>

#define SCL PB_8 
#define SDA PB_9
#define UART6_TX PA_11  //ROCKBLOCK
#define UART6_RX PA_12

//Power Management
#define BATT_ALERT_PIN PA_5
#define ALL_SLEEP_PIN PA_6
float battPercent = 0;
float battVoltage = 0;

//GPS on Serial1
#define GPS_BAUD 9600
#define GPS_PPS PA_7
#define GPS_TX PA_9
#define GPS_RX PA_10
#define GPS_PWR_PIN PB_6

bool gpsLock = false;
bool gpsConnect = false;
DigitalOut GPS_PWR(GPS_PWR_PIN);
SerialGPS gps(GPS_TX,GPS_RX,GPS_BAUD);

//Device State Variables
bool gpsOn = false;
bool bluetoothOn = true;
bool wxLog = false;
bool rockBlockOn = false;

//Rockblock Config - Rockblock on Serial2
#define ROCKBLOCK_BAUD 19200

//USB Serial
Serial serial(USBTX, USBRX);


class Device {
    public:
        Device();
        ~Device();

        bool debug_mode = false;
        bool gps_enabled = false;

        void device_setup();
        void enable_debug_mode();
        void update_gps_location();
        void enable_gps();
        void disable_gps();

        std::tuple <float, float, float> get_gps_location();
        float get_voltage();
        int get_charge_state();
        uint32_t write_datalog_entry(String filename, String data);
        uint32_t get_datalog_size(String filename);

        // Status Ring Functions
        void cycle_status_ring(uint16_t red, uint16_t green, uint16_t blue, uint16_t brightness);
        void pulse_status_ring_yellow(uint16_t max_brightness);
        void pulse_status_ring_blue(uint16_t max_brightness);
}; 
#endif

