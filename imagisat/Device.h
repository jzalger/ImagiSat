#ifndef __DEVICE_H
#define __DEVICE_H
#include <Wire.h>
#include <SPI.h>
#include "Arduino.h"
#include "types.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <u-blox_config_keys.h>
#include <u-blox_structs.h>
#include <tuple>
#include "freertos/task.h"
#include "communication.h"
#include <IridiumSBD.h>
#include "si4707.h"
//#include "WiFi.h"

#define PIXEL_COUNT 16
#define PIXEL_PIN 13
#define PIXEL_TYPE WS2812B
#define GPS_BAUD 9600
#define GPS_ENABLE_PIN 32
#define DEBUG_SERIAL_TX 33
#define DEBUG_SERIAL_RX 15
#define VOLTAGE_READ_PIN A13
#define MAX_BAT_VOLTAGE 4.2
#define WB_RST_PIN 33

void gps_data_callback(UBX_NAV_PVT_data_t ubxDataStruct);

class Logger;

class Device {
    public:
        Device();
        ~Device();
        bool debug_mode = true;
        bool gps_enabled = true;

        bool ble_enabled = false;
        void enable_ble();
        void disable_ble();
        void ble_setup();
        void ble_loop();
        void ble_transmit(uint8_t *ble_tx_buffer, int ble_tx_buff_size);

        uint16_t device_setup();
        uint16_t test();
        void enable_debug_mode();
        void update_gps_location();
        void enable_gps();
        void disable_gps();
        
        byte init_si4707();

        std::tuple <float, float, float, int, bool> get_gps_location();
        float get_latitude();
        float get_longitude();
        float get_altitude();
        uint8_t get_gps_fix_type();

        std::tuple <float, int> get_battery_health();
        float get_voltage();
        int get_charge_state();
        uint32_t write_datalog_entry(String filename, String data);
        uint64_t get_datalog_size(String filename);
        float get_temperature();
        void transmit_state(environment_state state);

        // Status Ring Functions
        void cycle_status_ring(uint16_t red, uint16_t green, uint16_t blue, uint16_t brightness);
        void pulse_status_ring(uint16_t red, uint16_t green, uint16_t blue, uint16_t brightness);

        void log_info(String str);
        void log_debug(String str);
        void log_warning(String str);
        void log_error(String str);

    private:
        uint16_t _test_gps();
        uint16_t _test_led_ui();
        void _write_log(String level, String str);
};      
#endif
