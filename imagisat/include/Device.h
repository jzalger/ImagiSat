#ifndef __DEVICE_H
#define __DEVICE_H
#include <Wire.h>
#include <SPI.h>
#include "Arduino.h"
#include "types.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <Adafruit_NeoPixel.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <u-blox_config_keys.h>
#include <u-blox_structs.h>
#include <tuple>
#include "freertos/task.h"
#include "communication.h"
#include <IridiumSBD.h>
#include <si4707.h>
#include <SparkFun_AS3935.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_SharpMem.h>
#include <Fonts/FreeSans9pt7b.h>
//#include "WiFi.h"

#define PIXEL_COUNT 16
#define PIXEL_PIN 13
#define PIXEL_TYPE WS2812B
#define GPS_BAUD 9600
#define GPS_ENABLE_PIN 32
#define VOLTAGE_READ_PIN A13
#define MAX_BAT_VOLTAGE 4.2
#define WB_RST_PIN 33
#define SEALEVELPRESSURE_HPA (1013.25)
#define AS3935_ADDR 0x03 
#define AS3935_INT_PIN 27
#define AS3935_LIGHTNING_INT 0x08
#define AS3935_DISTURBER_INT 0x04
#define AS3935_NOISE_INT 0x01
#define IR_RING_PIN 21
#define DISPLAY_SS A5
#define DISPLAY_SCK 5 
#define DISPLAY_MOSI 18
#define BLACK 0
#define WHITE 1

enum Indicator_State_t {
    IDLE,
    GPS_SEARCHING,
    GPS_LOCK,
    IRIDIUM_SENDING,
    IRIDIUM_SENT,
    IRIDIUM_RECEIVED,
    WX_ALERT,
    TEST,
    ERROR
};

void gps_data_callback(UBX_NAV_PVT_data_t ubxDataStruct);
void IRAM_ATTR lightning_detect_callback();

class Logger;

class Display {
    public:
        Display();
        virtual ~Display();
        uint16_t setup();
        void refresh();
        void test_ui();
        void status_ui(environment_state env_state, DeviceState device_state);
        void wb_rec_ui();
        void gps_searching_ui();
        void wx_history_ui(environment_state samples[12]);
        void forecast_ui(Forecast forecast[12]);
        void alert_ui(Alert alert);
};

class HapticDevice {
    public:
        HapticDevice();
        virtual ~HapticDevice();

        void tap(int pin);
        void notice(int pin);
        void alert(int pin);
};

class Indicator {
    public:
        Indicator();
        virtual ~Indicator();

        void pulse(uint16_t red, uint16_t green, uint16_t blue, uint16_t brightness);
        void cycle(uint16_t red, uint16_t green, uint16_t blue, uint16_t brightness);
};

class UIStateMachine {
    public:
        UIStateMachine();
        ~UIStateMachine();

        Display display;
        HapticDevice haptic;
        Indicator indicator;

        void setup();
        void refresh();
        void gps_searching_state();
        void test_ui_state();
        void status_ui_state(environment_state env_state, DeviceState device_state);
        void wb_rec_ui_state();
        void wx_history_ui_state(environment_state samples[12]);
        void forecast_ui_state(Forecast forecast[12]);
        void alert_ui_state(Alert alert);
        void update_indicator_state(Indicator_State_t state);
};

class Device {
    public:
        Device();
        ~Device();
        UIStateMachine ui;

        bool gps_enabled = true;
        bool ble_enabled = false;
        bool iridium_enabled = true;
        bool iridium_diagnostics = false;
        int iridium_signal_quality = -1;
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
        std::tuple <float, float, float, int, bool> get_gps_location();
        float get_latitude();
        float get_longitude();
        float get_altitude();
        uint8_t get_gps_fix_type();
        
        byte init_si4707();

        void bme680_setup();

        void iridium_setup();

        std::tuple <float, int> get_battery_health();
        float get_voltage();
        int get_charge_state();
        uint32_t write_datalog_entry(String filename, String data);
        uint64_t get_datalog_size(String filename);
        std::tuple <float, float, float, float, float> get_wx_reading();
        void transmit_state(environment_state state);

        // Status Ring Functions
        void cycle_status_ring(uint16_t red, uint16_t green, uint16_t blue, uint16_t brightness);
        void pulse_status_ring(uint16_t red, uint16_t green, uint16_t blue, uint16_t brightness);

    private:
        uint16_t _test_gps();
        uint16_t _test_bme();
        uint16_t _test_iridium();
        uint16_t _test_device_health();
        
};

void log_info(String str);
void log_debug(String str);
void log_warning(String str);
void log_error(String str);
void _write_log(String level, String str);

#endif
