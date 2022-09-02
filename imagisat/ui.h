#include <Adafruit_GFX.h>    // Core graphics library
#include "Adafruit_EPD.h"
#include "types.h"

#define EPD_CS     A5
#define EPD_DC      A1
#define SRAM_CS     -1
#define EPD_RESET   A0 
#define EPD_BUSY    32 

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

class UIStateMachine {
    public:
        UIStateMachine();
        ~UIStateMachine();

        void test_ui_state();
        void status_ui_state(environment_state env_state, DeviceState device_state);
        void wb_rec_ui_state();
        void wx_history_ui_state(environment_state samples[12]);
        void forecast_ui_state(Forecast forecast[12]);
        void alert_ui_state(Alert alert);
};

class HapticDevice {
    public:
        HapticDevice();
        virtual ~HapticDevice();

        void tap(int pin);
        void notice(int pin);
        void alert(int pin);
};

class Display {
    public:
        Display();
        virtual ~Display();
        uint16_t setup();

        void test_ui();
        void status_ui(environment_state env_state, DeviceState device_state);
        void wb_rec_ui();
        void wx_history_ui(environment_state samples[12]);
        void forecast_ui(Forecast forecast[12]);
        void alert_ui(Alert alert);
};
