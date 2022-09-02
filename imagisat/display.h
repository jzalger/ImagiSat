#include <Adafruit_GFX.h>    // Core graphics library
#include "Adafruit_EPD.h"
#include "types.h"

#define EPD_CS     A5
#define EPD_DC      A1
#define SRAM_CS     -1
#define EPD_RESET   A0 
#define EPD_BUSY    32 

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
