#include <Adafruit_GFX.h>    // Core graphics library
#include "Adafruit_EPD.h"

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

        void idle_ui();
        void wb_rec_ui();
        void wx_history_ui();
        void forecast_ui();
};