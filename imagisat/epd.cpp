#include "epd.h"

Adafruit_IL0373 _display(296, 128, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);

Display::Display() {   
}

Display::~Display() {
}

uint16_t Display::setup(){
    _display.begin();
    _display.clearBuffer();
    _display.fillScreen(EPD_WHITE);
    return 0;
}

void Display::idle_ui(){
  _display.clearBuffer();
  _display.setCursor(5, 5);
  _display.setTextSize(2);
  _display.setTextColor(EPD_RED);
  _display.setTextWrap(true);
  const char *text = "Welcome to ImagiSat. A meterologist in your pocket.";
  _display.print(text);
  _display.display();
}

void Display::forecast_ui(){

}
void Display::wx_history_ui() {

}

void Display::wb_rec_ui(){

}