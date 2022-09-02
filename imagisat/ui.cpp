#include "ui.h"

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

void Display::status_ui(environment_state env_state, DeviceState device_state){
}

void Display::test_ui(){
  _display.clearBuffer();
  _display.setCursor(5, 5);
  _display.setTextSize(2);
  _display.setTextColor(EPD_RED);
  _display.setTextWrap(true);
  const char *text = "Welcome to ImagiSat. A meterologist in your pocket.";
  _display.print(text);
  _display.display();
}

void Display::alert_ui(Alert alert){

}

void Display::forecast_ui(Forecast forecast[12]){

}

void Display::wx_history_ui(environment_state samples[12]) {

}

void Display::wb_rec_ui(){

}

//##########################################################################

UIStateMachine::UIStateMachine() {

}

UIStateMachine::~UIStateMachine() {

}

void UIStateMachine::test_ui_state(){

}

void UIStateMachine::status_ui_state(environment_state env_state, DeviceState device_state) {

}

void UIStateMachine::wb_rec_ui_state(){

}

void UIStateMachine::wx_history_ui_state(environment_state samples[12]){

}

void UIStateMachine::forecast_ui_state(Forecast forecast[12]){

}

void UIStateMachine::alert_ui_state(Alert alert){

}

//##########################################################################


HapticDevice::HapticDevice(){

}

HapticDevice::~HapticDevice(){

}

void HapticDevice::alert(int pin){

}

void HapticDevice::notice(int pin){

}

void HapticDevice::tap(int pin){

}