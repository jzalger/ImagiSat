# 1 "/tmp/tmpxie5cl9z"
#include <Arduino.h>
# 1 "/home/jzalger/Projects/ImagiSat/src/imagisat.ino"
#include "MainStateMachine.h"
#include "Device.h"
#include "types.h"





const uint8_t current_firmware_version = 0;

MainStateMachine mainStateMachine;
void setup();
void loop();
#line 18 "/home/jzalger/Projects/ImagiSat/src/imagisat.ino"
void setup()
{

 pinMode(DFU_PIN, INPUT);
 if(digitalRead(DFU_PIN)==HIGH) {

  log_debug("Entering DFU Mode");
 }
 mainStateMachine.setup();
}

void loop()
{
 mainStateMachine.loop();
}