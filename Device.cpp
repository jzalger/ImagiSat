#include "Device.h"


Device::Device() {
}

Device::~Device() { 

}

void Device::device_setup() {
    pinMode(GPS_ENABLE_PIN, OUTPUT);
    enable_gps(); // Turn on GPS by default
    Log.info("Initializing GPS");
}

// #####################################################################
// DEVICE FUNCTIONS
// #####################################################################
void Device::cycle_status_ring(uint16_t red, uint16_t green, uint16_t blue, uint16_t brightness) {
    ring.setBrightness(brightness);
    for(int i = 0; i < PIXEL_COUNT; i ++)
    {
        if(i != pixelCounter)
            // Clear the LED
            ring.setPixelColor(PIXEL_COUNT-i, 0, 0, 0);         
        else
            // Set LED to white
            ring.setPixelColor(PIXEL_COUNT-i, red, green, blue);  
    }
    pixelCounter ++;
    if(pixelCounter >= PIXEL_COUNT)
        pixelCounter =  0;
    ring.show();    
}
void Device::pulse_status_ring_yellow(uint16_t max_brightness){
  int R = 0;
  int G = 0;
  //Fade in
  for(R && G; R<max_brightness && G<max_brightness; R++ && G++) {
    for(int i=0; i<=ring.numPixels(); i++) {
      ring.setPixelColor(i, ring.Color(R, G, 0));
      ring.show();
      delay(0);
    }
    delay(1);
  }
  //Fade Out
  for(R && G; R>-1 && G>-1; R-- && G--) {
    for(int j=0; j<=ring.numPixels(); j++) {
      ring.setPixelColor(j, ring.Color(R, G, 0));
      ring.show();
      delay(0);
    }
    delay(1);
  }
}

void Device::pulse_status_ring_blue(uint16_t max_brightness){
  int B = 0;
  //Fade in
  for(B; B<max_brightness; B++) {
    for(int i=0; i<=ring.numPixels(); i++) {
      ring.setPixelColor(i, ring.Color(0, 0, B));
      ring.show();
      delay(0);
    }
    delay(1);
  }
  //Fade Out
  for(B; B>-1;B--) {
    for(int j=0; j<=ring.numPixels(); j++) {
      ring.setPixelColor(j, ring.Color(0,0,B));
      ring.show();
      delay(0);
    }
    delay(1);
  }
}

void Device::update_gps_location(){
    GPS.read();
    if (GPS.newNMEAreceived()) {
        Log.info(GPS.lastNMEA());
        Log.info(String(GPS.satellites));
        Log.info(String(GPS.fix));
        if(!GPS.parse(GPS.lastNMEA())){
          return;
        }
    }
}

std::tuple <float, float, float> Device::get_gps_location() {
    return {GPS.latitude, GPS.longitude, GPS.altitude};
}

// Battery Functions
float Device::get_voltage() { 
}

int Device::get_charge_state() {
}

// Logging Operations
uint32_t Device::write_datalog_entry(String filename, String data) {
}

uint32_t Device::get_datalog_size(String filename) {
}

// Display Functions

// Device Control
void Device::enable_debug_mode() {
}
void Device::enable_gps() {
    if (gps_enabled == false) {
        digitalWrite(GPS_ENABLE_PIN, HIGH);
        gps_enabled = true;
        delay(500);
    }
}
void Device::disable_gps() {
    if (gps_enabled == true){
        digitalWrite(GPS_ENABLE_PIN, LOW);
        gps_enabled = false;
    }
}

