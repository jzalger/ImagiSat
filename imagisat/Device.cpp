#include "Device.h"

#define USB_SERIAL_ACTIVE 1  // 1 for TRUE

HardwareSerial hwd_serial(1);
SFE_UBLOX_GNSS GPS;
UBX_NAV_PVT_data_t gps_data;
Adafruit_NeoPixel ring = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
int pixel_counter = 0;
int pixel_fade_counter = 0;
bool pixel_fade_in = true;

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool ble_device_connected = false;
bool old_ble_device_connected = false;

Device::Device() {
}

Device::~Device() { 
}

uint16_t Device::device_setup() {
    int error = 0;
    log_info("Beginning device setup");
    //WiFi.mode(WIFI_OFF);
    pinMode(VOLTAGE_READ_PIN, INPUT);
    pinMode(DEBUG_PIN, INPUT);
    debug_mode = digitalRead(DEBUG_PIN);
    hwd_serial.begin(9600);
    if (USB_SERIAL_ACTIVE){
      Serial.begin(115200);
    }
    Wire.begin();
    ring.begin();
    ring.setBrightness(50);
    ring.show();

    if (ble_enabled){
      log_info("Running BLE Setup");
      // FIXME: Something with interupts in BLE setup creates a kernel panic with tasks
      ble_setup();
      log_info("Completed BLE Setup");
    }
    pinMode(GPS_ENABLE_PIN, OUTPUT);
    enable_gps(); // Turn on GPS by default
    if (!GPS.begin()){
      log_error("GPS init failed");
      error += 1;
    }
    GPS.setAutoPVTcallback(&gps_data_callback);
    if (debug_mode){
      //GPS.setNMEAOutputPort(hwd_serial);
    }
    return error;
}
 
uint16_t Device::test() {
  // Return 0 if all tests passed
  int error = 0;
  error += _test_gps();
  error += _test_led_ui();
  return error;
}

// #####################################################################
// DEVICE FUNCTIONS
// #####################################################################
void Device::cycle_status_ring(uint16_t red, uint16_t green, uint16_t blue, uint16_t brightness) {
    //ring.setBrightness(brightness);
    for(int i = 0; i < PIXEL_COUNT; i ++) {
        if(i != pixel_counter)
            ring.setPixelColor(PIXEL_COUNT-i, 0, 0, 0);         
        else
            ring.setPixelColor(PIXEL_COUNT-i, red, green, blue);  
    }
    pixel_counter ++;
    if(pixel_counter >= PIXEL_COUNT){
        pixel_counter =  0;
    }
    ring.show();
    vTaskDelay(30);    
}
void Device::pulse_status_ring(uint16_t red, uint16_t green, uint16_t blue, uint16_t brightness) {
  int tmpR, tmpG, tmpB;
  int steps = 50;
  int pause = 30;

  // Fade up
  for (int s=1; s<=steps; s++) {
    tmpR = (red * s) / steps;     
    tmpG = (green * s) / steps;
    tmpB = (blue * s) / steps;

    for (int i=0; i < PIXEL_COUNT; i++) {
      ring.setPixelColor(i,tmpR,tmpG,tmpB);
    }
    ring.show();
    vTaskDelay(pause);
  }    

  // Fade down
  for (int s=steps; s>0; s--) {
    tmpR = (red * s) / steps;     
    tmpG = (green * s) / steps;
    tmpB = (blue * s) / steps;

    for (int i=0; i < PIXEL_COUNT; i++) {
      ring.setPixelColor(i,tmpR,tmpG,tmpB);
    }
    ring.show();
    vTaskDelay(pause);
  }    
}

void gps_data_callback(UBX_NAV_PVT_data_t ubxDataStruct){
  gps_data = ubxDataStruct;
}

void Device::update_gps_location(){
  GPS.checkUblox();
  GPS.checkCallbacks();
}

uint8_t Device::get_gps_fix_type(){
  return GPS.getFixType();
}

std::tuple <float, float, float, int, bool> Device::get_gps_location() {
  bool _gps_fix = false;
  if (gps_data.numSV > 0){
    _gps_fix = true;
  }
  return std::make_tuple(gps_data.lat, gps_data.lon, gps_data.height, gps_data.numSV, _gps_fix);
}

float Device::get_latitude(){
  return GPS.getLatitude();
}

float Device::get_longitude(){
  return GPS.getLongitude();
}

float Device::get_altitude(){
  return GPS.getAltitude();
}
float Device::get_temperature(){
    return 0;
}

std::tuple <float, int> Device::get_battery_health() {
    return std::make_tuple(get_voltage(), get_charge_state());
}

// Battery Functions
float Device::get_voltage() {
  float voltage = analogRead(VOLTAGE_READ_PIN); 
  return voltage * 2;  // Voltage is read via a 1/2 divider
}

int Device::get_charge_state() {
  float voltage = get_voltage();
  int charge_state = floor(voltage/MAX_BAT_VOLTAGE);
  return charge_state;
}

uint32_t Device::write_datalog_entry(String filename, String data) {
}

// uint32_t Device::_write_datalog_entry(fs::FS &fs, String filename, String data) {
// }

uint64_t Device::get_datalog_size(String filename) {
}

// uint64_t Device::_get_datalog_size(fs::FS &fs, String filename) {
// }

// Device Control
void Device::enable_debug_mode() {
}

void Device::enable_gps() {
    if (gps_enabled == false) {
      digitalWrite(GPS_ENABLE_PIN, LOW);
      delay(1000);
      digitalWrite(GPS_ENABLE_PIN, HIGH);
      delay(1000);
      digitalWrite(GPS_ENABLE_PIN, LOW);
      gps_enabled = true;
    }
}
void Device::disable_gps() {
    if (gps_enabled == true){
        GPS.powerOff(200000000);
        gps_enabled = false;
    }
}
void Device::enable_ble(){
  ble_enabled = true;
  ble_setup();
}
void Device::disable_ble(){
  ble_enabled = false;
}
void Device::transmit_state(environment_state state) {
  String state_string = csv_string(state, millis());
  String s = "STATE:" + state_string;
  // TODO: Abstract this to easily switch between serial and bluetooth

  hwd_serial.println(s);
  if (USB_SERIAL_ACTIVE){
    Serial.println(s);
  }
}

// #####################################################################
// BLE Functions
// #####################################################################
class DeviceBLEServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      ble_device_connected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      ble_device_connected = false;
    }
};

class DeviceBLECallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        for (int i = 0; i < rxValue.length(); i++){
          Serial.println(rxValue[i]);
        }
      }
    }
};

void Device::ble_setup() {
  // Create the BLE Device
  BLEDevice::init("ImagiSat");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new DeviceBLEServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
										CHARACTERISTIC_UUID_TX,
										BLECharacteristic::PROPERTY_NOTIFY
									);
                      
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
											 CHARACTERISTIC_UUID_RX,
											BLECharacteristic::PROPERTY_WRITE
										);

  pRxCharacteristic->setCallbacks(new DeviceBLECallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
}

void Device::ble_transmit(uint8_t *ble_tx_buffer, int ble_tx_buff_size) {
  if (ble_device_connected) {
    for (uint8_t i=0; i < ble_tx_buff_size-1; ++i){
      pTxCharacteristic->setValue(&ble_tx_buffer[i], 1);
      pTxCharacteristic->notify();
      delay(10); // bluetooth stack will go into congestion, if too many packets are sent
    }
  }
}

void Device::ble_loop() {
  if (ble_enabled) {
    // disconnecting
    if (!ble_device_connected && old_ble_device_connected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        log_debug("Starting advertising BLE");
        old_ble_device_connected = ble_device_connected;
    }
    // connecting
    if (ble_device_connected && !old_ble_device_connected) {
		// do stuff here on connecting
        old_ble_device_connected = ble_device_connected;
    }
  }
}

// #####################################################################
void Device::log_debug(String str){
  if (debug_mode){
    _write_log("DEBUG", str);
  }
}
void Device::log_error(String str) {
  _write_log("ERROR", str);
}
void Device::log_info(String str) {
  _write_log("INFO", str);
}
void Device::log_warning(String str) {
  _write_log("WARNING", str);
}
void Device::_write_log(String level, String str) {
    if (debug_mode){
      String s = level + ": " + str;
      hwd_serial.println(s);
      if (USB_SERIAL_ACTIVE){
        Serial.println(s);
      }
    }
}

// #####################################################################
// Device Test Methods
// #####################################################################
uint16_t Device::_test_gps(){
  return 0;
}

uint16_t Device::_test_led_ui() {
  return 0;
}
