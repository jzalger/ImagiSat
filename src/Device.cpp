#include "Device.h"

#define USB_SERIAL_ACTIVE 1  // 1 for TRUE

bool debug_mode = true;

HardwareSerial iridium_serial(1);
IridiumSBD iridium_modem(iridium_serial, -1, IR_RING_PIN);
int iridium_signal_quality = 0;
bool iridium_diagnostics = false;


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

Adafruit_BME680 bme;
SparkFun_AS3935 lightning(AS3935_ADDR);
int lightning_value = 0;

// ###############################################################################
// Device Class
// ###############################################################################

Device::Device() {
  UIStateMachine ui;
}

Device::~Device() { 
}

uint16_t Device::device_setup() {
    int error = 0;
    log_info("Beginning device setup");
    pinMode(VOLTAGE_READ_PIN, INPUT);
    if (USB_SERIAL_ACTIVE){
      Serial.begin(115200);
    }
    Wire.begin();
    ring.begin();
    ring.setBrightness(50);
    ring.show();
    error += ui.setup(); 

    if (ble_enabled){
      log_info("Running BLE Setup");
      // FIXME: Something with interupts in BLE setup creates a kernel panic with tasks
      ble_setup();
      log_info("Completed BLE Setup");
    }

    pinMode(GPS_ENABLE_PIN, OUTPUT);
    if (gps_enabled) {
      enable_gps();
      if (!GPS.begin()){
        log_error("GPS init failed");
        error += 1;
      }
      GPS.setAutoPVTcallback(&gps_data_callback);
      if (debug_mode && USB_SERIAL_ACTIVE){
        // GPS.setNMEAOutputPort(Serial);
      }
    }
    pinMode(AS3935_INT_PIN, INPUT);
    if (lightning_enabled) {
      if(!lightning.begin()){
        log_error("AS3935 Lighning Sensor init failed");
        error += 1;
      } else {
        attachInterrupt(AS3935_INT_PIN, lightning_detect_callback, RISING);
        log_info("Completed lightning sensor init");
      }
    }
    //init_si4707();
    //iridium_setup();
    if (bme680_enabled) {
      bme680_setup();
    }
    return error;
}

void Device::iridium_setup(){
  iridium_serial.begin(19200);
  iridium_modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE);
  if (debug_mode){
    iridium_diagnostics = true;
  }
  if (iridium_modem.begin() != ISBD_SUCCESS){
    log_error("Could not connect to Iridium modem");
  }
}

uint16_t Device::test() {
  // Return 0 if all tests passed
  log_info("Running device test");
  int error = 0;
  error += _test_device_health();
  error += _test_gps();
  error += _test_bme();
  //error += _test_iridium();
  return error;
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

bool get_gps_fix(){
  bool gps_fix = false;
  if (gps_data.numSV > 0){
    gps_fix = true;
  }
  return gps_fix;
}

std::tuple <float, float, float, int, bool> Device::get_gps_location() {
  bool _gps_fix = get_gps_fix();
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
std::tuple <float, float, float, float, float> Device::get_wx_reading(){
  return std::make_tuple(bme.temperature, bme.pressure, bme.humidity, bme.gas_resistance, bme.readAltitude(SEALEVELPRESSURE_HPA));
}

std::tuple <float, int> Device::get_battery_health() {
    return std::make_tuple(get_voltage(), get_charge_state());
}

// Battery Functions
float Device::get_voltage() {
  float voltage = analogRead(VOLTAGE_READ_PIN)/1024; 
  return voltage * 2;  // Voltage is read via a 1/2 divider
}

int Device::get_charge_state() {
  float voltage = get_voltage();
  int charge_state = floor(voltage/MAX_BAT_VOLTAGE*100);
  return charge_state;
}

uint32_t Device::write_datalog_entry(String filename, String data) {
  return 0;
}

uint64_t Device::get_datalog_size(String filename) {
  return 0;
}

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
  if (USB_SERIAL_ACTIVE){
    Serial.println(s);
  }
}

void IRAM_ATTR lightning_detect_callback() {
  lightning_value = lightning.readInterruptReg();
  if (lightning_value == AS3935_NOISE_INT){
    // TODO: Careful not to write to log during the interrupt. Set a flag instead, and check the flag state in the loop/state update

    //log_info("Lightning noise detected");
    // lightning.setNoiseLevel(setNoiseLevel); to change noise level
  } else if (lightning_value == AS3935_DISTURBER_INT) {
    //log_info("Lightning disturber detected");
    // Too many disturbers? Uncomment the code below, a higher number means better
    // disturber rejection.
    // lightning.watchdogThreshold(threshVal);  
  } else if (lightning_value == AS3935_LIGHTNING_INT) {
    byte distance = lightning.distanceToStorm();
    //log_info("Lightning Detected!");
    //log_info(distance +  " km away");
  }
}

void Device::bme680_setup() {
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  if (!bme.begin()) {
      log_error("Could not find a valid BME680 sensor, check wiring!");
  }
}


byte Device::init_si4707()
{
  // Set initial pin value: RST (Active-low reset)
  pinMode(WB_RST_PIN, OUTPUT);  // Reset
  digitalWrite(WB_RST_PIN, LOW);  // Keep the SI4707 in reset
  delay(1);  // Short delay before we take reset up
  digitalWrite(WB_RST_PIN, HIGH);
  delay(1);  // Give Si4707 a little time to reset
  wb::powerUp();
  log_debug("Completed powerup sequence");
  return wb::command_Get_Rev(1); // Ideally returns 7
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

// ###############################################################################
// Device Test Methods
uint16_t Device::_test_gps(){
  return 0;
}

uint16_t Device::_test_bme(){
    std::tuple <float, float, float, float, float> reading = get_wx_reading();
    log_info("BME680 Test: Temperature: " + String(std::get<0>(reading)) + " - Humidity: " + String(std::get<2>(reading)) + " - VOC: " + String(std::get<3>(reading)));
    return 0;
}

uint16_t Device::_test_iridium(){
  int error = iridium_modem.getSignalQuality(iridium_signal_quality);
  if (error != 0){
    log_warning("Could not get Iridium Signal Quality");
    return 1;
  } else {
    log_info("Iridium signal quality: " + String(iridium_signal_quality));
    return 0;
  }
}

uint16_t Device::_test_device_health(){
  float voltage = get_voltage();
  int charge = get_charge_state();
  // Return an error code if charge too low.
  log_info("Device voltage: " + String(voltage) + " - Device charge state: " + String(charge));
  return 0;
}

// ###############################################################################
// UI State Machine
// ###############################################################################

UIStateMachine::UIStateMachine() {
  HapticDevice haptic;
  Indicator indicator;
}

UIStateMachine::~UIStateMachine() {
}

uint8_t UIStateMachine::setup(){
  uint8_t error = 0;
  //haptic.setup(); TODO: Remove when hardware is implemented
  return error;
}

void UIStateMachine::test_ui_state(){
  current_ui_state = TEST_UI;
  haptic.notice();
}

void UIStateMachine::error_ui_state(String error_msg){
  current_ui_state = ERROR_UI;
  haptic.notice();
}


void UIStateMachine::status_ui_state(State state) {
  current_ui_state = STATUS_UI;
}

void UIStateMachine::wb_rec_ui_state(State state){
  current_ui_state = WB_RADIO_UI;
}

void UIStateMachine::gps_searching_state(){
}

void UIStateMachine::forecast_ui_state(State state){
  current_ui_state = FORECAST_UI;
}

void UIStateMachine::iridium_msg_ui_state(State state){
  current_ui_state = IRIDIUM_MSG_UI;
}

void UIStateMachine::alert_ui_state(Alert alert){
  current_ui_state = ALERT_UI;
  haptic.alert();
}

void UIStateMachine::refresh(){
}

void UIStateMachine::button_event_handler(State state) {
}

void UIStateMachine::update_ui_state(uint8_t state_num, State state){
   switch (state_num){
    case STATUS_UI:
      status_ui_state(state);
      break;
    case WB_RADIO_UI:
      wb_rec_ui_state(state);
      break;
    case FORECAST_UI:
      forecast_ui_state(state);
      break;
    case IRIDIUM_MSG_UI:
      iridium_msg_ui_state(state);
      break;
  } 
}

void UIStateMachine::modify_ui_state(UI_Action_t action){
  // TODO: Might need to refactor the ui states to classes with modifier methods.
}

void UIStateMachine::update_indicator_state(Indicator_State_t state){
  switch (state) {
    case IDLE:
        // Pulse white
        // indicator.pulse(50,50,50,30);
        break;
    case GPS_SEARCHING:
        // Cycle green
        indicator.cycle(0,100,0,30);
        break;
    case GPS_LOCK:
        // Pulse Green
        indicator.pulse(0,100,0,30);
        break;
    case TEST:
        // Cycle white
        indicator.cycle(100,100,100,30);
        break;
    case ERROR:
        // Pulse orange
        indicator.pulse(100,50,0,30);
        break;
    case IRIDIUM_SENDING:
        //Cycle Blue
        indicator.cycle(0,0,100,30);
        break;
    case IRIDIUM_SENT:
        // Pulse Blue
        indicator.pulse(0,0,100,30);
        break;
    case IRIDIUM_RECEIVED:
        // Pulse Magenta
        indicator.pulse(100,0,100,30);
        break;
    case WX_ALERT:
        // Pulse red
        indicator.pulse(100,0,0,30);
        break;
    default:
        break;
    }
}


// ###############################################################################
// Haptic Device
// ###############################################################################

HapticDevice::HapticDevice(){

}

HapticDevice::~HapticDevice(){

}

void HapticDevice::setup(){
  pinMode(HAPTIC_DEVICE_PIN, OUTPUT);
}

void HapticDevice::vibrate(int milliseconds) {
/*   digitalWrite(HAPTIC_DEVICE_PIN, HIGH);
  vTaskDelay(milliseconds);
  digitalWrite(HAPTIC_DEVICE_PIN, LOW); */
}

void HapticDevice::alert(){
/*   vibrate(2000);
  vTaskDelay(1000);
  vibrate(2000); */
}

void HapticDevice::notice(){
/*   vibrate(1000);
  vTaskDelay(1000);
  vibrate(1000); */
}

void HapticDevice::tap(){
  //vibrate(100);
}

// ###############################################################################
// Indicator
// ###############################################################################
Indicator::Indicator(){

}

Indicator::~Indicator(){

}

void Indicator::pulse(uint16_t red, uint16_t green, uint16_t blue, uint16_t brightness){
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

void Indicator::cycle(uint16_t red, uint16_t green, uint16_t blue, uint16_t brightness){
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


// #####################################################################
void log_debug(String str){
  if (debug_mode){
    _write_log("DEBUG", str);
  }
}
void log_error(String str) {
  _write_log("ERROR", str);
}
void log_info(String str) {
  _write_log("INFO", str);
}
void log_warning(String str) {
  _write_log("WARNING", str);
}
void _write_log(String level, String str) {
    if (debug_mode){
      String s = level + ": " + str;
      if (USB_SERIAL_ACTIVE){
        Serial.println(s);
      }
    }
}

