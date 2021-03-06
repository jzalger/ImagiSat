#include "mbed.h"
#include "MAX17048.h"
#include "BMP085.h"
#include "SerialGPS.h"
#include "Arduino.h"
#include "BLEPeripheral.h"
#include "stdlib.h"
#include <string>

#define SCL PB_8  //Fuel Gauge, IMU
#define SDA PB_9
#define UART6_TX PA_11  //ROCKBLOCK
#define UART6_RX PA_12

//Power Management
#define BATT_ALERT_PIN PA_5
#define ALL_SLEEP_PIN PA_6
float battPercent = 0;
float battVoltage = 0;
//MAX17048 fuelGauge(SDA,SCL);  //(Low Bat % threashold, alert pin)

//GPS on Serial1
#define GPS_BAUD 9600
#define GPS_PPS PA_7
#define GPS_TX PA_9
#define GPS_RX PA_10
#define GPS_PWR_PIN PB_6
bool gpsLock = false;
bool gpsConnect = false;
DigitalOut GPS_PWR(GPS_PWR_PIN);
SerialGPS gps(GPS_TX,GPS_RX,GPS_BAUD);


//Device State Variables
bool gpsOn = false;
bool bluetoothOn = true;
bool wxLog = false;
bool rockBlockOn = false;

//Rockblock Config - Rockblock on Serial2
#define ROCKBLOCK_BAUD 19200

//USB Serial
Serial serial(USBTX, USBRX);


//Bluetooth runs SPI
#define MOSI D11
#define MISO D12
#define SCK D13
#define REQ D6
#define RDY D7     // This should be an interrupt pin, on Uno thats #2 or #3
#define RST D5
#define BT_PWR_PIN PA_9
SPI spi(MOSI, MISO, SCK);
DigitalInOut BLE_RDY(RDY);
DigitalInOut BLE_REQ(REQ);
DigitalInOut BLE_RESET(RST);
DigitalOut BLE_PWR(BT_PWR_PIN);

/*----- BLE Utility -------------------------------------------------------------------------*/
BLEPeripheral            blePeripheral        = BLEPeripheral(&BLE_REQ, &BLE_RDY, &BLE_RESET);
BLEService               uartService          = BLEService("713d0000503e4c75ba943148f18d941e");
BLECharacteristic    txCharacteristic = BLECharacteristic("713d0002503e4c75ba943148f18d941e", BLENotify, 20);
BLECharacteristic    rxCharacteristic = BLECharacteristic("713d0003503e4c75ba943148f18d941e", BLEWriteWithoutResponse, 20);
unsigned char txbuf[16] = {0};
unsigned char txlen = 0;
unsigned int interval = 0;
unsigned char count_on = 0;

//BMP085 Pressure Sensor (I2C)
BMP085 psens(SDA,SCL);
struct wxReading {
    float pressure;
    float temp;
    float alt;
    float slp;
};

//Create a struct to hold the current weather reading
wxReading currentWx = {0,0};

void initBLE()
{
    /*----- BLE Utility ---------------------------------------------*/
    // set advertised local name and service UUID
    blePeripheral.setLocalName("ImagiSat");

    blePeripheral.setAdvertisedServiceUuid(uartService.uuid());

    // add service and characteristic
    blePeripheral.addAttribute(uartService);
    blePeripheral.addAttribute(rxCharacteristic);
    blePeripheral.addAttribute(txCharacteristic);

    // begin initialization
    blePeripheral.begin();
    /*---------------------------------------------------------------*/

    serial.printf("BLE Initialization Complete!\r\n");
}

void sendDataToBluetooth(const unsigned char* message)
{
    unsigned char len = strlen((const char *)message);
    txCharacteristic.setValue((const unsigned char *)message, len);
}

void toggleGPSPower()
{
    if (gpsOn == true) {
        GPS_PWR = 0;
        gpsOn = false;
    }  else if (gpsOn == false) {
        GPS_PWR = 1;
        gpsOn = true;
    }
}

void toggleBTPower()
{
    if (bluetoothOn == true) {
        BLE_PWR = 0;
        bluetoothOn = false;
    } else if (bluetoothOn == false) {
        BLE_PWR = 1;
        bluetoothOn = true;
    }
}
void toggleRBPower()
{
    if (rockBlockOn == true) {
        //RB_PWR = 0;
        rockBlockOn = false;
    } else if (rockBlockOn == false) {
        //RB_PWR = 1;
        rockBlockOn = true;
    }
}

void toggleWxLogging()
{
    if (wxLog == true) {
        wxLog = false;
    } else if (wxLog == false) {
        wxLog = true;
    }
}

void getBattLevel()
{
//fuelGauge.wake();
//battVoltage = fuelGauge.getBatteryVoltage();
//battPercent = fuelGauge.getBatteryPercentage();
//fuelGauge.sleep();
}

/*
void sendBattLevel() {
 char outBuff[10];
 int len = strlen(outBuff);
 PString str(outBuff, sizeof(outBuff) - len);
 str.print(battVoltage);
 str.print(",");
 str.print(battPercent);
 sendDataToBT(outBuff);
}
*/

void sendLocation()
{
    serial.printf("Send Location!\r\n");
    if (gps.sample()) {
        gpsLock = true;
        char gpsString[128];
        sprintf(gpsString,"sats %d, long %f, lat %f, alt %f, geoid %f, time %f\n\r", gps.sats, gps.longitude, gps.latitude, gps.alt, gps.geoid, gps.time);
        serial.printf(gpsString);
        sendDataToBluetooth((const unsigned char *)gpsString);
    } else {
        serial.printf("No Location");
        char gpsString[12] = "No Location";
        sendDataToBluetooth((const unsigned char *)gpsString);
    }
}

void getWxReading()
{
    //psens.update();
    //float pressure = psens.get_pressure();
    //float temp = psens.get_temperature();
    //currentWx.temp = temp;
    //currentWx.pressure = pressure;
    //currentWx.slp = psens.readSealevelPressure();
    //currentWx.alt = psens.readAltitude();
}

void sendWxReading()
{
    //char outBuffer[60];
    //int len = strlen(outBuffer);
    //PString str(outBuffer, sizeof(outBuffer) - len);
    //str.print(currentWx.temp, 3);
    //str.print(",");
    //str.print(currentWx.pressure, 3);

//    sendDataToBT(outBuffer);
}



void actionBluetooth(const unsigned char *rxBuf, int rxlen)
{
    //React to requests from bluetooth
    int cmd = atoi((const char *)rxBuf);
    serial.printf("%d\r\n", cmd);

    switch (cmd) {
        case 1: //Send location
            sendLocation();
            break;
        case 2: //Send sensor data
            break;
    }
}

void serviceBluetooth()
{
    BLECentral central = blePeripheral.central();
    if (central) {
        // central connected to peripheral
        serial.printf("Connected to central\r\n");
        while (central.connected()) {
            // Receive
            if (rxCharacteristic.written()) {
                unsigned char rxlen = rxCharacteristic.valueLength();
                const unsigned char *val = rxCharacteristic.value();
                serial.printf("Received, Length: %d\r\n", rxlen);
                actionBluetooth(val, rxlen);
            }

            //Send to BT from connected terminal
            if(serial.readable()) { // Do not spend much time on doing other things when serial available! Otherwisee, data will lose.
                if(!count_on) {
                    count_on = 1;
                }
                interval = 0;
                txbuf[txlen] = serial.getc();
                txlen++;
            }
            if(count_on) {   // Count the interval after receiving a new char from terminate
                interval++;
            }

            if(interval == 10) {  // If there is no char available last the interval, send the received chars to central.
                interval = 0;
                count_on = 0;
                txCharacteristic.setValue((const unsigned char *)txbuf, txlen);
                txlen = 0;
            }
        }
        // Peripheral disconnected
        serial.printf("Disconnected from central\r\n");
    }
}

int main()
{
    serial.baud(115200);
    serial.printf("Serial begin!\r\n");
    initBLE();

    while(1) {
        serviceBluetooth();
    }
}
