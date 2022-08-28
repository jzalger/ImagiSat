#ifndef __COMM_H
#define __COMM_H
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID           "2e63e1bf-4e03-41fa-9f61-bb3652ebf92b" // UART service UUID
#define CHARACTERISTIC_UUID_RX "2e63e1bf-4e03-41fa-9f61-bb3652ebf92c"
#define CHARACTERISTIC_UUID_TX "2e63e1bf-4e03-41fa-9f61-bb3652ebf92d"

void ble_setup();
void ble_loop();

#endif
