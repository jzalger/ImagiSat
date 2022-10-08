#include "MainStateMachine.h"
#include "Device.h"
#include "types.h"

//#include <esp32fota.h>
//#include <WiFi.h>

const uint8_t current_firmware_version = 0;

MainStateMachine mainStateMachine;

//WiFiClientSecure ota_client;
//esp32FOTA esp32FOTA("ImagiSat", current_firmware_version, false);

void setup()
{
	// Check for firmware update 
	pinMode(DFU_PIN, INPUT);
	if(digitalRead(DFU_PIN)==HIGH) {
		// Enter OTA update mode
	}
	mainStateMachine.setup();
}

void loop()
{
	mainStateMachine.loop();
}
