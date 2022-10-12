#include "MainStateMachine.h"
#include "Device.h"
#include "types.h"

// See below
//#include <esp32fota.h>
//#include <WiFi.h>

const uint8_t current_firmware_version = 0;

MainStateMachine mainStateMachine;

// FIXME: This seems create strange blocking behaviour - only activate when in DFU mode. Perhaps the indicator task is being interfered with.
// Just including the headers seems to do this, whether instantiated or not. Need to understand how to deconflict this, perhaps enable on separate cores?
//WiFiClientSecure ota_client;
//esp32FOTA esp32FOTA("ImagiSat", current_firmware_version, false);

void setup()
{
	// Check for firmware update 
	pinMode(DFU_PIN, INPUT);
	if(digitalRead(DFU_PIN)==HIGH) {
		// Enter OTA update mode
		log_debug("Entering DFU Mode");
	}
	mainStateMachine.setup();
}

void loop()
{
	mainStateMachine.loop();
}
