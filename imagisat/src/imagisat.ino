#include "MainStateMachine.h"
#include "Device.h"
#include "types.h"

MainStateMachine mainStateMachine;


void setup()
{
	mainStateMachine.setup();
}

void loop()
{
	mainStateMachine.loop();
}
