#ifndef __MAINSTATEMACHINE_H
#define __MAINSTATEMACHINE_H

#include "Device.h"

class MainStateMachine {
public:
    MainStateMachine();
    virtual ~MainStateMachine();

    void setup();
    void loop();

protected:
    // State handlers
    void idle_state();
    void test_state();
    void debug_state();
    void error_state();
    void update_health_state();
    void update_main_state();
    Device device;

    unsigned long state_time;
	std::function<void(MainStateMachine&)> state_handler = 0;  

    //State Variables    
    bool debug_mode = false;
    double voltage = 0.0;
    int charge_state = 0;
};

#endif /* __MAINSTATEMACHINE_H */

