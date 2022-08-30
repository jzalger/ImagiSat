#ifndef __MAINSTATEMACHINE_H
#define __MAINSTATEMACHINE_H

#include "Device.h"
#include "types.h"
#include <functional>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"

enum Indicator_State_t {
    IDLE,
    GPS_SEARCHING,
    GPS_LOCK,
    IRIDIUM_SENDING,
    IRIDIUM_SENT,
    IRIDIUM_RECEIVED,
    WX_ALERT,
    TEST,
    ERROR
};

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
    void error_state();
    void get_location_state();
    void update_main_state();
    void update_health_state();
    void wb_receive_state();
    void iridium_send_receive_state();
    void iridium_receive_state();

    unsigned long state_time;
	std::function<void(MainStateMachine&)> state_handler = 0;  

    bool tracking = true;
    bool debug_mode = false;
    double voltage = 0.0;
    int charge_state = 0;
};

void update_state_buffer(environment_state state);

// State Tasks
void indicator_task(void *parameter);
void transmit_state_task(void *parameter);
void update_gps_task(void *parameter);
void update_depth_task(void *parameter);
void update_data_buffer_task(void *parameter);

#endif /* __MAINSTATEMACHINE_H */
