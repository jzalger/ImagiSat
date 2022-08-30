#include "MainStateMachine.h"

uint32_t data_buffer_log_interval = 1000;
uint32_t health_update_interval = 12000;
uint32_t state_transmit_interval = 1000;
uint32_t gps_update_interval = 1000;

uint64_t last_data_buffer_update = millis();
uint64_t last_state_transmit = millis();

Device device;
environment_state env_state;

Indicator_State_t INDICATOR_STATE = IDLE;

bool gps_fix = false;
const int STATE_BUFFER_SIZE = 8;
uint32_t state_buffer_index = 0;
String filename = "temp";
environment_state state_buffer[STATE_BUFFER_SIZE];

//Task Handles
TaskHandle_t indicator_task_handle = NULL;
TaskHandle_t update_data_buffer_task_handle = NULL;
TaskHandle_t gps_update_task_handle = NULL;
TaskHandle_t transmit_task_handle = NULL;  // TODO: Refactor this - legacy BT thing
// TODO: Rockblock transmit or receieve task?

MainStateMachine::MainStateMachine() {
}

MainStateMachine::~MainStateMachine() {
}

void MainStateMachine::setup() {
    int error = 0;
    device.log_info("Starting setup");
    error = device.device_setup();  
    if (error > 0) {
        state_handler = &MainStateMachine::error_state;
        return;
    }
    debug_mode = device.debug_mode;

    // Initialize Tasks
    // xTaskCreate(
    //     transmit_state_task,
    //     "transmit_state_task",
    //     4096,
    //     NULL,
    //     1,
    //     &transmit_task_handle
    // );
    xTaskCreate(
        indicator_task,
        "indicator_task",
        1024,
        NULL,
        1,
        &indicator_task_handle
    );
    // xTaskCreate(
    //     update_data_buffer_task,
    //     "update_data_buffer_task",
    //     4096,
    //     NULL,
    //     1,
    //     &update_data_buffer_task_handle
    // );

    device.log_info("Setup complete");
    if (debug_mode) {
        state_handler = &MainStateMachine::debug_state;
    } else {
        state_handler = &MainStateMachine::test_state;
    }   
}

void MainStateMachine::loop() {
    if (state_handler) {
        state_handler(*this);
    }
}

// ########################################################################
// States
// ###########################################################################
void MainStateMachine::idle_state() {
    INDICATOR_STATE = IDLE;
    device.ble_loop();
    update_main_state();
}

void MainStateMachine::test_state() {
    INDICATOR_STATE = TEST;
    device.log_info("Entered test state");
    update_health_state();
    state_handler = &MainStateMachine::idle_state;
}

void MainStateMachine::debug_state() {
    // Enable additional functionality and logging
    // TODO: This might not need to be a state, just a prefunction or check in the test state
    device.log_info("Debug state enabled");
    state_handler = &MainStateMachine::test_state;
}

void MainStateMachine::get_location_state() {
    device.update_gps_location();
    std::tie(env_state.latitude, env_state.longitude, env_state.alititude, env_state.sats, gps_fix) = device.get_gps_location();
    env_state.time = millis();  // TODO: Fix with an actual time.

    //Manage LED Indicator
    if (gps_fix) {
        INDICATOR_STATE = GPS_LOCK;
    } else {
        INDICATOR_STATE = GPS_SEARCHING;
    }
    device.ble_loop();
    update_main_state();
}

void MainStateMachine::wx_radio_listen_state() {

}

void MainStateMachine::sample_wx_state() {

}

void MainStateMachine::error_state() {
    INDICATOR_STATE = ERROR;
    device.log_error("Entered Error State");
    device.ble_loop();
}

void MainStateMachine::update_health_state() {
    voltage = device.get_voltage();
    charge_state = device.get_charge_state();
}

void MainStateMachine::update_main_state() {
    if (tracking == true) {
        state_handler = &MainStateMachine::get_location_state;
    } else {
        state_handler = &MainStateMachine::error_state;
    }
}

void update_state_buffer(environment_state state){
    state_buffer[state_buffer_index] = state;
    if (state_buffer_index == STATE_BUFFER_SIZE-1){
        String log_str = "";
        for (int i = 0;i < STATE_BUFFER_SIZE; ++i){
            log_str = log_str + csv_string(state_buffer[i], millis()) + "\n";           
        }
        device.write_datalog_entry(filename, log_str);
        state_buffer_index = 0;
    } else {
        ++state_buffer_index;
    }
}

// ########################################################################
// State Task Handlers
// ###########################################################################
void indicator_task(void *parameter){
    for (;;){
        switch (INDICATOR_STATE)
        {
        case IDLE:
            break;
        case GPS_SEARCHING:
            device.cycle_status_ring(100,100,0,30);
            break;
        case GPS_LOCK:
            device.pulse_status_ring(0,200,0,30);
            break;
        case TEST:
            break;
        case ERROR:
            device.pulse_status_ring(200,0,0,30);
        default:
            break;
        }
        vTaskDelay(20);
    }
}

void update_data_buffer_task(void *parameter){
    for (;;){
        update_state_buffer(env_state);
        vTaskDelay(data_buffer_log_interval);
    }
}

void transmit_state_task(void *parameter){
    for (;;) {
        device.transmit_state(env_state);
        vTaskDelay(state_transmit_interval);
    }
}

void update_gps_task(void *parameter){
    for (;;) {
        device.update_gps_location();
        vTaskDelay(gps_update_interval);
    }
}
