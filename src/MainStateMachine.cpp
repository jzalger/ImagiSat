#include "MainStateMachine.h"

uint32_t wx_sampling_interval = 10000;
uint32_t health_update_interval = 60000;
uint32_t state_transmit_interval = 1000;
uint32_t gps_update_interval = 30000;
uint16_t button_debounce_time = 280;
uint64_t last_data_buffer_update = millis();
uint64_t last_state_transmit = millis();
uint64_t last_btn_press = millis();
uint64_t last_position_update = 0;
uint64_t last_wx_sample = 0;

Device device;
State state;

Indicator_State_t INDICATOR_STATE = IDLE;
States_t current_state = IDLE_STATE;
States_t last_state = IDLE_STATE;

int error = 0;
bool wb_rec_enabled = false;
bool gps_fix = false;
const int STATE_BUFFER_SIZE = 8;
uint32_t state_buffer_index = 0;
String filename = "temp";
environment_state state_buffer[STATE_BUFFER_SIZE];

//Task Handles
TaskHandle_t indicator_task_handle = NULL;
TaskHandle_t update_wx_conditions_task_handle = NULL;
TaskHandle_t update_gps_task_handle = NULL;
TaskHandle_t transmit_task_handle = NULL;  // TODO: Refactor this - legacy BT thing
TaskHandle_t monitor_mcp_handle = NULL;
// TODO: Rockblock transmit or receieve task?


MainStateMachine::MainStateMachine() {
}

MainStateMachine::~MainStateMachine() {
}

void MainStateMachine::setup() {
    log_info("Starting setup");
    error = device.device_setup();  

    if (error > 0) {
        state_handler = &MainStateMachine::error_state;
        return;
    }
    // Initialize Tasks
    xTaskCreate(
        indicator_task,
        "indicator_task",
        1024,
        NULL,
        1,
        &indicator_task_handle
    );
    log_info("Setup complete");
    state_handler = &MainStateMachine::test_state;
}

void MainStateMachine::loop() {
    if (state_handler) {
        state_handler(*this);
    }
    ui_update_loop();
}

// ########################################################################
// States
// ###########################################################################
void MainStateMachine::test_state() {
    INDICATOR_STATE = TEST;
    current_state = TEST_STATE;
    device.ui.test_ui_state();
    log_info("Entered test state");
    update_health_state();
    int error = device.test();
    if (error == 0){
        log_info("Exiting test state");
        device.ui.status_ui_state(state);  //TODO: This is probably the wrong spot
        update_main_state();
    } else {
        log_error("Exiting test state to ERROR");
        state.device_state.errors = 1;
        state_handler = &MainStateMachine::error_state;
    }
    last_state = TEST_STATE;
}

void MainStateMachine::idle_state() {
    //log_info("Entering idle state");
    INDICATOR_STATE = IDLE;
    current_state = IDLE_STATE;

    last_state = IDLE_STATE;
    update_main_state();
}

void MainStateMachine::update_location_state(){
    current_state = UPDATE_LOCATION_STATE;
    device.update_gps_location();
    std::tie(state.env_state.latitude, state.env_state.longitude, state.env_state.alititude, state.env_state.sats, gps_fix) = device.get_gps_location();
    state.env_state.time = millis();  // TODO: Fix with an actual time.
    //Manage LED Indicator
    if (gps_fix) {
        INDICATOR_STATE = GPS_LOCK;
        state.device_state.last_gps_lock_time = millis();
    } else {
        INDICATOR_STATE = GPS_SEARCHING;
    }
    last_state = UPDATE_LOCATION_STATE;
    update_main_state();
}

void MainStateMachine::sample_wx_condition_state(){
    current_state = SAMPLE_WX_CONDITION_STATE;
    std::tie(state.env_state.temperature, state.env_state.pressure, state.env_state.humidity, state.env_state.voc, state.env_state.p_alt) = device.get_wx_reading();
    last_state = SAMPLE_WX_CONDITION_STATE;
    last_wx_sample = millis();
    update_main_state();
}

void MainStateMachine::wb_receive_state() {
    current_state = WB_RECEIVE_STATE;
    last_state = WB_RECEIVE_STATE;
    update_main_state();
}

void MainStateMachine::iridium_receive_state(){
    current_state = IRIDIUM_RECEIVE_STATE;
    last_state = IRIDIUM_RECEIVE_STATE;
    update_main_state();
}

void MainStateMachine::iridium_send_receive_state(){
    current_state = IRIDIUM_SEND_RECEIVE_STATE;
    last_state = IRIDIUM_SEND_RECEIVE_STATE;
    update_main_state();
}

void MainStateMachine::error_state() {
    INDICATOR_STATE = ERROR;
    current_state = ERROR_STATE;
    log_error("Entered Error State");
    device.ui.error_ui_state("Entering Error State");

    for (;;){
        vTaskDelay(100);
    }

    last_state = ERROR_STATE;
    //update_main_state();
}

void MainStateMachine::update_health_state() {
    // Maybe move this to a task ?
    current_state = UPDATE_HEALTH_STATE;
    voltage = device.get_voltage();
    charge_state = device.get_charge_state();
    state.device_state.charge_state = charge_state;
    state.device_state.voltage = voltage;
    last_state = UPDATE_HEALTH_STATE;
    update_main_state();
}

void MainStateMachine::update_main_state() {
    device.ble_loop();
    if (wb_rec_enabled == true) {
        //FIXME: Needs to be driven by the UI state
        state_handler = &MainStateMachine::wb_receive_state;
    } else if (millis()- last_wx_sample > wx_sampling_interval) {
        state_handler = &MainStateMachine::sample_wx_condition_state;
    } else if (gps_fix == false || millis()-last_position_update > gps_update_interval){
        state_handler = &MainStateMachine::update_location_state;
    } else if (error > 1) {
        state_handler = &MainStateMachine::error_state;
    } else {
        state_handler = &MainStateMachine::idle_state;
    }
}

void ui_update_loop(){

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
        device.ui.update_indicator_state(INDICATOR_STATE);
        vTaskDelay(10);
    }
}

void transmit_state_task(void *parameter){
    for (;;) {
        device.transmit_state(state.env_state);
        vTaskDelay(state_transmit_interval);
    }
}
