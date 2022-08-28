// Public domain (CC0) 
// Can be used in open or closed-source commercial projects and derivative works without attribution.

#include "MainStateMachine.h"

uint32_t display_update_timer = millis();
const std::chrono::milliseconds display_update_interval = 30s;
const std::chrono::milliseconds data_log_interval = 2s;


MainStateMachine::MainStateMachine() {
}

MainStateMachine::~MainStateMachine() {
}

void MainStateMachine::setup() {
    device.device_setup();  
    state_time = millis();
    debug_mode = device.debug_mode;
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

void MainStateMachine::idle_state() {
    Log.info("Entered Idle State");
    update_main_state();
}

void MainStateMachine::test_state() {
    Log.info("Entered test state");
    state_handler = &MainStateMachine::idle_state;
}

void MainStateMachine::debug_state() {
    Log.info("Debug state enabled");
    state_handler = &MainStateMachine::test_state;
}

void MainStateMachine::error_state() {
}

void MainStateMachine::sbd_transmit() {
}

void MainStateMachine::sbd_receive() {
}

void MainStateMachine::handle_ble() {
}

void MainStateMachine::wb_listen() {
    //Listen to Weather band radio
}

void MainStateMachine::wx_listen() {
    // Record Weather data
}

void MainStateMachine::wx_alert(){
    //Triggered by WB alert tone or lightning interrupt
}
    

void MainStateMachine::update_main_state() {
    state_handler = &MainStateMachine::error_state;
    
}

void MainStateMachine::update_health_state() {
    voltage = device.get_voltage();
    charge_state = device.get_charge_state();
    // Possibly add check on GPS or RBD status
}
