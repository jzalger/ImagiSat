#include MainStateMachine.h
#include <unity.h>

void void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_iridium_send_state);
    RUN_TEST(test_iridium_receive_state);
    RUN_TEST(test_iridium_send_receive_state);
    UNITY_END();
}

void test_iridium_send_state(){
    state_machine = MainStateMachine();
    state_machine.iridium_send_state();
    assert(state_machine.current_state == IRIDIUM_SEND_STATE);
}

void test_iridium_receive_state(){
    state_machine = MainStateMachine();
    state_machine.iridium_receive_state();
    assert(state_machine.current_state == IRIDIUM_RECEIVE_STATE);
}

void test_iridium_send_receive_state(){
    state_machine = MainStateMachine();
    state_machine.iridium_send_receive_state();
    assert(state_machine.current_state == IRIDIUM_SEND_RECEIVE_STATE);
}
