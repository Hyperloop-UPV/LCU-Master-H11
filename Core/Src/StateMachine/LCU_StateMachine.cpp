#include "StateMachine/LCU_StateMachine.hpp"
#include "LCU_MASTER.hpp"

namespace LCU_SM {

// Transition guards
bool transition_connecting_to_idle() {
    return Communications::is_connected();
}

bool transition_operational_to_idle() {
    return !Communications::operational_state;
}

bool transition_idle_to_levitating() {
    return Communications::operational_state &&
           slave_state_machine.desired_state == SlaveState::LEVITATION;
}

bool transition_idle_to_current_control() {
    return Communications::operational_state &&
           slave_state_machine.desired_state == SlaveState::CURRENT_CONTROL;
}

bool transition_idle_to_debug() {
    return Communications::operational_state &&
           slave_state_machine.desired_state == SlaveState::DEBUG;
}

bool transition_to_levitating() {
    return slave_state_machine.desired_state == SlaveState::LEVITATION;
}

bool transition_to_current_control() {
    return slave_state_machine.desired_state == SlaveState::CURRENT_CONTROL;
}

bool transition_to_debug() {
    return slave_state_machine.desired_state == SlaveState::DEBUG;
}

// Actions
void on_idle_enter() {
    LCU_Master::led_connected.turn_on();
    LCU_Master::lpu_array.disable_all();
    slave_state_machine.desired_state = SlaveState::IDLE;
}

void on_levitating_enter() {
    LCU_Master::led_levitation.turn_on();
    LCU_Master::lpu_array.enable_all();
}

void on_levitating_exit() {
    LCU_Master::led_levitation.turn_off();
}

void on_current_control_enter() {
    LCU_Master::led_current_control.turn_on();
    LCU_Master::lpu_array.enable_all();
}

void on_current_control_exit() {
    LCU_Master::led_current_control.turn_off();
}

void on_debug_enter() {
    LCU_Master::led_debug.turn_on();
    LCU_Master::lpu_array.enable_all();
}

void on_debug_exit() {
    LCU_Master::led_debug.turn_off();
}

void on_fault_enter() {
    LCU_Master::lpu_array.disable_all();
    LCU_Master::led_fault.turn_on();
    LCU_Master::led_levitation.turn_off();
    LCU_Master::led_current_control.turn_off();
    LCU_Master::led_debug.turn_off();
    Scheduler::unregister_task(check_slave_fault_id);
    slave_state_machine.current_state = SlaveState::FAULT;
}

// Cyclic actions
void cyclic_connecting_toggle_led() {
    LCU_Master::led_connected.toggle();
}

void cyclic_update_lpus() {
    LCU_Master::lpu_array.update_all();
}

void check_slave_fault() {
    auto curr_state = state_machine.get_current_state();
    if (LCU_Master::slave_fault.read() == GPIO_PinState::GPIO_PIN_RESET) {
        FAULT("Slave Fault Detected via GPIO");
    }
    if (curr_state != MasterStates::Connecting) {
        if (!Communications::is_connected()) {
            FAULT("Disconnexion Detected");
        }
    }
}

void start() {
    check_slave_fault_id = Scheduler::register_task(1000, check_slave_fault);
    Scheduler::register_task(1000, cyclic_update_lpus);
}

void update() {
    if (FaultController::is_faulted()) {
        slave_state_machine.desired_state = SlaveState::FAULT;
        Communications::master_state_machine_state = MasterStates::Fault;
    } else {
        Communications::master_state_machine_state = state_machine.get_current_state();
    }
}

} // namespace LCU_SM
