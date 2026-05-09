#ifndef LCU_STATE_MACHINE_HPP
#define LCU_STATE_MACHINE_HPP

#include "C++Utilities/CppImports.hpp"
#include "LCU_MASTER_TYPES.hpp"
#include "Communications/Communications.hpp"

namespace LCU_StateMachine {

using GeneralStates = DataPackets::general_state_machine;
using OperationalStates = DataPackets::operational_state_machine;

void on_fault_enter() {
    LCU_Master::led_fault->turn_on();
    LCU_Master::led_operational->turn_off();
    LCU_Master::lpu_array->disable_all();
}

static constexpr auto connecting_state = make_state(
    GeneralStates::Connecting,
    Transition<GeneralStates>{GeneralStates::Operational, []() { return Comms::is_connected(); }}
);

static constexpr auto operational_state = make_state(GeneralStates::Operational);

static constexpr auto nested_idle_state = make_state(
    OperationalStates::Idle,
    Transition<OperationalStates>{
        OperationalStates::Levitating,
        []() { return Comms::levitating_state; }
    }
);

static constexpr auto nested_levitating_state = make_state(
    OperationalStates::Levitating,
    Transition<OperationalStates>{
        OperationalStates::Idle,
        []() { return !Comms::levitating_state; }
    }
);

static inline constinit auto operational_state_machine = []() consteval {
    auto sm =
        make_state_machine(OperationalStates::Idle, nested_idle_state, nested_levitating_state);
    using namespace std::chrono_literals;

    sm.add_enter_action([]() { LCU_Master::lpu_array->enable_all(); }, nested_levitating_state);

    sm.add_exit_action([]() { LCU_Master::lpu_array->disable_all(); }, nested_levitating_state);

    return sm;
}();

static inline constinit auto general_state_machine = []() consteval {
    auto nested = StateMachineHelper::add_nested_machines(
        StateMachineHelper::add_nesting(operational_state, operational_state_machine)
    );
    auto sm =
        make_state_machine(GeneralStates::Connecting, nested, connecting_state, operational_state);
    using namespace std::chrono_literals;

    sm.add_enter_action([]() { LCU_Master::led_operational->turn_on(); }, operational_state);

    sm.add_exit_action([]() { LCU_Master::led_operational->turn_off(); }, operational_state);

    sm.add_cyclic_action([]() { LCU_Master::lpu_array->update_all(); }, 1ms, operational_state);

    return sm;
}();

void start() {}

void update() {
    general_state_machine.check_transitions();
    LCU_Master::general_state_machine_state = general_state_machine.get_current_state();
    LCU_Master::operational_state_machine_state = operational_state_machine.get_current_state();

    if (general_state_machine.get_current_state() != GeneralStates::Connecting) {
        if (!Comms::is_connected()) {
            FAULT("SPI / Ethernet Disconnected");
        }
    }
    if (LCU_Master::slave_fault_triggered) {
        FAULT("Slave Fault Triggered");
    }
    if (!LCU_Master::lpu_array->is_all_ok()) {
        FAULT("LPU Array Fault Detected");
    }
}
}; // namespace LCU_StateMachine

#endif // LCU_STATE_MACHINE_HPP
