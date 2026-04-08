#ifndef LCU_STATE_MACHINE_HPP
#define LCU_STATE_MACHINE_HPP

#include "C++Utilities/CppImports.hpp"
#include "LCU_MASTER_TYPES.hpp"
#include "Communications/Communications.hpp"

namespace LCU_StateMachine {

using GeneralStates = DataPackets::general_state_machine;
using OperationalStates = DataPackets::operational_state_machine;

// Helper: Check if buffers should be active based on control state
static inline bool should_buffers_be_active() {
    // Buffers active if any control mode is active:
    // - Levitating (distance control)
    // - Current control mode
    // - Fixed PWM on any LPU
    // (Fixed VBAT is just a sensor override, not a control mode)
    return Comms::levitating_state || 
           Comms::current_control_active || 
           Comms::fixed_pwm_active;
}

static constexpr auto connecting_state = make_state(
    GeneralStates::Connecting,
    Transition<GeneralStates>{GeneralStates::Operational, []() { return Comms::is_connected(); }},
    Transition<GeneralStates>{
        GeneralStates::Fault,
        []() { return LCU_Master::slave_fault_triggered; }
    }
);

static constexpr auto operational_state = make_state(
    GeneralStates::Operational,
    Transition<GeneralStates>{
        GeneralStates::Fault,
        []() {
            return !Comms::is_connected() || LCU_Master::slave_fault_triggered ||
                   !LCU_Master::lpu_array->is_all_ok(); // || other things
        }
    }
);

static constexpr auto fault_state = make_state(GeneralStates::Fault);

static constexpr auto nested_idle_state = make_state(
    OperationalStates::Idle,
    Transition<OperationalStates>{
        OperationalStates::Levitating,
        []() { return Comms::levitating_state; }
    },
    Transition<OperationalStates>{
        OperationalStates::Current_Control,
        []() { return Comms::current_control_active; }
    },
    Transition<OperationalStates>{
        OperationalStates::Debug,
        []() { return Comms::fixed_pwm_active != 0; }
    }
);

static constexpr auto nested_levitating_state = make_state(
    OperationalStates::Levitating,
    Transition<OperationalStates>{
        OperationalStates::Idle,
        []() { return !Comms::levitating_state || !LCU_Master::lpu_array->is_all_ok(); }
    },
    Transition<OperationalStates>{
        OperationalStates::Current_Control,
        []() { return Comms::current_control_active && !Comms::levitating_state; }
    },
    Transition<OperationalStates>{
        OperationalStates::Debug,
        []() { return Comms::fixed_pwm_active != 0 && !Comms::levitating_state; }
    }
);

static constexpr auto nested_current_control_state = make_state(
    OperationalStates::Current_Control,
    Transition<OperationalStates>{
        OperationalStates::Idle,
        []() { return !Comms::current_control_active || !LCU_Master::lpu_array->is_all_ok(); }
    },
    Transition<OperationalStates>{
        OperationalStates::Levitating,
        []() { return Comms::levitating_state && !Comms::current_control_active; }
    },
    Transition<OperationalStates>{
        OperationalStates::Debug,
        []() { return Comms::fixed_pwm_active != 0 && !Comms::current_control_active; }
    }
);

static constexpr auto nested_debug_state = make_state(
    OperationalStates::Debug,
    Transition<OperationalStates>{
        OperationalStates::Idle,
        []() { return Comms::fixed_pwm_active == 0 || !LCU_Master::lpu_array->is_all_ok(); }
    },
    Transition<OperationalStates>{
        OperationalStates::Levitating,
        []() { return Comms::levitating_state && !Comms::fixed_pwm_active; }
    },
    Transition<OperationalStates>{
        OperationalStates::Current_Control,
        []() { return Comms::current_control_active && !Comms::fixed_pwm_active; }
    }
);

static inline constinit auto operational_state_machine = []() consteval {
    auto sm = make_state_machine(
        OperationalStates::Idle, 
        nested_idle_state, 
        nested_levitating_state,
        nested_current_control_state,
        nested_debug_state
    );
    using namespace std::chrono_literals;

    sm.add_enter_action([]() { LCU_Master::lpu_array->enable_all(); }, nested_levitating_state);
    sm.add_exit_action([]() { LCU_Master::lpu_array->disable_all(); }, nested_levitating_state);

    sm.add_enter_action([]() { LCU_Master::lpu_array->enable_all(); }, nested_current_control_state);
    sm.add_exit_action([]() { LCU_Master::lpu_array->disable_all(); }, nested_current_control_state);

    sm.add_enter_action([]() { LCU_Master::lpu_array->enable_all(); }, nested_debug_state);
    sm.add_exit_action([]() { LCU_Master::lpu_array->disable_all(); }, nested_debug_state);

    return sm;
}();

static inline constinit auto general_state_machine = []() consteval {
    auto nested = StateMachineHelper::add_nested_machines(
        StateMachineHelper::add_nesting(operational_state, operational_state_machine)
    );
    auto sm = make_state_machine(
        GeneralStates::Connecting,
        nested,
        connecting_state,
        operational_state,
        fault_state
    );
    using namespace std::chrono_literals;

    sm.add_enter_action([]() { LCU_Master::led_operational->turn_on(); }, operational_state);

    sm.add_exit_action([]() { LCU_Master::led_operational->turn_off(); }, operational_state);

    sm.add_enter_action(
        []() {
            LCU_Master::led_fault->turn_on();
            LCU_Master::master_fault->turn_off();
            ErrorHandler("Entered Fault State");
            // while (1);
        },
        fault_state
    );

    sm.add_exit_action([]() { LCU_Master::led_fault->turn_off(); }, fault_state);

    sm.add_cyclic_action([]() { LCU_Master::lpu_array->update_all(); }, 100us, operational_state);

    return sm;
}();

void start() {
    ProtectionManager::add_standard_protections();
    ProtectionManager::link_state_machine(general_state_machine, static_cast<size_t>(GeneralStates::Fault));
    ProtectionManager::initialize();
    general_state_machine.start();
}

void update() {
    general_state_machine.check_transitions();
    LCU_Master::general_state_machine_state = general_state_machine.get_current_state();
    LCU_Master::operational_state_machine_state = operational_state_machine.get_current_state();
}
}; // namespace LCU_StateMachine

#endif // LCU_STATE_MACHINE_HPP
