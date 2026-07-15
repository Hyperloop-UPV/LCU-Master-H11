#ifndef LCU_STATE_MACHINE_HPP
#define LCU_STATE_MACHINE_HPP

#include "C++Utilities/CppImports.hpp"
#include "Communications/Communications.hpp"
#include "Communications/Packets/DataPackets.hpp"
#include "ConfigShared.hpp"

namespace LCU_SM {

using MasterStates = DataPackets::master_state_machine;
inline StateMachineBase slave_state_machine{};

void on_fault_enter();
void start();
void update();

// Transition guards
bool transition_connecting_to_idle();
bool transition_operational_to_idle();
bool transition_idle_to_levitating();
bool transition_idle_to_current_control();
bool transition_idle_to_debug();
bool transition_to_levitating();
bool transition_to_current_control();
bool transition_to_debug();

// Actions
void on_idle_enter();
void on_levitating_enter();
void on_levitating_exit();
void on_current_control_enter();
void on_current_control_exit();
void on_debug_enter();
void on_debug_exit();
void cyclic_update_lpus();
void cyclic_connecting_toggle_led();

inline uint32_t check_slave_fault_id;

inline constexpr auto connecting_state = make_state(
    MasterStates::Connecting,
    Transition<MasterStates>{MasterStates::Idle, transition_connecting_to_idle}
);

inline constexpr auto idle_state = make_state(
    MasterStates::Idle,
    Transition<MasterStates>{MasterStates::Levitating, transition_idle_to_levitating},
    Transition<MasterStates>{MasterStates::Current_Control, transition_idle_to_current_control},
    Transition<MasterStates>{MasterStates::Debug, transition_idle_to_debug}
);

inline constexpr auto levitating_state = make_state(
    MasterStates::Levitating,
    Transition<MasterStates>{MasterStates::Idle, transition_operational_to_idle},
    Transition<MasterStates>{MasterStates::Current_Control, transition_to_current_control},
    Transition<MasterStates>{MasterStates::Debug, transition_to_debug}
);

inline constexpr auto current_control_state = make_state(
    MasterStates::Current_Control,
    Transition<MasterStates>{MasterStates::Idle, transition_operational_to_idle},
    Transition<MasterStates>{MasterStates::Levitating, transition_to_levitating},
    Transition<MasterStates>{MasterStates::Debug, transition_to_debug}
);

inline constexpr auto debug_state = make_state(
    MasterStates::Debug,
    Transition<MasterStates>{MasterStates::Idle, transition_operational_to_idle},
    Transition<MasterStates>{MasterStates::Levitating, transition_to_levitating},
    Transition<MasterStates>{MasterStates::Current_Control, transition_to_current_control}
);

inline constinit auto state_machine = []() consteval {
    auto sm = make_state_machine(MasterStates::Connecting, connecting_state, idle_state,
                                 levitating_state, current_control_state, debug_state);
    using namespace std::chrono_literals;

    sm.add_cyclic_action(cyclic_connecting_toggle_led, 500ms, connecting_state);

    sm.add_enter_action(on_levitating_enter, levitating_state);
    sm.add_exit_action(on_levitating_exit, levitating_state);

    sm.add_enter_action(on_current_control_enter, current_control_state);
    sm.add_exit_action(on_current_control_exit, current_control_state);

    sm.add_enter_action(on_debug_enter, debug_state);
    sm.add_exit_action(on_debug_exit, debug_state);

    sm.add_enter_action(on_idle_enter, idle_state);

    return sm;
}();

} // namespace LCU_SM

#endif // LCU_STATE_MACHINE_HPP
