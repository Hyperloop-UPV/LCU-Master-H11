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
bool transition_idle_to_operational();
bool transition_operational_to_idle();

// Actions
void on_idle_enter();
void on_idle_exit();
void on_operational_enter();
void on_operational_exit();
void cyclic_update_lpus();

inline uint32_t check_slave_fault_id;

inline constexpr auto connecting_state = make_state(
    MasterStates::Connecting,
    Transition<MasterStates>{MasterStates::Idle, transition_connecting_to_idle}
);

inline constexpr auto idle_state = make_state(
    MasterStates::Idle,
    Transition<MasterStates>{MasterStates::Operational, transition_idle_to_operational}
);

inline constexpr auto operational_state = make_state(
    MasterStates::Operational,
    Transition<MasterStates>{MasterStates::Idle, transition_operational_to_idle}
);

inline constinit auto state_machine = []() consteval {
    auto sm = make_state_machine(MasterStates::Connecting, connecting_state, idle_state, operational_state);
    using namespace std::chrono_literals;

    sm.add_enter_action(on_operational_enter, operational_state);
    sm.add_exit_action(on_operational_exit, operational_state);

    sm.add_enter_action(on_idle_enter, idle_state);
    sm.add_exit_action(on_idle_exit, idle_state);

    return sm;
}();

} // namespace LCU_SM

#endif // LCU_STATE_MACHINE_HPP
