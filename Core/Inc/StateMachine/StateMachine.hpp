#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include "C++Utilities/CppImports.hpp"
#include "LCU_MASTER_TYPES.hpp"
#include "Communications/Communications.hpp"

namespace LCU_StateMachine {

    using GeneralStates = DataPackets::general_state_machine;
    using OperationalStates = DataPackets::operational_state_machine;

    static constexpr auto connecting_state = 
        make_state(GeneralStates::Connecting,
        Transition<GeneralStates>{GeneralStates::Operational,[]()
        {
            return Comms::is_connected();
        }}
    );

    static constexpr auto operational_state = 
        make_state(GeneralStates::Operational,
        Transition<GeneralStates>{GeneralStates::Fault,[]()
        {
            return !Comms::is_connected(); // || other things
        }}
    );

    static constexpr auto fault_state = make_state(GeneralStates::Fault);

    static constexpr auto nested_idle_state = make_state(OperationalStates::Idle,
        Transition<OperationalStates>{OperationalStates::Levitating,[]()
        {
            return OrderPackets::levitate_flag;
        }}
    );

    static constexpr auto nested_levitating_state = make_state(OperationalStates::Levitating,
        Transition<OperationalStates>{OperationalStates::Idle,[]()
        {
            return  OrderPackets::stop_levitate_flag 
                    || !LCU_Master::lpu_array->is_all_ok();
        }}
    );

    static inline constinit auto operational_state_machine = []() consteval {
        auto sm = make_state_machine(OperationalStates::Idle,
            nested_idle_state,
            nested_levitating_state
        );
        using namespace std::chrono_literals;

        sm.add_cyclic_action([]()
        {
            LCU_Master::lpu_array->update_all();
        }, 100us , nested_levitating_state);
        
        sm.add_enter_action([]()
        {
            LCU_Master::lpu_array->enable_all();
        }, nested_levitating_state);

        sm.add_exit_action([]()
        {
            LCU_Master::lpu_array->disable_all();
        }, nested_levitating_state);

        return sm;
    }();


    static inline constinit auto general_state_machine = []() consteval {
        auto sm = make_state_machine(GeneralStates::Connecting,
            connecting_state,
            operational_state,
            fault_state
        );
        using namespace std::chrono_literals;

        sm.add_state_machine(operational_state_machine, operational_state);

        sm.add_enter_action([]()
        {
            LCU_Master::led_operational->turn_on();
        }, operational_state);

        sm.add_exit_action([]()
        {
            LCU_Master::led_operational->turn_off();
        }, operational_state);

        sm.add_enter_action([]()
        {
            LCU_Master::led_fault->turn_on();
            ErrorHandler("Entered Fault State");
            while(1);
        }, fault_state);
        
        sm.add_exit_action([]()
        {
            LCU_Master::led_fault->turn_off();
        }, fault_state);

        return sm;
    }();

    void start() {
        general_state_machine.start();
    }

    void update() {
        general_state_machine.check_transitions();
    }
};

#endif // STATE_MACHINE_HPP