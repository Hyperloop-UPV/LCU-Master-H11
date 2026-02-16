#ifndef LCU_MASTER_HPP
#define LCU_MASTER_HPP

#include "LCU_MASTER_TYPES.hpp"
#include "ST-LIB.hpp"
#include "Communications/Communications.hpp"
#include "StateMachine/LCU_StateMachine.hpp"

namespace LCU_Master {
inline void init() {
    Board::init();

    static auto led_operational_inst = Board::instance_of<led_operational_req>();
    LCU_Master::led_operational = &led_operational_inst;
    static auto led_fault_inst = Board::instance_of<led_fault_req>();
    LCU_Master::led_fault = &led_fault_inst;

    static auto master_fault_inst = Board::instance_of<master_fault_req>();
    LCU_Master::master_fault = &master_fault_inst;

/* Comms */
#ifdef STLIB_ETH
    static auto eth_instance = Board::instance_of<eth>();
    Comms::g_eth = &eth_instance;
#endif

    static auto my_spi_inst = &Board::instance_of<LCU_Master::spi_req>();
    static auto my_spi = ST_LIB::SPIDomain::SPIWrapper<spi_req>(*my_spi_inst);
    Comms::g_spi = &my_spi;

    static auto my_slave_ready_inst = Board::instance_of<LCU_Master::slave_ready_req>();
    Comms::g_slave_ready = &my_slave_ready_inst;

    /* LPU */
    auto& ready_pin1 = Board::instance_of<ready1_req>();
    auto& fault_pin1 = Board::instance_of<fault1_req>();
    static LPU lpu1_inst(ready_pin1, fault_pin1);
    LCU_Master::lpu1 = &lpu1_inst;

    static auto& rst_pin1 = Board::instance_of<rst1_req>();
    static LpuArray lpu_array_inst(
        std::make_tuple(std::ref(lpu1_inst)),
        std::make_tuple(std::ref(rst_pin1))
    );
    LCU_Master::lpu_array = &lpu_array_inst;

    /* Tie comms variables to packets */
    // TODO

    // STLIB::start();
    CommsFrame::init(Comms::communications, lpu1_inst, Comms::communications, lpu1_inst, airgap1);
    Comms::start();
    Scheduler::start();
    LCU_StateMachine::start();
    MDMA::start();
}

void update() {
    general_state_machine_state = LCU_StateMachine::general_state_machine.get_current_state();
    operational_state_machine_state =
        LCU_StateMachine::operational_state_machine.get_current_state();
    if (general_state_machine_state == LCU_StateMachine::GeneralStates::Connecting) {
        LCU_StateMachine::update();
        return;
    }

    Comms::update();
    LCU_StateMachine::update();
    MDMA::update();
    Scheduler::update();
    // do things with flags
    Comms::clear_flags();
    // ...
}
} // namespace LCU_Master

#endif // LCU_MASTER_HPP
