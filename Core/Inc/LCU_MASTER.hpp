#ifndef LCU_MASTER_HPP
#define LCU_MASTER_HPP

#include "LCU_MASTER_TYPES.hpp"
#include "ST-LIB.hpp"
#include "Communications/Communications.hpp"
#include "StateMachine/LCU_StateMachine.hpp"

namespace LCU_Master {
inline void init() {
    Board::init();

    LCU_Master::led_operational = &Board::instance_of<led_operational_req>();
    LCU_Master::led_fault = &Board::instance_of<led_fault_req>();

    LCU_Master::master_fault = &Board::instance_of<master_fault_req>();

/* Comms */
#ifdef STLIB_ETH
    static auto& eth_instance = Board::instance_of<eth>();
    Comms::g_eth = &eth_instance;
#endif

    static auto my_spi = ST_LIB::SPIDomain::SPIWrapper<spi_req>(Board::instance_of<LCU_Master::spi_req>());
    Comms::g_spi = &my_spi;

    Comms::g_slave_ready = &Board::instance_of<LCU_Master::slave_ready_req>();

    /* LPU */
    static LPU lpu1_inst(Board::instance_of<ready1_req>(), Board::instance_of<fault1_req>());
    LCU_Master::lpu1 = &lpu1_inst;

    static LpuArray lpu_array_inst(
        std::make_tuple(std::ref(lpu1_inst)),
        std::make_tuple(std::ref(Board::instance_of<rst1_req>()))
    );
    LCU_Master::lpu_array = &lpu_array_inst;

    Scheduler::start();
    MDMA::start();

    LCU_StateMachine::start();

    CommsFrame::init(Comms::communications, lpu1_inst, Comms::communications, lpu1_inst, airgap1);
    Comms::start();
}

void update() {
    Comms::update();
    LCU_StateMachine::update();
    Scheduler::update();
    MDMA::update();
}
} // namespace LCU_Master

#endif // LCU_MASTER_HPP
