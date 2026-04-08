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
    master_fault->turn_on();

    LCU_Master::slave_fault = &Board::instance_of<slave_fault_req>();

/* Comms */
#ifdef STLIB_ETH
    static auto& eth_instance = Board::instance_of<eth>();
    Comms::g_eth = &eth_instance;
#endif

    static auto my_spi =
        ST_LIB::SPIDomain::SPIWrapper<spi_req>(Board::instance_of<LCU_Master::spi_req>());
    Comms::g_spi = &my_spi;

    Comms::g_slave_ready = &Board::instance_of<LCU_Master::slave_ready_req>();

#ifdef USE_1_DOF
    /* LPU */
    static LPU lpu_inst(Board::instance_of<ready_req>(), Board::instance_of<fault_req>());

    static LpuArrayType lpu_array_inst(
        std::tie(lpu_inst),
        std::tie(Board::instance_of<rst_req>())
    );
    LCU_Master::lpu_array = &lpu_array_inst;

    /* Airgap */
    static Airgap airgap_inst;

    static AirgapArrayType airgap_array_inst(
        std::tie(airgap_inst)
    );
    LCU_Master::airgap_array = &airgap_array_inst;

#elif defined(USE_5_DOF)
    /* LPUs */
    static LPU lpu1(Board::instance_of<ready1_req>(), Board::instance_of<fault1_req>());
    static LPU lpu2(Board::instance_of<ready2_req>(), Board::instance_of<fault2_req>());
    static LPU lpu3(Board::instance_of<ready3_req>(), Board::instance_of<fault3_req>());
    static LPU lpu4(Board::instance_of<ready4_req>(), Board::instance_of<fault4_req>());
    static LPU lpu5(Board::instance_of<ready5_req>(), Board::instance_of<fault5_req>());
    static LPU lpu6(Board::instance_of<ready6_req>(), Board::instance_of<fault6_req>());
    static LPU lpu7(Board::instance_of<ready7_req>(), Board::instance_of<fault7_req>());
    static LPU lpu8(Board::instance_of<ready8_req>(), Board::instance_of<fault8_req>());
    static LPU lpu9(Board::instance_of<ready9_req>(), Board::instance_of<fault9_req>());
    static LPU lpu10(Board::instance_of<ready10_req>(), Board::instance_of<fault10_req>());

    static LpuArrayType lpu_array_inst(
        std::tie(lpu1, lpu2, lpu3, lpu4, lpu5, lpu6, lpu7, lpu8, lpu9, lpu10),
        std::tie(Board::instance_of<rst1_req>(), Board::instance_of<rst2_req>(), Board::instance_of<rst3_req>(),
                 Board::instance_of<rst4_req>(), Board::instance_of<rst5_req>())
    );
    LCU_Master::lpu_array = &lpu_array_inst;

    /* Airgaps */
    static Airgap airgap1;
    static Airgap airgap2;
    static Airgap airgap3;
    static Airgap airgap4;
    static Airgap airgap5;
    static Airgap airgap6;
    static Airgap airgap7;
    static Airgap airgap8;

    static AirgapArrayType airgap_array_inst(
        std::tie(airgap1, airgap2, airgap3, airgap4, airgap5, airgap6, airgap7, airgap8)
    );
    LCU_Master::airgap_array = &airgap_array_inst;
#endif

    MDMA::start();

    LCU_StateMachine::start();

#ifdef USE_1_DOF
    CommsFrame::init(Comms::control, lpu_inst, Comms::slave_state, lpu_inst, airgap_inst);
#elif defined(USE_5_DOF)
    CommsFrame::init(Comms::control, lpu1, lpu2, lpu3, lpu4, lpu5, lpu6, lpu7, lpu8, lpu9, lpu10,
                      Comms::slave_state, lpu1, lpu2, lpu3, lpu4, lpu5, lpu6, lpu7, lpu8, lpu9, lpu10,
                      airgap1, airgap2, airgap3, airgap4, airgap5, airgap6, airgap7, airgap8);
#endif

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
