#ifndef LCU_MASTER_HPP
#define LCU_MASTER_HPP

#include "LCU_MASTER_TYPES.hpp"
#include "ST-LIB.hpp"
#include "Communications/Communications.hpp"

namespace LCU_Master {
    inline void init() {
        Board::init();

        /* Comms */
        #ifdef STLIB_ETH
        static auto eth_instance = Board::instance_of<eth>();
        Comms::g_eth = &eth_instance;
        #endif

        static auto my_spi_inst = Board::instance_of<LCU_Master::spi_req>();
        static auto my_spi = ST_LIB::SPIDomain::SPIWrapper<spi_req>(my_spi_inst);
        Comms::g_spi = &my_spi;

        static auto my_master_ready_inst = Board::instance_of<LCU_Master::master_ready_req>();
        Comms::g_master_ready = &my_master_ready_inst;

        /* LPU */
        auto& ready_pin1 = Board::instance_of<ready1_req>();
        auto& fault_pin1 = Board::instance_of<fault1_req>();
        static LPU lpu1_inst(ready_pin1, fault_pin1);
        LCU_Master::lpu1 = &lpu1_inst;

        static auto& rst_pin1 = Board::instance_of<rst1_req>();
        static LpuArray lpu_array1_inst(std::make_tuple(std::ref(lpu1_inst)), std::make_tuple(std::ref(rst_pin1)));
        LCU_Master::lpu_array1 = &lpu_array1_inst;


        /* Tie comms variables to packets */
        // TODO

        STLIB::start();
        CommsFrame::init(Comms::communications, lpu1_inst,
                         Comms::communications, lpu1_inst, airgap1);
        Comms::start();

    }

    void update() {
        STLIB::update();
        Comms::update();
        // ...
    }
}

#endif // LCU_MASTER_HPP