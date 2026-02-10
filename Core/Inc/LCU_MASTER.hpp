#ifndef LCU_MASTER_HPP
#define LCU_MASTER_HPP

#include "LCU_MASTER_TYPES.hpp"
#include "ST-LIB.hpp"
#include "Communications/Communications.hpp"

namespace LCU_Master {
    inline void init() {
        Board::init();

        static auto my_spi_inst = Board::instance_of<LCU_Master::spi_req>();
        static auto my_spi = ST_LIB::SPIDomain::SPIWrapper<spi_req>(my_spi_inst);
        Comms::g_spi = &my_spi;

        static auto my_master_ready_inst = Board::instance_of<LCU_Master::master_ready_req>();
        Comms::g_master_ready = &my_master_ready_inst;

        STLIB::start();
        CommsFrame::init(Comms::communications, Comms::communications);
        Comms::start();
    }

    void update() {
        STLIB::update();
        Comms::update();
        // ...
    }
}

#endif // LCU_MASTER_HPP