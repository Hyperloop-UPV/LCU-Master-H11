#ifndef PINOUT_HPP
#define PINOUT_HPP

#include "HALAL/Models/Pin.hpp" // New pins
#include "HALAL/Models/PinModel/Pin.hpp" // Old pins
#include "Common/Flags.hpp"

namespace Pinout {
    // ============================================
    // General bits and bobs
    // ============================================

    /* LED (Digital Output) */
    auto &LED_OPERATIONAL = ST_LIB::PG8;
    auto &LED_FAULT = ST_LIB::PG7;

    /* Don't know? */
    auto &RST1 = ST_LIB::PD9;
    auto &RST2 = ST_LIB::PD10;
    auto &RST3 = ST_LIB::PD8;
    auto &RST4 = ST_LIB::PB15;
    auto &RST5 = ST_LIB::PB14;


    // ============================================
    // LPU
    // ============================================

    /* Fault (Digital Input) */
    auto &fautlt1 = ST_LIB::PE9;
    auto &fault2 = ST_LIB::PE7;
    auto &fault3 = ST_LIB::PF2;
    auto &fault4 = ST_LIB::PE6;
    auto &fault5 = ST_LIB::PE3;
    auto &fault6 = ST_LIB::PC8;
    auto &fault7 = ST_LIB::PD15;
    auto &fault8 = ST_LIB::PB10;
    auto &fault9 = ST_LIB::PE14;
    auto &fault10 = ST_LIB::PE11;

    /* Ready (Digital Input) */
    auto &ready1 = ST_LIB::PE8;
    auto &ready2 = ST_LIB::PG1;
    auto &ready3 = ST_LIB::PF1;
    auto &ready4 = ST_LIB::PE5;
    auto &ready5 = ST_LIB::PE4;
    auto &ready6 = ST_LIB::PC7;
    auto &ready7 = ST_LIB::PD14;
    auto &ready8 = ST_LIB::PB11;
    auto &ready9 = ST_LIB::PE13;
    auto &ready10 = ST_LIB::PE10;
};



// static IPV4 LCU_IP(LCU_IP_STR);
// static IPV4 HVSCU_IP(HVSCU_IP_STR);


static const uint32_t TCP_SERVER_PORT = 50500;
static const uint32_t TCP_CLIENT_PORT = 50401;

static const uint32_t UDP_PORT = 50400;

namespace OUTPUTS
{

}


#endif // PINOUT_HPP