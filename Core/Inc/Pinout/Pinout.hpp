#ifndef PINOUT_HPP
#define PINOUT_HPP

#include "HALAL/Models/Pin.hpp"          // New pins
#include "HALAL/Models/PinModel/Pin.hpp" // Old pins
#include "Common/Flags.hpp"

namespace Pinout {
// ============================================
// General bits and bobs
// ============================================

/* LED (Digital Output) */
auto& led_operational = ST_LIB::PG8;
auto& led_fault = ST_LIB::PG7;

/* Fault Lines (EXTI) */
// TODO

// ============================================
// LPU
// ============================================

/* Fault (Digital Input) */
auto& fault1 = ST_LIB::PE9;
auto& fault2 = ST_LIB::PE7;
auto& fault3 = ST_LIB::PF2;
auto& fault4 = ST_LIB::PE6;
auto& fault5 = ST_LIB::PE3;
auto& fault6 = ST_LIB::PC8;
auto& fault7 = ST_LIB::PD15;
auto& fault8 = ST_LIB::PB10;
auto& fault9 = ST_LIB::PE14;
auto& fault10 = ST_LIB::PE11;

/* Ready (Digital Input) */
auto& ready1 = ST_LIB::PE8;
auto& ready2 = ST_LIB::PG1;
auto& ready3 = ST_LIB::PF1;
auto& ready4 = ST_LIB::PE5;
auto& ready5 = ST_LIB::PE4;
auto& ready6 = ST_LIB::PC7;
auto& ready7 = ST_LIB::PD14;
auto& ready8 = ST_LIB::PB11;
auto& ready9 = ST_LIB::PE13;
auto& ready10 = ST_LIB::PE10;

/* Reset (Digital Output) */
auto& rst1 = ST_LIB::PD9;
auto& rst2 = ST_LIB::PD10;
auto& rst3 = ST_LIB::PD8;
auto& rst4 = ST_LIB::PB15;
auto& rst5 = ST_LIB::PB14;

// ============================================
// SPI
// ============================================

auto constexpr spi_peripheral = ST_LIB::SPIDomain::SPIPeripheral::spi3;
auto& spi_sck = ST_LIB::PC10;
auto& spi_miso = ST_LIB::PC11;
auto& spi_mosi = ST_LIB::PC12;
auto& spi_nss =
    ST_LIB::PD3; // Will use as GPIO (master_ready pin) with software NSS management (always active)
};               // namespace Pinout

#endif // PINOUT_HPP
