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
inline auto& led_operational = ST_LIB::PG8;
inline auto& led_fault = ST_LIB::PG7;

/* Fault Lines */
inline auto& master_fault = ST_LIB::PE0;
inline auto& slave_fault = ST_LIB::PD2;

// ============================================
// LPU
// ============================================

/* Fault (Digital Input) */
inline auto& fault1 = ST_LIB::PE9;
inline auto& fault2 = ST_LIB::PE7;
inline auto& fault3 = ST_LIB::PF2;
inline auto& fault4 = ST_LIB::PE6;
inline auto& fault5 = ST_LIB::PE3;
inline auto& fault6 = ST_LIB::PC8;
inline auto& fault7 = ST_LIB::PD15;
inline auto& fault8 = ST_LIB::PB10;
inline auto& fault9 = ST_LIB::PE14;
inline auto& fault10 = ST_LIB::PE11;

/* Ready (Digital Input) */
inline auto& ready1 = ST_LIB::PE8;
inline auto& ready2 = ST_LIB::PG1;
inline auto& ready3 = ST_LIB::PF1;
inline auto& ready4 = ST_LIB::PE5;
inline auto& ready5 = ST_LIB::PE4;
inline auto& ready6 = ST_LIB::PC7;
inline auto& ready7 = ST_LIB::PD14;
inline auto& ready8 = ST_LIB::PB11;
inline auto& ready9 = ST_LIB::PE13;
inline auto& ready10 = ST_LIB::PE10;

/* Reset (Digital Output) */
inline auto& rst1 = ST_LIB::PD9;
inline auto& rst2 = ST_LIB::PD10;
inline auto& rst3 = ST_LIB::PD8;
inline auto& rst4 = ST_LIB::PB15;
inline auto& rst5 = ST_LIB::PB14;

// ============================================
// SPI
// ============================================

inline auto constexpr spi_peripheral = ST_LIB::SPIDomain::SPIPeripheral::spi3;
inline auto& spi_sck = ST_LIB::PC10;
inline auto& spi_miso = ST_LIB::PC11;
inline auto& spi_mosi = ST_LIB::PC12;
inline auto& spi_nss = ST_LIB::PD3; // Used as GPIO (slave_ready pin) with software NSS management

}; // namespace Pinout

#endif // PINOUT_HPP
