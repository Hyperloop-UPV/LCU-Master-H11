#include "ST-LIB.hpp"
#include "main.h"
#include "Pinout/Pinout.hpp"
#include "Communications/Communications.hpp"

using namespace ST_LIB;

constexpr auto rst = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::rst1);

#ifdef STLIB_ETH
#if defined(USE_PHY_LAN8742)
constexpr auto eth =
    EthernetDomain::Ethernet(EthernetDomain::PINSET_H10, "00:80:e1:00:01:07",
                             "192.168.1.4", "255.255.0.0");
#elif defined(USE_PHY_LAN8700)
constexpr auto eth =
    EthernetDomain::Ethernet(EthernetDomain::PINSET_H10, "00:80:e1:00:01:07",
                             "192.168.1.4", "255.255.0.0");
#elif defined(USE_PHY_KSZ8041)
constexpr auto eth =
    EthernetDomain::Ethernet(EthernetDomain::PINSET_H11, "00:80:e1:00:01:07",
                             "192.168.1.4", "255.255.0.0");
#else
#error "No PHY selected for Ethernet pinset selection"
#endif

using myBoard = ST_LIB::Board<eth, rst>;
#else
using myBoard = ST_LIB::Board<rst>;
#endif


float lcu_vbat_1, lcu_vbat_2, lcu_vbat_3, lcu_vbat_4, lcu_vbat_5, lcu_vbat_6, lcu_vbat_7, lcu_vbat_8, lcu_vbat_9, lcu_vbat_10;
float lcu_coil_current_1, lcu_coil_current_2, lcu_coil_current_3, lcu_coil_current_4, lcu_coil_current_5, lcu_coil_current_6, lcu_coil_current_7, lcu_coil_current_8, lcu_coil_current_9, lcu_coil_current_10;
float lcu_airgap_1, lcu_airgap_2, lcu_airgap_3, lcu_airgap_4, lcu_airgap_5, lcu_airgap_6, lcu_airgap_7, lcu_airgap_8;


int main(void) {
  Hard_fault_check();

  myBoard::init();
#ifdef STLIB_ETH
  auto eth_instance = &myBoard::instance_of<eth>();
#endif
  auto rst_instance = &myBoard::instance_of<rst>();

  rst_instance->turn_on();

  DataPackets::lpu_currents_init(lcu_vbat_1, lcu_vbat_2, lcu_vbat_3, lcu_vbat_4, lcu_vbat_5, lcu_vbat_6, lcu_vbat_7, lcu_vbat_8, lcu_vbat_9, lcu_vbat_10, lcu_coil_current_1, lcu_coil_current_2, lcu_coil_current_3, lcu_coil_current_4, lcu_coil_current_5, lcu_coil_current_6, lcu_coil_current_7, lcu_coil_current_8, lcu_coil_current_9, lcu_coil_current_10);
  DataPackets::airgap_measurements_init(lcu_airgap_1, lcu_airgap_2, lcu_airgap_3, lcu_airgap_4, lcu_airgap_5, lcu_airgap_6, lcu_airgap_7, lcu_airgap_8);

  Comms::start();

  while (1) {
#ifdef STLIB_ETH
    eth_instance->update();
#endif
  }
}
void Error_Handler(void) {
  ErrorHandler("HAL error handler triggered");
  while (1) {
  }
}
