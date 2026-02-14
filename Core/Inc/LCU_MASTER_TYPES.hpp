#ifndef LCU_MASTER_TYPES_HPP
#define LCU_MASTER_TYPES_HPP

#include "ST-LIB.hpp"
#include "Pinout/Pinout.hpp"
#include "SpiShared.hpp"
#include "ConfigShared.hpp"
#include "LPU/LPU.hpp"
#include "AirgapShared.hpp"
#include "Communications/Packets/DataPackets.hpp"
#include "Communications/Packets/OrderPackets.hpp"

using namespace ST_LIB;


namespace LCU_Master {

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
    #endif

    inline constexpr auto led_operational_req = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::led_operational);
    inline constexpr auto led_fault_req = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::led_fault);
    
    inline constexpr auto spi_req = 
        ST_LIB::SPIDomain::Device<DMA_Domain::Stream::dma1_stream0, DMA_Domain::Stream::dma1_stream1>(
            ST_LIB::SPIDomain::SPIMode::MASTER, ST_LIB::SPIDomain::SPIPeripheral::spi3,
            10000, Pinout::spi_sck, Pinout::spi_miso, Pinout::spi_mosi, spi_conf
            // 10khz for now, should test later higher speeds
        );
    inline constexpr auto slave_ready_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::spi_nss);


    inline constexpr auto fault1_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::fault1);
    // inline constexpr auto fault2_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::fault2);
    // inline constexpr auto fault3_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::fault3);
    // inline constexpr auto fault4_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::fault4);
    // inline constexpr auto fault5_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::fault5);
    // inline constexpr auto fault6_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::fault6);
    // inline constexpr auto fault7_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::fault7);
    // inline constexpr auto fault8_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::fault8);
    // inline constexpr auto fault9_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::fault9);
    // inline constexpr auto fault10_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::fault10);

    inline constexpr auto ready1_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::ready1);
    // inline constexpr auto ready2_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::ready2);
    // inline constexpr auto ready3_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::ready3);
    // inline constexpr auto ready4_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::ready4);
    // inline constexpr auto ready5_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::ready5);
    // inline constexpr auto ready6_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::ready6);
    // inline constexpr auto ready7_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::ready7);
    // inline constexpr auto ready8_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::ready8);
    // inline constexpr auto ready9_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::ready9);
    // inline constexpr auto ready10_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::ready10);
    
    inline constexpr auto rst1_req = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::rst1);
    // inline constexpr auto rst2_req = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::rst2);
    // inline constexpr auto rst3_req = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::rst3);
    // inline constexpr auto rst4_req = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::rst4);
    // inline constexpr auto rst5_req = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::rst5);

    using Board = ST_LIB::Board<
        #ifdef STLIB_ETH
        eth,
        #endif
        led_operational_req, led_fault_req,
        spi_req, slave_ready_req,
        fault1_req, /*fault2_req, fault3_req, fault4_req, fault5_req,
        fault6_req, fault7_req, fault8_req, fault9_req, fault10_req,*/
        ready1_req, /*ready2_req, ready3_req, ready4_req, ready5_req,
        ready6_req, ready7_req, ready8_req, ready9_req, ready10_req,*/
        rst1_req/*, rst2_req, rst3_req, rst4_req, rst5_req,*/ 
        >;

    using CommsFrame = SystemFrame<true>;

    inline ST_LIB::DigitalOutputDomain::Instance* led_operational = nullptr;
    inline ST_LIB::DigitalOutputDomain::Instance* led_fault = nullptr;

    inline LPU *lpu1 = nullptr;
    inline LpuArray<std::tuple<LPU>, std::tuple<ST_LIB::DigitalOutputDomain::Instance>>* lpu_array;

    inline AirgapBase airgap1;

    inline float &lcu_vbat_1 = lpu1->vbat_v;
    inline float &lcu_coil_current_1 = lpu1->shunt_v;
    inline float &lcu_airgap_1 = airgap1.airgap_v;

    inline DataPackets::general_state_machine general_state_machine_state = DataPackets::general_state_machine::Connecting;
    inline DataPackets::operational_state_machine operational_state_machine_state = DataPackets::operational_state_machine::Idle;
}

#endif // LCU_MASTER_TYPES_HPP