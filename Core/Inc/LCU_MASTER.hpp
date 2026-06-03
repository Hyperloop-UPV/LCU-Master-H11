#ifndef LCU_MASTER_HPP
#define LCU_MASTER_HPP

#include "ST-LIB.hpp"
#include "LPU/LPU.hpp"
#include "Airgap/Airgap.hpp"
#include "Pinout/Pinout.hpp"
#include "Config/LCUHardwareConfig.hpp"
#include "ConfigShared.hpp"
#include "SpiShared.hpp"
#include "FlagsShared.hpp"
#include "StateMachine/LCU_StateMachine.hpp"
#include "Communications/Communications.hpp"

namespace LCU_Master {

void init();
void update();

// ============================================
// LED and Fault pins
// ============================================

inline constexpr auto led_connected_req = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::led_connected);
inline constexpr auto led_fault_req = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::led_fault);
inline constexpr auto led_debug_req = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::led_debug);
inline constexpr auto led_current_control_req = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::led_current_control);
inline constexpr auto led_levitation_req = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::led_levitation);

inline bool slave_fault_triggered = false;

inline constexpr auto master_fault_req =
    ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::master_fault);
inline constexpr auto slave_fault_req = ST_LIB::EXTIDomain::Device(
    Pinout::slave_fault,
    ST_LIB::EXTIDomain::Trigger::FALLING_EDGE,
    []() {
        if (Communications::is_resetting_slave) {
            // Ignore faults triggered during slave reset, as they are expected
            return;
        }
        slave_fault_triggered = true;
        if (!FaultController::is_faulted()) FAULT("Slave fault detected via EXTI");
    }
);

// ============================================
// SPI
// ============================================

inline constexpr auto spi_req =
    ST_LIB::SPIDomain::Device<ST_LIB::DMADomain::Stream::dma1_stream0, ST_LIB::DMADomain::Stream::dma1_stream1>(
        ST_LIB::SPIDomain::SPIMode::MASTER,
        Pinout::spi_peripheral,
        20'000'000, 
        Pinout::spi_sck,
        Pinout::spi_miso,
        Pinout::spi_mosi,
        spi_conf
    );
inline bool slave_ready_triggered = false;
inline constexpr auto slave_ready_req =
    ST_LIB::EXTIDomain::Device(Pinout::spi_nss, ST_LIB::EXTIDomain::Trigger::RISING_EDGE, []() {
        slave_ready_triggered = true;
    });

// ============================================
// LPU control pins (always defined)
// ============================================

// Fault (Digital Input)
inline constexpr auto fault1_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::fault1);
inline constexpr auto fault2_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::fault2);
inline constexpr auto fault3_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::fault3);
inline constexpr auto fault4_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::fault4);
inline constexpr auto fault5_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::fault5);
inline constexpr auto fault6_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::fault6);
inline constexpr auto fault7_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::fault7);
inline constexpr auto fault8_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::fault8);
inline constexpr auto fault9_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::fault9);
inline constexpr auto fault10_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::fault10);

// Ready (Digital Input)
inline constexpr auto ready1_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::ready1);
inline constexpr auto ready2_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::ready2);
inline constexpr auto ready3_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::ready3);
inline constexpr auto ready4_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::ready4);
inline constexpr auto ready5_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::ready5);
inline constexpr auto ready6_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::ready6);
inline constexpr auto ready7_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::ready7);
inline constexpr auto ready8_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::ready8);
inline constexpr auto ready9_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::ready9);
inline constexpr auto ready10_req = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::ready10);

// Reset (Digital Output)
inline constexpr auto rst1_req = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::rst1);
inline constexpr auto rst2_req = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::rst2);
inline constexpr auto rst3_req = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::rst3);
inline constexpr auto rst4_req = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::rst4);
inline constexpr auto rst5_req = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::rst5);

// ============================================
// Ethernet
// ============================================

#ifdef STLIB_ETH
#if defined(USE_PHY_LAN8742)
inline constexpr auto eth_req = ST_LIB::EthernetDomain::Ethernet(
    ST_LIB::EthernetDomain::PINSET_H10, "00:80:e1:00:01:07", "192.168.1.4", "255.255.0.0"
);
#elif defined(USE_PHY_LAN8700)
inline constexpr auto eth_req = ST_LIB::EthernetDomain::Ethernet(
    ST_LIB::EthernetDomain::PINSET_H10, "00:80:e1:00:01:07", "192.168.1.4", "255.255.0.0"
);
#else
#error "No PHY selected for Ethernet pinset selection"
#endif
#endif

// ============================================
// Board definition
// ============================================

using BoardPolicy =
    ST_LIB::FaultPolicy<LCU_SM::state_machine, LCU_SM::on_fault_enter>;

using Board = ST_LIB::Board<
    BoardPolicy,
#ifdef STLIB_ETH
    eth_req,
#endif
    led_connected_req,
    led_fault_req,
    led_debug_req,
    led_current_control_req,
    led_levitation_req,
    master_fault_req,
    slave_fault_req,
    spi_req,
    slave_ready_req,
    fault1_req,
    fault2_req,
    fault3_req,
    fault4_req,
    fault5_req,
    fault6_req,
    fault7_req,
    fault8_req,
    fault9_req,
    fault10_req,
    ready1_req,
    ready2_req,
    ready3_req,
    ready4_req,
    ready5_req,
    ready6_req,
    ready7_req,
    ready8_req,
    ready9_req,
    ready10_req,
    rst1_req,
    rst2_req,
    rst3_req,
    rst4_req,
    rst5_req>;

// ============================================
// Instance references
// ============================================

#ifdef STLIB_ETH
inline constexpr auto& eth = Board::instance_of<eth_req>();
#endif

inline constexpr auto& led_connected = Board::instance_of<led_connected_req>();
inline constexpr auto& led_fault = Board::instance_of<led_fault_req>();
inline constexpr auto& led_debug = Board::instance_of<led_debug_req>();
inline constexpr auto& led_current_control = Board::instance_of<led_current_control_req>();
inline constexpr auto& led_levitation = Board::instance_of<led_levitation_req>();

inline constexpr auto& master_fault = Board::instance_of<master_fault_req>();
inline constexpr auto& slave_fault = Board::instance_of<slave_fault_req>();
inline auto spi = ST_LIB::SPIDomain::SPIWrapper<spi_req>(Board::instance_of<spi_req>());
inline constexpr auto& slave_ready = Board::instance_of<slave_ready_req>();

// Reset pin instances
inline constexpr auto& rst1 = Board::instance_of<rst1_req>();
inline constexpr auto& rst2 = Board::instance_of<rst2_req>();
inline constexpr auto& rst3 = Board::instance_of<rst3_req>();
inline constexpr auto& rst4 = Board::instance_of<rst4_req>();
inline constexpr auto& rst5 = Board::instance_of<rst5_req>();

// ============================================
// Hardware tuples (all defined, used by pack expansion)
// ============================================

// Ready/ Fault tuples (connector index 0..9)
inline auto all_ready = std::make_tuple(
    &Board::instance_of<ready1_req>(),
    &Board::instance_of<ready2_req>(),
    &Board::instance_of<ready3_req>(),
    &Board::instance_of<ready4_req>(),
    &Board::instance_of<ready5_req>(),
    &Board::instance_of<ready6_req>(),
    &Board::instance_of<ready7_req>(),
    &Board::instance_of<ready8_req>(),
    &Board::instance_of<ready9_req>(),
    &Board::instance_of<ready10_req>()
);
inline auto all_fault = std::make_tuple(
    &Board::instance_of<fault1_req>(),
    &Board::instance_of<fault2_req>(),
    &Board::instance_of<fault3_req>(),
    &Board::instance_of<fault4_req>(),
    &Board::instance_of<fault5_req>(),
    &Board::instance_of<fault6_req>(),
    &Board::instance_of<fault7_req>(),
    &Board::instance_of<fault8_req>(),
    &Board::instance_of<fault9_req>(),
    &Board::instance_of<fault10_req>()
);

// RST tuple (connector index 0..4)
inline auto all_rst = std::forward_as_tuple(rst1, rst2, rst3, rst4, rst5);

// ============================================
// LPU setup using pack expansion
// ============================================

template <size_t VirtualIdx> auto make_lpu_from_config() {
    constexpr auto id = LCUConfig::lpu_virtual_to_connector(VirtualIdx);
    return LPU(*std::get<id>(all_ready), *std::get<id>(all_fault));
}

template <size_t VirtualIdx> struct LpuStorage {
    inline static auto lpu = make_lpu_from_config<VirtualIdx>();
};

template <size_t VirtualIdx> auto& lpu_ref() { return LpuStorage<VirtualIdx>::lpu; }

using LpuSeq = std::make_index_sequence<LCUConfig::ACTIVE_LPU_COUNT>;

template <size_t... Is> auto make_lpu_tuple(std::index_sequence<Is...>) {
    return std::forward_as_tuple(lpu_ref<Is>()...);
}

inline auto lpu_tuple = make_lpu_tuple(LpuSeq{});
inline LpuArray lpu_array(lpu_tuple, all_rst);

// ============================================
// Airgap setup using pack expansion
// ============================================

template <size_t VirtualIdx> auto make_airgap_from_config() {
    return Airgap{};
}

template <size_t VirtualIdx> struct AirgapStorage {
    inline static auto airgap = make_airgap_from_config<VirtualIdx>();
};

template <size_t VirtualIdx> auto& airgap_ref() { return AirgapStorage<VirtualIdx>::airgap; }

using AirgapSeq = std::make_index_sequence<LCUConfig::ACTIVE_AIRGAP_COUNT>;

template <size_t... Is> auto make_airgap_tuple(std::index_sequence<Is...>) {
    return std::forward_as_tuple(airgap_ref<Is>()...);
}

inline auto airgap_tuple = make_airgap_tuple(AirgapSeq{});
inline AirgapArray<decltype(airgap_tuple)> airgap_array(airgap_tuple);

// ============================================
// Frame
// ============================================

using Frame = FrameType<true, decltype(lpu_array), decltype(airgap_array)>;

} // namespace LCU_Master

#endif // LCU_MASTER_HPP
