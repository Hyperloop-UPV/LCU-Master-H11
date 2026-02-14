#ifndef COMMUNICATIONS_HPP
#define COMMUNICATIONS_HPP

#include "Communications/Packets/DataPackets.hpp"
#include "Communications/Packets/OrderPackets.hpp"
#include "CommunicationsShared.hpp"
#include "ConfigShared.hpp"
#include "FrameShared.hpp"
#include "HALAL/Models/SPI/SPI2.hpp"
#include "LCU_MASTER_TYPES.hpp"

namespace Comms {
inline CommunicationsBase communications;

// Order Variables
inline float desired_distance = 0.0f;
inline float desired_current = 0.0f;
inline uint32_t pwm_frequency = 0;
inline float pwm_duty_cycle = 0.0f;

inline ST_LIB::SPIDomain::SPIWrapper<LCU_Master::spi_req>* g_spi = nullptr;
inline ST_LIB::DigitalInputDomain::Instance* g_slave_ready = nullptr;
#ifdef STLIB_ETH
inline ST_LIB::EthernetDomain::Instance* g_eth = nullptr;
#endif

volatile bool send_flag = false;
volatile bool spi_flag = false;
volatile bool receive_flag = false;
volatile bool operation_flag = false;

inline void start() {
    // Initialize Orders
    OrderPackets::levitate_init(desired_distance);
    OrderPackets::stop_levitate_init();
    OrderPackets::current_control_init(desired_current);
    OrderPackets::start_pwm_init(pwm_frequency, pwm_duty_cycle);
    OrderPackets::stop_pwm_init();

    // Initialize Data Packets
    DataPackets::lpu_currents_init(LCU_Master::lcu_vbat_1, LCU_Master::lcu_coil_current_1);
    DataPackets::airgap_measurements_init(LCU_Master::lcu_airgap_1);
    DataPackets::state_machine_init(
        LCU_Master::general_state_machine_state,
        LCU_Master::operational_state_machine_state
    );

    DataPackets::start();
}

inline bool is_connected() {
#ifdef STLIB_ETH
    g_eth->update();
    return DataPackets::control_station_tcp->is_connected();
#else
    return true; // If no Ethernet, assume always connected for SPI
#endif
}

// Must later clear flags
inline void update() {
#ifdef STLIB_ETH
    g_eth->update();
#endif

    // Process TCP/UDP Orders
    if (OrderPackets::levitate_flag) {
        communications.send_order(OrderID::LEVITATE, desired_distance);
    }

    if (OrderPackets::stop_levitate_flag) {
        communications.send_order(OrderID::STOP_LEVITATE);
    }

    if (OrderPackets::current_control_flag) {
        communications.send_order(OrderID::CURRENT_CONTROL, desired_current, 0.0f);
    }

    if (OrderPackets::start_pwm_flag) {
        communications
            .send_order(OrderID::START_PWM, static_cast<float>(pwm_frequency), pwm_duty_cycle);
    }

    if (OrderPackets::stop_pwm_flag) {
        communications.send_order(OrderID::STOP_PWM, 0.0f, 0.0f);
    }

    // Synchronize Flags
    communications.update_flag_synchronization();

    // SPI Communication Logic
    if (!operation_flag) {
        operation_flag = true;
        LCU_Master::CommsFrame::update_tx(&send_flag);
    } else if (send_flag) {
        if (g_slave_ready->read() == GPIO_PIN_SET) {
            send_flag = false;
            g_spi->transceive_DMA(
                LCU_Master::CommsFrame::tx_buffer,
                LCU_Master::CommsFrame::rx_buffer,
                &spi_flag
            );
        }
    } else if (spi_flag) {
        spi_flag = false;
        LCU_Master::CommsFrame::update_rx(&receive_flag);
    } else if (receive_flag) {
        receive_flag = false;
        operation_flag = false;
    }
}

inline void clear_flags() {
    OrderPackets::levitate_flag = false;
    OrderPackets::current_control_flag = false;
    OrderPackets::start_pwm_flag = false;
    OrderPackets::stop_pwm_flag = false;
}
}; // namespace Comms

#endif // COMMUNICATIONS_HPP
