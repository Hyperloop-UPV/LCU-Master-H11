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
    inline uint8_t lpu_id = 0;
    inline float desired_current = 0.0f;
    inline uint32_t pwm_frequency = 0;
    inline float pwm_duty_cycle = 0.0f;

    // Data Variables
    inline float lcu_vbat[10];
    inline float lcu_coil_current[10];
    inline float lcu_airgap[8];

    inline ST_LIB::SPIDomain::SPIWrapper<LCU_Master::spi_req> *g_spi = nullptr;
    inline ST_LIB::DigitalOutputDomain::Instance *g_master_ready = nullptr;
    #ifdef STLIB_ETH
    inline ST_LIB::EthernetDomain::Instance *g_eth = nullptr;
    #endif

    volatile bool send_flag = false;
    volatile bool spi_flag = false;
    volatile bool receive_flag = false;
    volatile bool operation_flag = false;

    inline void start() {
        // Initialize Orders
        OrderPackets::levitate_init(desired_distance);
        OrderPackets::current_control_init(lpu_id, desired_current);
        OrderPackets::start_pwm_init(lpu_id, pwm_frequency, pwm_duty_cycle);
        OrderPackets::stop_pwm_init(lpu_id);

        // Initialize Data Packets
        DataPackets::lpu_currents_init(
            lcu_vbat[0], lcu_vbat[1], lcu_vbat[2], lcu_vbat[3], lcu_vbat[4], 
            lcu_vbat[5], lcu_vbat[6], lcu_vbat[7], lcu_vbat[8], lcu_vbat[9],
            lcu_coil_current[0], lcu_coil_current[1], lcu_coil_current[2], lcu_coil_current[3], lcu_coil_current[4],
            lcu_coil_current[5], lcu_coil_current[6], lcu_coil_current[7], lcu_coil_current[8], lcu_coil_current[9]
        );
        DataPackets::airgap_measurements_init(
            lcu_airgap[0], lcu_airgap[1], lcu_airgap[2], lcu_airgap[3], 
            lcu_airgap[4], lcu_airgap[5], lcu_airgap[6], lcu_airgap[7]
        );

        DataPackets::start();
    }

    // Must later clear flags
    inline void update() {
        #ifdef STLIB_ETH
        g_eth->update();
        #endif

        // Process TCP/UDP Orders
        if (OrderPackets::levitate_flag) {
            communications.send_order(OrderID::LEVITATE, desired_distance);
            OrderPackets::levitate_flag = false;
        }

        if (OrderPackets::current_control_flag) {
            communications.send_order(OrderID::CURRENT_CONTROL, desired_current, 0.0f, lpu_id);
            OrderPackets::current_control_flag = false;
        }

        if (OrderPackets::start_pwm_flag) {
            communications.send_order(OrderID::START_PWM, static_cast<float>(pwm_frequency), pwm_duty_cycle, lpu_id);
            OrderPackets::start_pwm_flag = false;
        }

        if (OrderPackets::stop_pwm_flag) {
            communications.send_order(OrderID::STOP_PWM, 0.0f, 0.0f, lpu_id);
            OrderPackets::stop_pwm_flag = false;
        }

        // Synchronize Flags
        communications.update_flag_synchronization();

        // SPI Communication Logic
        if (!operation_flag) {
            operation_flag = true;
            LCU_Master::CommsFrame::update_tx(&send_flag);
        } else if (send_flag) {
            send_flag = false;
            g_spi->transceive(LCU_Master::CommsFrame::tx_buffer, LCU_Master::CommsFrame::rx_buffer, &spi_flag);
            g_master_ready->turn_on();
        } else if (spi_flag) {
            g_master_ready->turn_off();
            spi_flag = false;
            LCU_Master::CommsFrame::update_rx(&receive_flag);
        } else if (receive_flag) {
            receive_flag = false;
            operation_flag = false;
        }
    }

    inline void clear_flags() {
        send_flag = false;
        spi_flag = false;
        receive_flag = false;
        operation_flag = false;
    }
};

#endif // COMMUNICATIONS_HPP