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

inline ST_LIB::SPIDomain::SPIWrapper<LCU_Master::spi_req>* g_spi = nullptr;
inline ST_LIB::DigitalInputDomain::Instance* g_slave_ready = nullptr;
#ifdef STLIB_ETH
inline ST_LIB::EthernetDomain::Instance* g_eth = nullptr;
#endif

volatile bool send_flag = false;
volatile bool spi_flag = false;
volatile bool receive_flag = false;
volatile bool operation_flag = false;
bool pending_master_reset = false;
inline uint32_t last_spi_packet_ms = 0;
inline bool spi_connected = false;
constexpr uint32_t SPI_TIMEOUT_MS = 1000;

inline void start() {
    // Initialize Orders
    OrderPackets::levitate_init(communications.command_packet.levitate.desired_distance);
    OrderPackets::stop_levitate_init();
    OrderPackets::current_control_init(communications.command_packet.current_control.desired_current);
    OrderPackets::start_pwm_init(communications.command_packet.pwm.frequency, communications.command_packet.pwm.duty_cycle);
    OrderPackets::stop_pwm_init();
    OrderPackets::start_control_loop_init();
    OrderPackets::stop_control_loop_init();
    OrderPackets::set_fixed_vbat_init(communications.command_packet.fixed_vbat.fixed_vbat);
    OrderPackets::unset_fixed_vbat_init();
    OrderPackets::reset_init();

    // Initialize Data Packets
    DataPackets::lpu_currents_init(LCU_Master::lpu1->vbat_v, LCU_Master::lpu1->shunt_v);
    DataPackets::airgap_measurements_init(LCU_Master::airgap1.airgap_v);
    DataPackets::state_machine_init(
        LCU_Master::general_state_machine_state,
        LCU_Master::operational_state_machine_state
    );

    DataPackets::start();
    OrderPackets::start();
}

inline bool is_connected() {
#ifdef STLIB_ETH
    g_eth->update();
    return OrderPackets::control_station_tcp->is_connected() && spi_connected;
#else
    return spi_connected;
#endif
}

inline void clear_flags() {
    OrderPackets::levitate_flag = false;
    OrderPackets::stop_levitate_flag = false;
    OrderPackets::current_control_flag = false;
    OrderPackets::start_pwm_flag = false;
    OrderPackets::stop_pwm_flag = false;
    OrderPackets::start_control_loop_flag = false;
    OrderPackets::stop_control_loop_flag = false;
    OrderPackets::set_fixed_vbat_flag = false;
    OrderPackets::unset_fixed_vbat_flag = false;
    OrderPackets::set_control_params_flag = false;
    OrderPackets::reset_slave_flag = false;
    OrderPackets::reset_master_flag = false;
    OrderPackets::reset_flag = false;
}

inline void update() {
#ifdef STLIB_ETH
    g_eth->update();
#endif

    //SPI Communication Logic
    if (!operation_flag) {
 
        if (OrderPackets::levitate_flag) {
            communications.command_packet.flags = communications.command_packet.flags | CommandFlags::LEVITATE;
        } 
        
        if (OrderPackets::stop_levitate_flag) {
             communications.command_packet.flags = communications.command_packet.flags & ~CommandFlags::LEVITATE;
        }

        if (OrderPackets::current_control_flag) {
            communications.command_packet.flags = communications.command_packet.flags | CommandFlags::CURRENT_CONTROL;
            communications.command_packet.current_control.lpu_id_bitmask = 0x01; 
        } 

        if (OrderPackets::start_pwm_flag) {
            communications.command_packet.flags = communications.command_packet.flags | CommandFlags::PWM;
            communications.command_packet.pwm.lpu_id_bitmask = 0x01;
        } 
        
        if (OrderPackets::stop_pwm_flag) {
            communications.command_packet.flags = communications.command_packet.flags & ~CommandFlags::PWM;
        }

        if (OrderPackets::start_control_loop_flag) {
            communications.command_packet.flags = communications.command_packet.flags | CommandFlags::CONTROL_LOOP;
        }

        if (OrderPackets::stop_control_loop_flag) {
            communications.command_packet.flags = communications.command_packet.flags & ~CommandFlags::CONTROL_LOOP;
        }

        if (OrderPackets::set_fixed_vbat_flag) {
            communications.command_packet.flags = communications.command_packet.flags | CommandFlags::FIXED_VBAT;
        }

        if (OrderPackets::unset_fixed_vbat_flag) {
            communications.command_packet.flags = communications.command_packet.flags & ~CommandFlags::FIXED_VBAT;
        }

        if (OrderPackets::set_control_params_flag) {
            // TODO
        }

        if (OrderPackets::reset_master_flag) {
            pending_master_reset = true;
        }

        if (OrderPackets::reset_slave_flag) {
            communications.command_packet.flags = communications.command_packet.flags | CommandFlags::RESET_SLAVE;
        }

        if (OrderPackets::reset_flag) {
            communications.command_packet.flags = communications.command_packet.flags | CommandFlags::RESET_SLAVE;
            pending_master_reset = true;
        }

        clear_flags();

        // // SPI Timeout Logic
        // if (spi_connected && HAL_GetTick() - last_spi_packet_ms > SPI_TIMEOUT_MS) {
        //     spi_connected = false;
        // }

        operation_flag = true;
        LCU_Master::CommsFrame::update_tx(&send_flag);
        while (!send_flag) { // Busy wait for synchronization
            MDMA::update();
        }
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
        
        if ((communications.command_packet.flags & CommandFlags::RESET_SLAVE) == CommandFlags::RESET_SLAVE) {
            communications.command_packet.flags = communications.command_packet.flags & ~CommandFlags::RESET_SLAVE;
        }

        if (pending_master_reset) {
            pending_master_reset = false;
            HAL_NVIC_SystemReset();
        }

        LCU_Master::CommsFrame::update_rx(&receive_flag);
        while (!receive_flag) { // Busy wait for synchronization
            MDMA::update();
        }

    } else if (receive_flag) {
        receive_flag = false;

        if (communications.status_packet.start_byte == StatusPacket::START_BYTE &&
            communications.status_packet.end_byte == StatusPacket::END_BYTE) {
            
            last_spi_packet_ms = HAL_GetTick();
            spi_connected = true;
        }

        operation_flag = false;
    }
}
}; // namespace Comms

#endif // COMMUNICATIONS_HPP
