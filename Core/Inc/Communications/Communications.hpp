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
inline ST_LIB::EXTIDomain::Instance* g_slave_ready = nullptr;
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

bool levitating_state = false;

float desired_levitation_distance = 0.0f;
float desired_current = 0.0f;
uint32_t pwm_frequency = 0;
float pwm_duty_cycle = 0.0f;
float fixed_vbat = 0.0f;
uint32_t buffer_id = 0;

float vbat = 5.0f;
float shunt = 0.0f;
float airgap = 0.0f;

inline void start() {
    // Initialize Orders
    OrderPackets::levitate_init(desired_levitation_distance);
    OrderPackets::stop_levitate_init();
    OrderPackets::current_control_init(desired_current);
    OrderPackets::start_pwm_init(pwm_frequency, pwm_duty_cycle);
    OrderPackets::stop_pwm_init();
    OrderPackets::enable_buffer_init(buffer_id);
    OrderPackets::disable_buffer_init(buffer_id);
    OrderPackets::set_fixed_vbat_init(fixed_vbat);
    OrderPackets::unset_fixed_vbat_init();
    OrderPackets::set_control_params_init();
    OrderPackets::reset_master_init();
    OrderPackets::reset_slave_init();
    OrderPackets::reset_init();

    // Initialize Data Packets
    DataPackets::lpu_currents_init(vbat, shunt);
    DataPackets::airgap_measurements_init(airgap);
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
    OrderPackets::enable_buffer_flag = false;
    OrderPackets::disable_buffer_flag = false;
    OrderPackets::set_fixed_vbat_flag = false;
    OrderPackets::unset_fixed_vbat_flag = false;
    OrderPackets::set_control_params_flag = false;
    OrderPackets::reset_slave_flag = false;
    OrderPackets::reset_master_flag = false;
    OrderPackets::reset_flag = false;
}

inline void reset_slave() {
    for (int i = 0; i < 5; i++) {
        LCU_Master::master_fault->turn_off();
        while (!LCU_Master::slave_fault_triggered);
        LCU_Master::slave_fault_triggered = false;
        LCU_Master::master_fault->turn_on();
    }
}

inline void update() {
#ifdef STLIB_ETH
    g_eth->update();
#endif

    //SPI Communication Logic
    if (!operation_flag) {
 
        if (OrderPackets::levitate_flag) {
            communications.command_packet.flags = communications.command_packet.flags | CommandFlags::LEVITATE;
            communications.command_packet.levitate.desired_distance = desired_levitation_distance;
            levitating_state = true;
        } 
        
        if (OrderPackets::stop_levitate_flag) {
            communications.command_packet.flags = communications.command_packet.flags & ~CommandFlags::LEVITATE;
            levitating_state = false;
        }

        if (OrderPackets::current_control_flag) {
            communications.command_packet.flags = communications.command_packet.flags | CommandFlags::CURRENT_CONTROL;
            communications.command_packet.current_control.lpu_id_bitmask = 0x01; 
            communications.command_packet.current_control.desired_current = desired_current;
        } 

        if (OrderPackets::start_pwm_flag) {
            LCU_Master::lpu1->fixed_duty_cycle = pwm_duty_cycle;
            LCU_Master::lpu1->is_fixed_duty_cycle = true;
        } 
        
        if (OrderPackets::stop_pwm_flag) {
            LCU_Master::lpu1->is_fixed_duty_cycle = false;
        }

        if (OrderPackets::enable_buffer_flag) {
            communications.command_packet.flags = communications.command_packet.flags | CommandFlags::ENABLE_LPU_BUFFER;
            communications.command_packet.force_enable_lpu_buffer.lpu_buffer_id_bitmask |= (1 << buffer_id);
        }

        if (OrderPackets::disable_buffer_flag) {
            communications.command_packet.force_enable_lpu_buffer.lpu_buffer_id_bitmask &= ~(1 << buffer_id);
            if (communications.command_packet.force_enable_lpu_buffer.lpu_buffer_id_bitmask == 0) {
                communications.command_packet.flags = communications.command_packet.flags & ~CommandFlags::ENABLE_LPU_BUFFER;
            }
        }

        if (OrderPackets::set_fixed_vbat_flag) {
            LCU_Master::lpu1->is_fixed_vbat = true;
            LCU_Master::lpu1->fixed_vbat = fixed_vbat;
        }

        if (OrderPackets::unset_fixed_vbat_flag) {
            LCU_Master::lpu1->is_fixed_vbat = false;
        }

        if (OrderPackets::set_control_params_flag) {
            // TODO
        }

        if (OrderPackets::reset_master_flag) {
            pending_master_reset = true;
        }

        if (OrderPackets::reset_slave_flag) {
            reset_slave();
        }

        if (OrderPackets::reset_flag) {
            reset_slave();
            pending_master_reset = true;
        }

        clear_flags();

        // // SPI Timeout Logic
        // if (spi_connected && HAL_GetTick() - last_spi_packet_ms > SPI_TIMEOUT_MS) {
        //     spi_connected = false;
        // }

        operation_flag = true;
        LCU_Master::CommsFrame::update_tx(&send_flag);
    } else if (send_flag) {
        if (LCU_Master::slave_ready_triggered ||
            (!spi_connected && g_slave_ready->read() == GPIO_PIN_SET)) {
            LCU_Master::slave_ready_triggered = false;
            send_flag = false;
            g_spi->transceive_DMA(
                LCU_Master::CommsFrame::tx_buffer,
                LCU_Master::CommsFrame::rx_buffer,
                &spi_flag
            );
        }
        
    } else if (spi_flag) {
        spi_flag = false;
        
        if (pending_master_reset) {
            pending_master_reset = false;
            HAL_NVIC_SystemReset();
        }

        LCU_Master::CommsFrame::update_rx(&receive_flag);

    } else if (receive_flag) {
        receive_flag = false;

        if (communications.status_packet.start_byte == StatusPacket::START_BYTE &&
            communications.status_packet.end_byte == StatusPacket::END_BYTE) {
            
            last_spi_packet_ms = HAL_GetTick();
            spi_connected = true;

            vbat = LCU_Master::lpu1->vbat_v;
            shunt = LCU_Master::lpu1->shunt_v;
            airgap = LCU_Master::airgap1.airgap_v;
        }

        operation_flag = false;
    }
}
}; // namespace Comms

#endif // COMMUNICATIONS_HPP
