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
float pwm_duty_cycle = 0.0f;
float fixed_vbat = 0.0f;
uint32_t buffer_id = 0;

float vbat = 5.0f;
float shunt = 0.0f;
float airgap = 0.0f;
float curr_pwm_duty_cycle = 0.0f;

inline void start() {
    // Initialize Orders
    OrderPackets::Stop_All_init();
    OrderPackets::Levitate_init(desired_levitation_distance);
    OrderPackets::Stop_Levitate_init();
    OrderPackets::Current_Control_init(desired_current);
    OrderPackets::Start_PWM_init(pwm_duty_cycle);
    OrderPackets::Stop_PWM_init();
    OrderPackets::Enable_Buffer_init();
    OrderPackets::Disable_Buffer_init();
    OrderPackets::Set_Fixed_VBAT_init(fixed_vbat);
    OrderPackets::Unset_Fixed_VBAT_init();
    OrderPackets::Set_Control_Params_init();
    OrderPackets::Reset_Master_init();
    OrderPackets::Reset_Slave_init();
    OrderPackets::Reset_All_init();

    // Initialize Data Packets
    DataPackets::LPU_1_measurements_init(vbat, shunt, curr_pwm_duty_cycle);
    DataPackets::Airgap_1_measurements_init(airgap);
    DataPackets::State_Machine_init(
        LCU_Master::general_state_machine_state,
        LCU_Master::operational_state_machine_state
    );
    DataPackets::General_State_init(desired_levitation_distance);

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
    OrderPackets::Stop_All_flag = false;
    OrderPackets::Levitate_flag = false;
    OrderPackets::Stop_Levitate_flag = false;
    OrderPackets::Current_Control_flag = false;
    OrderPackets::Start_PWM_flag = false;
    OrderPackets::Stop_PWM_flag = false;
    OrderPackets::Enable_Buffer_flag = false;
    OrderPackets::Disable_Buffer_flag = false;
    OrderPackets::Set_Fixed_VBAT_flag = false;
    OrderPackets::Unset_Fixed_VBAT_flag = false;
    OrderPackets::Set_Control_Params_flag = false;
    OrderPackets::Reset_Slave_flag = false;
    OrderPackets::Reset_Master_flag = false;
    OrderPackets::Reset_All_flag = false;
}

inline void reset_slave() {
    for (int i = 0; i < 5; i++) {
        LCU_Master::master_fault->turn_off();
        HAL_Delay(100);
        LCU_Master::master_fault->turn_on();
        HAL_Delay(100);
    }
    LCU_Master::slave_fault_triggered = false;
}

inline void update() {
#ifdef STLIB_ETH
    g_eth->update();
#endif

    // SPI Communication Logic
    if (!operation_flag) {

        if (OrderPackets::Stop_All_flag) {
            communications.command_packet.flags = CommandFlags::NONE;
            levitating_state = false;
        }

        if (OrderPackets::Levitate_flag) {
            communications.command_packet.flags =
                communications.command_packet.flags | CommandFlags::LEVITATE;
            communications.command_packet.levitate.desired_distance = desired_levitation_distance;
            levitating_state = true;
        }

        if (OrderPackets::Stop_Levitate_flag) {
            communications.command_packet.flags =
                communications.command_packet.flags & ~CommandFlags::LEVITATE;
            levitating_state = false;
        }

        if (OrderPackets::Current_Control_flag) {
            communications.command_packet.flags =
                communications.command_packet.flags | CommandFlags::CURRENT_CONTROL;
            communications.command_packet.current_control.lpu_id_bitmask = 0x01;
            communications.command_packet.current_control.desired_current = desired_current;
        }

        if (OrderPackets::Start_PWM_flag) {
            LCU_Master::lpu_array->get_lpu<0>().fixed_duty_cycle = pwm_duty_cycle;
            LCU_Master::lpu_array->get_lpu<0>().is_fixed_duty_cycle = true;
        }

        if (OrderPackets::Stop_PWM_flag) {
            LCU_Master::lpu_array->get_lpu<0>().is_fixed_duty_cycle = false;
        }

        if (OrderPackets::Enable_Buffer_flag) {
            buffer_id = 0;
            communications.command_packet.flags =
                communications.command_packet.flags | CommandFlags::ENABLE_LPU_BUFFER;
            communications.command_packet.force_enable_lpu_buffer.lpu_buffer_id_bitmask |=
                (1 << buffer_id);
        }

        if (OrderPackets::Disable_Buffer_flag) {
            buffer_id = 0;
            communications.command_packet.force_enable_lpu_buffer.lpu_buffer_id_bitmask &=
                ~(1 << buffer_id);
            if (communications.command_packet.force_enable_lpu_buffer.lpu_buffer_id_bitmask == 0) {
                communications.command_packet.flags =
                    communications.command_packet.flags & ~CommandFlags::ENABLE_LPU_BUFFER;
            }
        }

        if (OrderPackets::Set_Fixed_VBAT_flag) {
            LCU_Master::lpu_array->get_lpu<0>().is_fixed_vbat = true;
            LCU_Master::lpu_array->get_lpu<0>().fixed_vbat = fixed_vbat;
        }

        if (OrderPackets::Unset_Fixed_VBAT_flag) {
            LCU_Master::lpu_array->get_lpu<0>().is_fixed_vbat = false;
        }

        if (OrderPackets::Set_Control_Params_flag) {
            // TODO
        }

        if (OrderPackets::Reset_Master_flag) {
            pending_master_reset = true;
        }

        if (OrderPackets::Reset_Slave_flag) {
            reset_slave();
        }

        if (OrderPackets::Reset_All_flag) {
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

            vbat = LCU_Master::lpu_array->get_lpu<0>().vbat_v;
            shunt = LCU_Master::lpu_array->get_lpu<0>().shunt_v;
            airgap = LCU_Master::airgap_array->get_airgap<0>().airgap_v;
            curr_pwm_duty_cycle = LCU_Master::lpu_array->get_lpu<0>().duty_cycle;
        }

        operation_flag = false;
    }
}
}; // namespace Comms

#endif // COMMUNICATIONS_HPP
