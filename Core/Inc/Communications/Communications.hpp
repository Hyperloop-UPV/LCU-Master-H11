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
inline uint32_t last_spi_packet_ms = 0;
inline bool spi_connected = false;
constexpr uint32_t SPI_TIMEOUT_MS = 1000;

bool levitating_state = false;

float desired_levitation_distance = 0.0f;
float desired_current = 0.0f;
float pwm_duty_cycle = 0.0f;
float fixed_vbat = 0.0f;
#ifdef USE_5_DOF
uint32_t current_control_id = 0;
uint32_t start_pwm_id = 0;
uint32_t stop_pwm_id = 0;
uint32_t enable_buffer_id = 0;
uint32_t disable_buffer_id = 0;
#endif

#ifdef USE_1_DOF
float vbat = 5.0f;
float shunt = 0.0f;
float airgap = 0.0f;
float curr_pwm_duty_cycle = 0.0f;
#elif defined(USE_5_DOF)
float lpu_vbat[10] = {5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f};
float lpu_shunt[10] = {0.0f};
float lpu_pwm_duty[10] = {0.0f};
float airgap_measurements[8] = {0.0f};
#endif

inline void reset_slave() {
    for (int i = 0; i < 5; i++) {
        LCU_Master::master_fault->turn_off();
        HAL_Delay(10);
        LCU_Master::master_fault->turn_on();
        HAL_Delay(10);
    }
    HAL_Delay(100); // Ensure slave has time to reset
    LCU_Master::slave_fault_triggered = false;
}

inline void start() {
    reset_slave();

    // Initialize Orders
    OrderPackets::Stop_All_init();
    OrderPackets::Levitate_init(desired_levitation_distance);
    OrderPackets::Stop_Levitate_init();
#ifdef USE_1_DOF
    OrderPackets::Current_Control_init(desired_current);
    OrderPackets::Start_PWM_init(pwm_duty_cycle);
    OrderPackets::Stop_PWM_init();
    OrderPackets::Enable_Buffer_init();
    OrderPackets::Disable_Buffer_init();
#elif defined(USE_5_DOF)
    OrderPackets::Current_Control_init(desired_current, current_control_id);
    OrderPackets::Start_PWM_init(pwm_duty_cycle, start_pwm_id);
    OrderPackets::Stop_PWM_init(stop_pwm_id);
    OrderPackets::Enable_Buffer_init(enable_buffer_id);
    OrderPackets::Disable_Buffer_init(disable_buffer_id);
#endif
    OrderPackets::Set_Fixed_VBAT_init(fixed_vbat);
    OrderPackets::Unset_Fixed_VBAT_init();
    OrderPackets::Set_Control_Params_init();
    OrderPackets::Reset_Master_init();
    OrderPackets::Reset_Slave_init();
    OrderPackets::Reset_All_init();

    // Initialize Data Packets
#ifdef USE_1_DOF
    float vbat = 5.0f;
    float shunt = 0.0f;
    float airgap = 0.0f;
    float curr_pwm_duty_cycle = 0.0f;
    DataPackets::LPU_1_measurements_init(vbat, shunt, curr_pwm_duty_cycle);
    DataPackets::Airgap_1_measurements_init(airgap);
#elif defined(USE_5_DOF)
    DataPackets::LPU_1_measurements_init(lpu_vbat[0], lpu_shunt[0], lpu_pwm_duty[0]);
    DataPackets::LPU_2_measurements_init(lpu_vbat[1], lpu_shunt[1], lpu_pwm_duty[1]);
    DataPackets::LPU_3_measurements_init(lpu_vbat[2], lpu_shunt[2], lpu_pwm_duty[2]);
    DataPackets::LPU_4_measurements_init(lpu_vbat[3], lpu_shunt[3], lpu_pwm_duty[3]);
    DataPackets::LPU_5_measurements_init(lpu_vbat[4], lpu_shunt[4], lpu_pwm_duty[4]);
    DataPackets::LPU_6_measurements_init(lpu_vbat[5], lpu_shunt[5], lpu_pwm_duty[5]);
    DataPackets::LPU_7_measurements_init(lpu_vbat[6], lpu_shunt[6], lpu_pwm_duty[6]);
    DataPackets::LPU_8_measurements_init(lpu_vbat[7], lpu_shunt[7], lpu_pwm_duty[7]);
    DataPackets::LPU_9_measurements_init(lpu_vbat[8], lpu_shunt[8], lpu_pwm_duty[8]);
    DataPackets::LPU_10_measurements_init(lpu_vbat[9], lpu_shunt[9], lpu_pwm_duty[9]);
    DataPackets::Airgap_1_measurements_init(airgap_measurements[0]);
    DataPackets::Airgap_2_measurements_init(airgap_measurements[1]);
    DataPackets::Airgap_3_measurements_init(airgap_measurements[2]);
    DataPackets::Airgap_4_measurements_init(airgap_measurements[3]);
    DataPackets::Airgap_5_measurements_init(airgap_measurements[4]);
    DataPackets::Airgap_6_measurements_init(airgap_measurements[5]);
    DataPackets::Airgap_7_measurements_init(airgap_measurements[6]);
    DataPackets::Airgap_8_measurements_init(airgap_measurements[7]);
#endif
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
#ifdef USE_1_DOF
            communications.command_packet.current_control.lpu_id_bitmask = 0x01;
#elif defined(USE_5_DOF)
            if (current_control_id > 0 && current_control_id <= 10) {
                communications.command_packet.current_control.lpu_id_bitmask = (1 << (current_control_id - 1));
            }
#endif
            communications.command_packet.current_control.desired_current = desired_current;
        }

        if (OrderPackets::Start_PWM_flag) {
#ifdef USE_1_DOF
            LCU_Master::lpu_array->get_lpu<0>().fixed_duty_cycle = pwm_duty_cycle;
            LCU_Master::lpu_array->get_lpu<0>().is_fixed_duty_cycle = true;
#elif defined(USE_5_DOF)
            switch(start_pwm_id) {
                case 1: LCU_Master::lpu_array->get_lpu<0>().fixed_duty_cycle = pwm_duty_cycle; 
                        LCU_Master::lpu_array->get_lpu<0>().is_fixed_duty_cycle = true; break;
                case 2: LCU_Master::lpu_array->get_lpu<1>().fixed_duty_cycle = pwm_duty_cycle; 
                        LCU_Master::lpu_array->get_lpu<1>().is_fixed_duty_cycle = true; break;
                case 3: LCU_Master::lpu_array->get_lpu<2>().fixed_duty_cycle = pwm_duty_cycle; 
                        LCU_Master::lpu_array->get_lpu<2>().is_fixed_duty_cycle = true; break;
                case 4: LCU_Master::lpu_array->get_lpu<3>().fixed_duty_cycle = pwm_duty_cycle; 
                        LCU_Master::lpu_array->get_lpu<3>().is_fixed_duty_cycle = true; break;
                case 5: LCU_Master::lpu_array->get_lpu<4>().fixed_duty_cycle = pwm_duty_cycle; 
                        LCU_Master::lpu_array->get_lpu<4>().is_fixed_duty_cycle = true; break;
                case 6: LCU_Master::lpu_array->get_lpu<5>().fixed_duty_cycle = pwm_duty_cycle; 
                        LCU_Master::lpu_array->get_lpu<5>().is_fixed_duty_cycle = true; break;
                case 7: LCU_Master::lpu_array->get_lpu<6>().fixed_duty_cycle = pwm_duty_cycle; 
                        LCU_Master::lpu_array->get_lpu<6>().is_fixed_duty_cycle = true; break;
                case 8: LCU_Master::lpu_array->get_lpu<7>().fixed_duty_cycle = pwm_duty_cycle; 
                        LCU_Master::lpu_array->get_lpu<7>().is_fixed_duty_cycle = true; break;
                case 9: LCU_Master::lpu_array->get_lpu<8>().fixed_duty_cycle = pwm_duty_cycle; 
                        LCU_Master::lpu_array->get_lpu<8>().is_fixed_duty_cycle = true; break;
                case 10: LCU_Master::lpu_array->get_lpu<9>().fixed_duty_cycle = pwm_duty_cycle; 
                         LCU_Master::lpu_array->get_lpu<9>().is_fixed_duty_cycle = true; break;
            }
#endif
        }

        if (OrderPackets::Stop_PWM_flag) {
#ifdef USE_1_DOF
            LCU_Master::lpu_array->get_lpu<0>().is_fixed_duty_cycle = false;
#elif defined(USE_5_DOF)
            switch(stop_pwm_id) {
                case 1: LCU_Master::lpu_array->get_lpu<0>().is_fixed_duty_cycle = false; break;
                case 2: LCU_Master::lpu_array->get_lpu<1>().is_fixed_duty_cycle = false; break;
                case 3: LCU_Master::lpu_array->get_lpu<2>().is_fixed_duty_cycle = false; break;
                case 4: LCU_Master::lpu_array->get_lpu<3>().is_fixed_duty_cycle = false; break;
                case 5: LCU_Master::lpu_array->get_lpu<4>().is_fixed_duty_cycle = false; break;
                case 6: LCU_Master::lpu_array->get_lpu<5>().is_fixed_duty_cycle = false; break;
                case 7: LCU_Master::lpu_array->get_lpu<6>().is_fixed_duty_cycle = false; break;
                case 8: LCU_Master::lpu_array->get_lpu<7>().is_fixed_duty_cycle = false; break;
                case 9: LCU_Master::lpu_array->get_lpu<8>().is_fixed_duty_cycle = false; break;
                case 10: LCU_Master::lpu_array->get_lpu<9>().is_fixed_duty_cycle = false; break;
            }
#endif
        }

        if (OrderPackets::Enable_Buffer_flag) {
            communications.command_packet.flags =
                communications.command_packet.flags | CommandFlags::ENABLE_LPU_BUFFER;
            LCU_Master::lpu_array->enable_pair(enable_buffer_id - 1); // Convert to 0-based index
#ifdef USE_1_DOF
            communications.command_packet.force_enable_lpu_buffer.lpu_buffer_id_bitmask |=
                (1 << 0);
#elif defined(USE_5_DOF)
            if (enable_buffer_id > 0 && enable_buffer_id <= 10) {
                communications.command_packet.force_enable_lpu_buffer.lpu_buffer_id_bitmask |=
                    (1 << (enable_buffer_id - 1));
            }
#endif
        }

        if (OrderPackets::Disable_Buffer_flag) {
            LCU_Master::lpu_array->disable_pair(disable_buffer_id - 1); // Convert to 0-based index
#ifdef USE_1_DOF
            communications.command_packet.force_enable_lpu_buffer.lpu_buffer_id_bitmask &=
                ~(1 << 0);
#elif defined(USE_5_DOF)
            if (disable_buffer_id > 0 && disable_buffer_id <= 10) {
                communications.command_packet.force_enable_lpu_buffer.lpu_buffer_id_bitmask &=
                    ~(1 << (disable_buffer_id - 1));
            }
#endif
            if (communications.command_packet.force_enable_lpu_buffer.lpu_buffer_id_bitmask == 0) {
                communications.command_packet.flags =
                    communications.command_packet.flags & ~CommandFlags::ENABLE_LPU_BUFFER;
            }
        }

        if (OrderPackets::Set_Fixed_VBAT_flag) {
#ifdef USE_1_DOF
            LCU_Master::lpu_array->get_lpu<0>().is_fixed_vbat = true;
            LCU_Master::lpu_array->get_lpu<0>().fixed_vbat = fixed_vbat;
#elif defined(USE_5_DOF)
            LCU_Master::lpu_array->get_lpu<0>().is_fixed_vbat = true; LCU_Master::lpu_array->get_lpu<0>().fixed_vbat = fixed_vbat;
            LCU_Master::lpu_array->get_lpu<1>().is_fixed_vbat = true; LCU_Master::lpu_array->get_lpu<1>().fixed_vbat = fixed_vbat;
            LCU_Master::lpu_array->get_lpu<2>().is_fixed_vbat = true; LCU_Master::lpu_array->get_lpu<2>().fixed_vbat = fixed_vbat;
            LCU_Master::lpu_array->get_lpu<3>().is_fixed_vbat = true; LCU_Master::lpu_array->get_lpu<3>().fixed_vbat = fixed_vbat;
            LCU_Master::lpu_array->get_lpu<4>().is_fixed_vbat = true; LCU_Master::lpu_array->get_lpu<4>().fixed_vbat = fixed_vbat;
            LCU_Master::lpu_array->get_lpu<5>().is_fixed_vbat = true; LCU_Master::lpu_array->get_lpu<5>().fixed_vbat = fixed_vbat;
            LCU_Master::lpu_array->get_lpu<6>().is_fixed_vbat = true; LCU_Master::lpu_array->get_lpu<6>().fixed_vbat = fixed_vbat;
            LCU_Master::lpu_array->get_lpu<7>().is_fixed_vbat = true; LCU_Master::lpu_array->get_lpu<7>().fixed_vbat = fixed_vbat;
            LCU_Master::lpu_array->get_lpu<8>().is_fixed_vbat = true; LCU_Master::lpu_array->get_lpu<8>().fixed_vbat = fixed_vbat;
            LCU_Master::lpu_array->get_lpu<9>().is_fixed_vbat = true; LCU_Master::lpu_array->get_lpu<9>().fixed_vbat = fixed_vbat;
#endif
        }

        if (OrderPackets::Unset_Fixed_VBAT_flag) {
#ifdef USE_1_DOF
            LCU_Master::lpu_array->get_lpu<0>().is_fixed_vbat = false;
#elif defined(USE_5_DOF)
            LCU_Master::lpu_array->get_lpu<0>().is_fixed_vbat = false;
            LCU_Master::lpu_array->get_lpu<1>().is_fixed_vbat = false;
            LCU_Master::lpu_array->get_lpu<2>().is_fixed_vbat = false;
            LCU_Master::lpu_array->get_lpu<3>().is_fixed_vbat = false;
            LCU_Master::lpu_array->get_lpu<4>().is_fixed_vbat = false;
            LCU_Master::lpu_array->get_lpu<5>().is_fixed_vbat = false;
            LCU_Master::lpu_array->get_lpu<6>().is_fixed_vbat = false;
            LCU_Master::lpu_array->get_lpu<7>().is_fixed_vbat = false;
            LCU_Master::lpu_array->get_lpu<8>().is_fixed_vbat = false;
            LCU_Master::lpu_array->get_lpu<9>().is_fixed_vbat = false;
#endif
        }

        if (OrderPackets::Set_Control_Params_flag) {
            // TODO
        }

        if (OrderPackets::Reset_Master_flag) { // Should remove
            HAL_NVIC_SystemReset();
        }

        if (OrderPackets::Reset_Slave_flag) {
            reset_slave();
        }

        if (OrderPackets::Reset_All_flag) {
            HAL_NVIC_SystemReset();
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

        LCU_Master::CommsFrame::update_rx(&receive_flag);

    } else if (receive_flag) {
        receive_flag = false;

        if (communications.status_packet.start_byte == StatusPacket::START_BYTE &&
            communications.status_packet.end_byte == StatusPacket::END_BYTE) {

            last_spi_packet_ms = HAL_GetTick();
            spi_connected = true;

#ifdef USE_1_DOF
            vbat = LCU_Master::lpu_array->get_lpu<0>().vbat_v;
            shunt = LCU_Master::lpu_array->get_lpu<0>().shunt_v;
            airgap = LCU_Master::airgap_array->get_airgap<0>().airgap_v;
            curr_pwm_duty_cycle = LCU_Master::lpu_array->get_lpu<0>().duty_cycle;
#elif defined(USE_5_DOF)
            lpu_vbat[0] = LCU_Master::lpu_array->get_lpu<0>().vbat_v; lpu_shunt[0] = LCU_Master::lpu_array->get_lpu<0>().shunt_v; lpu_pwm_duty[0] = LCU_Master::lpu_array->get_lpu<0>().duty_cycle;
            lpu_vbat[1] = LCU_Master::lpu_array->get_lpu<1>().vbat_v; lpu_shunt[1] = LCU_Master::lpu_array->get_lpu<1>().shunt_v; lpu_pwm_duty[1] = LCU_Master::lpu_array->get_lpu<1>().duty_cycle;
            lpu_vbat[2] = LCU_Master::lpu_array->get_lpu<2>().vbat_v; lpu_shunt[2] = LCU_Master::lpu_array->get_lpu<2>().shunt_v; lpu_pwm_duty[2] = LCU_Master::lpu_array->get_lpu<2>().duty_cycle;
            lpu_vbat[3] = LCU_Master::lpu_array->get_lpu<3>().vbat_v; lpu_shunt[3] = LCU_Master::lpu_array->get_lpu<3>().shunt_v; lpu_pwm_duty[3] = LCU_Master::lpu_array->get_lpu<3>().duty_cycle;
            lpu_vbat[4] = LCU_Master::lpu_array->get_lpu<4>().vbat_v; lpu_shunt[4] = LCU_Master::lpu_array->get_lpu<4>().shunt_v; lpu_pwm_duty[4] = LCU_Master::lpu_array->get_lpu<4>().duty_cycle;
            lpu_vbat[5] = LCU_Master::lpu_array->get_lpu<5>().vbat_v; lpu_shunt[5] = LCU_Master::lpu_array->get_lpu<5>().shunt_v; lpu_pwm_duty[5] = LCU_Master::lpu_array->get_lpu<5>().duty_cycle;
            lpu_vbat[6] = LCU_Master::lpu_array->get_lpu<6>().vbat_v; lpu_shunt[6] = LCU_Master::lpu_array->get_lpu<6>().shunt_v; lpu_pwm_duty[6] = LCU_Master::lpu_array->get_lpu<6>().duty_cycle;
            lpu_vbat[7] = LCU_Master::lpu_array->get_lpu<7>().vbat_v; lpu_shunt[7] = LCU_Master::lpu_array->get_lpu<7>().shunt_v; lpu_pwm_duty[7] = LCU_Master::lpu_array->get_lpu<7>().duty_cycle;
            lpu_vbat[8] = LCU_Master::lpu_array->get_lpu<8>().vbat_v; lpu_shunt[8] = LCU_Master::lpu_array->get_lpu<8>().shunt_v; lpu_pwm_duty[8] = LCU_Master::lpu_array->get_lpu<8>().duty_cycle;
            lpu_vbat[9] = LCU_Master::lpu_array->get_lpu<9>().vbat_v; lpu_shunt[9] = LCU_Master::lpu_array->get_lpu<9>().shunt_v; lpu_pwm_duty[9] = LCU_Master::lpu_array->get_lpu<9>().duty_cycle;
            airgap_measurements[0] = LCU_Master::airgap_array->get_airgap<0>().airgap_v;
            airgap_measurements[1] = LCU_Master::airgap_array->get_airgap<1>().airgap_v;
            airgap_measurements[2] = LCU_Master::airgap_array->get_airgap<2>().airgap_v;
            airgap_measurements[3] = LCU_Master::airgap_array->get_airgap<3>().airgap_v;
            airgap_measurements[4] = LCU_Master::airgap_array->get_airgap<4>().airgap_v;
            airgap_measurements[5] = LCU_Master::airgap_array->get_airgap<5>().airgap_v;
            airgap_measurements[6] = LCU_Master::airgap_array->get_airgap<6>().airgap_v;
            airgap_measurements[7] = LCU_Master::airgap_array->get_airgap<7>().airgap_v;
#endif
        }

        operation_flag = false;
    }
}
}; // namespace Comms

#endif // COMMUNICATIONS_HPP
