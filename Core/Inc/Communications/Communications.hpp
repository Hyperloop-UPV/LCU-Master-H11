#ifndef COMMUNICATIONS_HPP
#define COMMUNICATIONS_HPP

#include "Communications/Packets/DataPackets.hpp"
#include "Communications/Packets/OrderPackets.hpp"
#include "ControlShared.hpp"
#include "StateMachineShared.hpp"
#include "ConfigShared.hpp"
#include "FrameShared.hpp"
#include "SharedCommunicationsLogic.hpp"
#include "Common/Flags.hpp"
#include "HALAL/Models/SPI/SPI2.hpp"
#include "LCU_MASTER_TYPES.hpp"

namespace Comms {
inline ControlBase control;
inline StateMachineBase slave_state;

inline ST_LIB::SPIDomain::SPIWrapper<LCU_Master::spi_req>* g_spi = nullptr;
inline ST_LIB::EXTIDomain::Instance* g_slave_ready = nullptr;
#ifdef STLIB_ETH
inline ST_LIB::EthernetDomain::Instance* g_eth = nullptr;
#endif

inline uint32_t last_spi_packet_ms = 0;
inline bool spi_connected = false;
constexpr uint32_t SPI_TIMEOUT_MS = 1000;

bool levitating_state = false;
bool current_control_active = false;  // Track if current control mode is active
uint16_t fixed_pwm_active = 0;        // Track if any LPU has fixed PWM enabled, as a bitmask

float desired_levitation_distance = 0.0f;
float desired_current = 0.0f;
float pwm_duty_cycle = 0.0f;
float fixed_vbat = 0.0f;
#ifdef USE_5_DOF
uint32_t current_control_id = 0;
uint32_t start_pwm_id = 0;
uint32_t stop_pwm_id = 0;
#endif

#ifdef USE_1_DOF
float vbat = 5.0f;
float shunt = 0.0f;
float airgap = 0.0f;
float curr_pwm_duty_cycle = 0.0f;
#elif defined(USE_5_DOF)
float lpu_v`at[10] = {5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f};
float lpu_shunt[10] = {0.0f};
float lpu_pwm_duty[10] = {0.0f};
float airgap_measurements[8] = {0.0f};
#endif

inline void reset_slave();

// ============================================
// Master-Specific SPI Callbacks
// ============================================
struct MasterSPICallbacks {
    static void on_prepare_tx() {
        // Just package the current control state into the frame
        // All order processing and parameter updates happened before this callback
        // This ensures MDMA copies consistent, fully-initialized values
    }

    static void on_spi_start() {
        // Called when SPI transfer initiates
        // Slave ready check already handled in update()
    }

    static void on_spi_complete() {
        // Called after SPI transfer completes (before frame validation)
    }

    static void on_data_received() {
        // Extract received LPU and Airgap data
        last_spi_packet_ms = HAL_GetTick();
        spi_connected = true;

#ifdef USE_1_DOF
        vbat = LCU_Master::lpu_array->get_lpu<0>().vbat_v;
        shunt = LCU_Master::lpu_array->get_lpu<0>().shunt_v;
        airgap = LCU_Master::airgap_array->get_airgap<0>().airgap_v * 1000.0f;
        curr_pwm_duty_cycle = LCU_Master::lpu_array->get_lpu<0>().duty_cycle;
#elif defined(USE_5_DOF)
        for (size_t i = 0; i < 10; ++i) {
            lpu_vbat[i] = LCU_Master::lpu_array->get_lpu<i>().vbat_v;
            lpu_shunt[i] = LCU_Master::lpu_array->get_lpu<i>().shunt_v;
            lpu_pwm_duty[i] = LCU_Master::lpu_array->get_lpu<i>().duty_cycle;
        }
        for (size_t i = 0; i < 8; ++i) {
            airgap_measurements[i] = LCU_Master::airgap_array->get_airgap<i>().airgap_v * 1000.0f;
        }
#endif
    }

    static void on_frame_error() {
        // Frame validation failed - log and reset
        spi_connected = false;
    }
};

// SPI Communication Logic Instance (flags control error handling and timeout)
using MasterSPILogic = SharedSPICommunicationLogic<
    LCU_Master::CommsFrame,
    ST_LIB::SPIDomain::SPIWrapper<LCU_Master::spi_req>,
    MasterSPICallbacks,
    true,                       // IsMaster
    ENABLE_SPI_ERROR_HANDLING,  // EnableErrorHandling (from Flags.hpp)
    10,                         // MaxErrors
    ENABLE_SPI_TIMEOUT,         // EnableTimeout (from Flags.hpp)
    1000                        // TimeoutMs
>;

inline MasterSPILogic spi_logic;

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
#elif defined(USE_5_DOF)
    OrderPackets::Current_Control_init(desired_current, current_control_id);
    OrderPackets::Start_PWM_init(pwm_duty_cycle, start_pwm_id);
    OrderPackets::Stop_PWM_init(stop_pwm_id);
#endif
    OrderPackets::Set_Fixed_VBAT_init(fixed_vbat);
    OrderPackets::Unset_Fixed_VBAT_init();
    OrderPackets::Set_Control_Params_init();
    OrderPackets::Reset_Master_init();
    OrderPackets::Reset_Slave_init();
    OrderPackets::Reset_All_init();

    // Initialize Data Packets
#ifdef USE_1_DOF
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
    
    // Initialize SPI Communication Logic
    spi_logic.init(g_spi);
}

inline bool is_connected() {
#ifdef STLIB_ETH
    g_eth->update();
    return OrderPackets::control_station_tcp->is_connected() && spi_connected;
#else
    return spi_connected;
#endif
}

inline void update() {
#ifdef STLIB_ETH
    g_eth->update();
#endif


    if (OrderPackets::Stop_All_flag) {
        control.control_packet.mode = ControlMode::NONE;
        LCU_Master::lpu_array->clear_all_fixed_duty_cycle();
        LCU_Master::lpu_array->clear_all_fixed_vbat();
        levitating_state = false;
        current_control_active = false;
        fixed_pwm_active = 0;
        OrderPackets::Stop_All_flag = false;
    }

    if (OrderPackets::Levitate_flag) {
        control.control_packet.distance_control.desired_distance = desired_levitation_distance / 1000.0f; // Convert mm to m
        control.control_packet.mode = ControlMode::DISTANCE_CONTROL;
        levitating_state = true;
        OrderPackets::Levitate_flag = false;
    }

    if (OrderPackets::Stop_Levitate_flag) {
        control.control_packet.mode = ControlMode::NONE;
        levitating_state = false;
        OrderPackets::Stop_Levitate_flag = false;
    }

    if (OrderPackets::Current_Control_flag) {
        control.control_packet.current_control.desired_current = desired_current;
        control.control_packet.mode = ControlMode::CURRENT_CONTROL;
        current_control_active = true;
        OrderPackets::Current_Control_flag = false;
    }

    if (OrderPackets::Start_PWM_flag) {
#ifdef USE_1_DOF
        LCU_Master::lpu_array->set_indexed_fixed_duty_cycle(0, pwm_duty_cycle);
        fixed_pwm_active = 1; // Set bitmask to indicate fixed PWM active on LPU 1
#else
        if (start_pwm_id >= 1 && start_pwm_id <= 10) {
            LCU_Master::lpu_array->set_indexed_fixed_duty_cycle(start_pwm_id - 1, pwm_duty_cycle);
            fixed_pwm_active |= (1 << (start_pwm_id - 1)); // Set bit for this LPU
        }
#endif
        OrderPackets::Start_PWM_flag = false;
    }

    if (OrderPackets::Stop_PWM_flag) {
#ifdef USE_1_DOF
        LCU_Master::lpu_array->clear_indexed_fixed_duty_cycle(0);
        fixed_pwm_active = 0; // Clear bitmask to indicate no fixed PWM active
#else
        if (stop_pwm_id >= 1 && stop_pwm_id <= 10) {
            LCU_Master::lpu_array->clear_indexed_fixed_duty_cycle(stop_pwm_id - 1);
            fixed_pwm_active &= ~(1 << (stop_pwm_id - 1)); // Clear bit for this LPU
        }
#endif
        OrderPackets::Stop_PWM_flag = false;
    }

    if (OrderPackets::Set_Fixed_VBAT_flag) {
        LCU_Master::lpu_array->set_all_fixed_vbat(fixed_vbat);
        OrderPackets::Set_Fixed_VBAT_flag = false;
    }

    if (OrderPackets::Unset_Fixed_VBAT_flag) {
        LCU_Master::lpu_array->clear_all_fixed_vbat();
        OrderPackets::Unset_Fixed_VBAT_flag = false;
    }

    if (OrderPackets::Set_Control_Params_flag) {
        // TODO
        OrderPackets::Set_Control_Params_flag = false;
    }

    if (OrderPackets::Reset_Master_flag) {
        HAL_NVIC_SystemReset();
    }

    if (OrderPackets::Reset_Slave_flag) {
        reset_slave();
        OrderPackets::Reset_Slave_flag = false;
    }

    if (OrderPackets::Reset_All_flag) {
        HAL_NVIC_SystemReset();
    }

    // =========================================
    // Handle Slave Ready Signal from EXTI
    // =========================================
    // When EXTI signals slave is ready, notify the SPI logic
    if (LCU_Master::slave_ready_triggered && spi_logic.waiting_for_ready) {
        LCU_Master::slave_ready_triggered = false;
        spi_logic.ready_for_transfer();
    }

    // =========================================
    // SPI Communication via Shared Logic
    // =========================================
    // All state machine, callbacks, and frame validation handled by SharedSPICommunicationLogic
    spi_logic.update();
}
}; // namespace Comms

#endif // COMMUNICATIONS_HPP
