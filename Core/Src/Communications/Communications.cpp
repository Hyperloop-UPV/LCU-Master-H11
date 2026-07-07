#include "Communications/Communications.hpp"
#include "LCU_MASTER.hpp"
#include "StateMachine/LCU_StateMachine.hpp"

namespace Communications {

// ============================================
// Variables for Ethernet data exchange
// ============================================

float desired_levitation_distance = 0.0f;
float desired_current = 0.0f;
float pwm_duty_cycle = 0.0f;
float fixed_vbat = 0.0f;
uint32_t enable_buffer_id = 0;
uint32_t disable_buffer_id = 0;
#if defined(USE_5_DOF) || defined(USE_3_DOF)
uint32_t lpu_id = 0;
#endif

float lpu_vbat[10] = {0.0f};
float lpu_shunt[10] = {0.0f};
float lpu_pwm_duty[10] = {0.0f};
float airgap_measurements[8] = {0.0f};

float target_distance = 0.0f;
float desired_currents[4] = {0.0f};
float state[5] = {0.0f};
float local_airgaps[4] = {0.0f};

float Fe[3] = {0.0f};
float Fa[4] = {0.0f};
float Ef[3] = {0.0f};
float P[3] = {0.0f};
float R[3] = {0.0f};
float Zz[3] = {0.0f};
float Fe_L[3] = {0.0f};

float desired_voltages[4] = {0.0f};

float A[8] = {0.0f};
float Ak[4] = {0.0f};
float Bk[3] = {0.0f};

auto slave_state = DataPackets::slave_state_machine::SPI_Connecting;

float cinema_current = 0.0f;

// ============================================
// SPI Communications
// ============================================

void on_error() {
    if (LCU_Master::spi.was_aborted()) {
        LCU_Master::spi.clear_abort_flag();
    }
}

using SpiComms = SpiCommunications<
    LCU_Master::spi,
    LCU_Master::Frame,
    +[]() { // SpiReady
        return LCU_Master::slave_ready_triggered == true;
    },
    +[]() { // OnTx
        // Master initiates the transfer
        LCU_Master::slave_ready_triggered = false;
    },
    +[]() { // OnRxReceived
        // Transfer complete
    },
    +[]() { // OnRxValid
        read_slave_data();
    },
    +[]() { // OnRxInvalid
        on_error();
    },
    +[]() { // OnTimeout
        on_error();
    },
    +[]() { // OnMaxErrors
        FAULT("Maximum SPI error count exceeded");
    },
    1,  // max_errors (0 = disabled)
    10000>; // spi_timeout_limit (0 = disabled)

SpiComms spi_comms{};

// ============================================
// Implementation
// ============================================

void reset_slave() {
    is_resetting_slave = true;
    for (int i = 0; i < 5; i++) {
        LCU_Master::master_fault.turn_off();
        HAL_Delay(1);
        LCU_Master::master_fault.turn_on();
        HAL_Delay(1);
    }
    HAL_Delay(200);
    is_resetting_slave = false;
}

void init() {
    reset_slave();

    // Initialize Orders (invariant across DOFs)
    OrderPackets::Stop_init();
    OrderPackets::Set_Fixed_VBAT_init(fixed_vbat);
    OrderPackets::Unset_Fixed_VBAT_init();
    OrderPackets::Levitate_init(desired_levitation_distance);
    OrderPackets::Levitate_Ramp_init(desired_levitation_distance);
    OrderPackets::Set_Desired_Distance_init(desired_levitation_distance);
    OrderPackets::Set_Desired_Distance_Ramp_init(desired_levitation_distance);
    OrderPackets::Stop_Ramp_init();
    OrderPackets::All_Current_Control_init(desired_current);
    OrderPackets::All_PWM_init(pwm_duty_cycle);
    OrderPackets::Reset_Slave_init();
    OrderPackets::Reset_All_init();
    OrderPackets::Cinema_init(cinema_current);
    OrderPackets::Stop_Cinema_init();

    // Orders with lpu_id (multi-DOF) vs without (1-DOF)
#if defined(USE_1_DOF)
    #define LPU_ID_COMMA
    #define LPU_ID_NONE
#else
    #define LPU_ID_COMMA lpu_id,
    #define LPU_ID_NONE lpu_id
#endif

    OrderPackets::Current_Control_init(LPU_ID_COMMA desired_current);
    OrderPackets::Stop_Current_Control_init(LPU_ID_NONE);
    OrderPackets::PWM_init(LPU_ID_COMMA pwm_duty_cycle);
    OrderPackets::Stop_PWM_init(LPU_ID_NONE);

    // Per-DOF argument unrolling for data packet init calls
#if defined(USE_5_DOF)
    #define ARGS_LPU(arr)   arr[0], arr[1], arr[2], arr[3], arr[4], arr[5], arr[6], arr[7], arr[8], arr[9]
    #define ARGS_AIRGAP(arr) arr[0], arr[1], arr[2], arr[3], arr[4], arr[5], arr[6], arr[7]
#elif defined(USE_3_DOF)
    #define ARGS_LPU(arr)   arr[0], arr[1], arr[2], arr[3]
    #define ARGS_AIRGAP(arr) arr[0], arr[1], arr[2], arr[3]
#else
    #define ARGS_LPU(arr)   arr[0]
    #define ARGS_AIRGAP(arr) arr[0]
#endif

    DataPackets::LPU_PWM_duties_init(ARGS_LPU(lpu_pwm_duty));
    DataPackets::LPU_coil_currents_init(ARGS_LPU(lpu_shunt));
    DataPackets::LPU_VBATs_init(ARGS_LPU(lpu_vbat));
    DataPackets::Airgaps_init(ARGS_AIRGAP(airgap_measurements));

    #undef ARGS_LPU
    #undef ARGS_AIRGAP
    #undef LPU_ID_COMMA
    #undef LPU_ID_NONE
    DataPackets::State_Machine_init(
        master_state_machine_state,
        slave_state
    );
    DataPackets::General_State_init(
        target_distance,
        desired_currents[0], desired_currents[1], desired_currents[2], desired_currents[3],
        state[0], state[1], state[2], state[3], state[4],
        local_airgaps[0], local_airgaps[1], local_airgaps[2], local_airgaps[3],
        desired_voltages[0], desired_voltages[1], desired_voltages[2], desired_voltages[3],
        Fe[0], Fe[1], Fe[2],
        Fa[0], Fa[1], Fa[2], Fa[3],
        Ef[0], Ef[1], Ef[2],
        P[0], P[1], P[2],
        R[0], R[1], R[2],
        Zz[0], Zz[1], Zz[2],
        Fe_L[0], Fe_L[1], Fe_L[2],
        A[0], A[1], A[2], A[3], A[4], A[5], A[6], A[7],
        Ak[0], Ak[1], Ak[2], Ak[3],
        Bk[0], Bk[1], Bk[2]
    );

    DataPackets::start();
    OrderPackets::start();
}

bool is_connected() {
    return spi_comms.is_connected() && LCU_Master::eth.is_connected() && OrderPackets::control_station_tcp->is_connected();
}

void clear_flags() {
    OrderPackets::Stop_flag = false;
    OrderPackets::Set_Fixed_VBAT_flag = false;
    OrderPackets::Unset_Fixed_VBAT_flag = false;
    OrderPackets::Levitate_flag = false;
    OrderPackets::Levitate_Ramp_flag = false;
    OrderPackets::Set_Desired_Distance_flag = false;
    OrderPackets::Set_Desired_Distance_Ramp_flag = false;
    OrderPackets::Stop_Ramp_flag = false;
    OrderPackets::Current_Control_flag = false;
    OrderPackets::All_Current_Control_flag = false;
    OrderPackets::Stop_Current_Control_flag = false;
    OrderPackets::PWM_flag = false;
    OrderPackets::All_PWM_flag = false;
    OrderPackets::Stop_PWM_flag = false;
    OrderPackets::Reset_All_flag = false;
    OrderPackets::Reset_Slave_flag = false;
    OrderPackets::Cinema_flag = false;
    OrderPackets::Stop_Cinema_flag = false;
}

void process_orders() {
    if (OrderPackets::Stop_flag) {
        LCU_SM::slave_state_machine.desired_state = SlaveState::IDLE;
        operational_state = false;
        control.input.ramping = false;
        control.input.RefCurrent = 0.0f;
        control.input.RefZ = 0.0f;
        control.input.cinema = false;
        control.input.cinema_current = 0.0f;
        LCU_SM::slave_state_machine.lpu_bitmask = 0;
        LCU_Master::lpu_array.set_fixed_duty_cycle_all(0.0f);
    }

    if (OrderPackets::Set_Fixed_VBAT_flag) {
        LCU_Master::lpu_array.set_fixed_vbat_all(fixed_vbat);
    }

    if (OrderPackets::Unset_Fixed_VBAT_flag) {
        LCU_Master::lpu_array.unset_fixed_vbat_all();
    }

    if (OrderPackets::Levitate_flag) {
        LCU_SM::slave_state_machine.desired_state = SlaveState::LEVITATION;
        operational_state = true;
        control.input.ramping = false;
        control.input.RefZ = desired_levitation_distance;
    }

    if (OrderPackets::Levitate_Ramp_flag) {
        LCU_SM::slave_state_machine.desired_state = SlaveState::LEVITATION;
        operational_state = true;
        control.input.ramping = true;
        control.input.RefZ = desired_levitation_distance;
    }

    if (OrderPackets::Set_Desired_Distance_flag) {
        control.input.RefZ = desired_levitation_distance;
    }

    if (OrderPackets::Set_Desired_Distance_Ramp_flag) {
        control.input.ramping = true;
        control.input.RefZ = desired_levitation_distance;
    }

    if (OrderPackets::Stop_Ramp_flag) {
        control.input.ramping = false;
    }

    if (OrderPackets::Current_Control_flag) {
#if defined(USE_5_DOF) || defined(USE_3_DOF)
        if (lpu_id < 1 || lpu_id > LCUConfig::MAX_LPU_COUNT) {
            WARNING("Invalid LPU ID in Current Control Order");
            return;
        }
        LCU_SM::slave_state_machine.lpu_bitmask |= 1 << (lpu_id - 1);
#else
        LCU_SM::slave_state_machine.lpu_bitmask = 1;
#endif
        LCU_SM::slave_state_machine.desired_state = SlaveState::CURRENT_CONTROL;
        operational_state = true;
        control.input.ramping = false;
        control.input.RefCurrent = desired_current;
    }

    if (OrderPackets::All_Current_Control_flag) {
        LCU_SM::slave_state_machine.lpu_bitmask = (1U << LCUConfig::ACTIVE_LPU_COUNT) - 1;
        LCU_SM::slave_state_machine.desired_state = SlaveState::CURRENT_CONTROL;
        operational_state = true;
        control.input.ramping = false;
        control.input.RefCurrent = desired_current;
    }

    if (OrderPackets::Stop_Current_Control_flag) {
#if defined(USE_5_DOF) || defined(USE_3_DOF)
        if (lpu_id < 1 || lpu_id > LCUConfig::MAX_LPU_COUNT) {
            WARNING("Invalid LPU ID in Stop Current Control Order");
            return;
        }
        LCU_SM::slave_state_machine.lpu_bitmask &= ~(1 << (lpu_id - 1));
        if (LCU_SM::slave_state_machine.lpu_bitmask == 0) {
            LCU_SM::slave_state_machine.desired_state = SlaveState::IDLE;
            operational_state = false;
        }
#else
        LCU_SM::slave_state_machine.desired_state = SlaveState::IDLE;
        operational_state = false;
#endif
    }

    if (OrderPackets::PWM_flag) {
#if defined(USE_5_DOF) || defined(USE_3_DOF)
        if (lpu_id < 1 || lpu_id > LCUConfig::MAX_LPU_COUNT) {
            WARNING("Invalid LPU ID in PWM Order");
            return;
        }
        LCU_Master::lpu_array.set_fixed_duty_cycle_to(pwm_duty_cycle, lpu_id - 1);
        LCU_SM::slave_state_machine.lpu_bitmask |= 1 << (lpu_id - 1);
#else
        LCU_Master::lpu_array.set_fixed_duty_cycle_to(pwm_duty_cycle, 0);
        LCU_SM::slave_state_machine.lpu_bitmask = 1;
#endif
        LCU_SM::slave_state_machine.desired_state = SlaveState::DEBUG;
        operational_state = true;
    }

    if (OrderPackets::All_PWM_flag) {
        LCU_Master::lpu_array.set_fixed_duty_cycle_all(pwm_duty_cycle);
        LCU_SM::slave_state_machine.lpu_bitmask = (1U << LCUConfig::ACTIVE_LPU_COUNT) - 1;
        LCU_SM::slave_state_machine.desired_state = SlaveState::DEBUG;
        operational_state = true;
    }

    if (OrderPackets::Stop_PWM_flag) {
#if defined(USE_5_DOF) || defined(USE_3_DOF)
        if (lpu_id < 1 || lpu_id > LCUConfig::MAX_LPU_COUNT) {
            WARNING("Invalid LPU ID in Stop PWM Order");
            return;
        }
        LCU_Master::lpu_array.set_fixed_duty_cycle_to(0.0f, lpu_id - 1);
        LCU_SM::slave_state_machine.lpu_bitmask &= ~(1 << (lpu_id - 1));
        if (LCU_SM::slave_state_machine.lpu_bitmask == 0) {
            LCU_SM::slave_state_machine.desired_state = SlaveState::IDLE;
            operational_state = false;
        }
#else
        LCU_Master::lpu_array.set_fixed_duty_cycle_to(0.0f, 0);
        LCU_SM::slave_state_machine.desired_state = SlaveState::IDLE;
        operational_state = false;
#endif
    }

    if (OrderPackets::Reset_Slave_flag) {
        reset_slave();
    }

    if (OrderPackets::Reset_All_flag) {
        HAL_NVIC_SystemReset();
    }

    if (OrderPackets::Cinema_flag) {
        control.input.cinema = true;
        control.input.cinema_current = cinema_current;
    }

    if (OrderPackets::Stop_Cinema_flag) {
        control.input.cinema = false;
        control.input.cinema_current = 0.0f;
    }
}

void read_slave_data() {
    // LPU data is synced via Frame (LPUBase::get_uplink_layout -> vbat_v, shunt_v, duty_cycle)
    // The Frame automatically populates these fields on the Master's LPU objects
    auto vbats = LCU_Master::lpu_array.get_all_vbat();
    auto shunts = LCU_Master::lpu_array.get_all_shunt();
    auto duty_cycles = LCU_Master::lpu_array.get_all_duty_cycle();
    for (size_t i = 0; i < LCUConfig::ACTIVE_LPU_COUNT; i++) {
        lpu_vbat[i] = vbats[i];
        lpu_shunt[i] = shunts[i];
        lpu_pwm_duty[i] = duty_cycles[i];
    }

    // Airgap data synced via Frame
    auto airgaps = LCU_Master::airgap_array.get_all_airgap();
    for (size_t i = 0; i < LCUConfig::ACTIVE_AIRGAP_COUNT; i++) {
        airgap_measurements[i] = airgaps[i];
    }

    // Control outputs synced via Frame (ControlBase::get_uplink_layout -> output)
    for (int i = 0; i < 4; i++) desired_voltages[i] = control.output.Voltages[i];
    for (int i = 0; i < 3; i++) Fe[i] = control.output.Fe[i];
    for (int i = 0; i < 4; i++) Fa[i] = control.output.Fa[i];
    for (int i = 0; i < 3; i++) Ef[i] = control.output.Ef[i];
    for (int i = 0; i < 3; i++) P[i] = control.output.P[i];
    for (int i = 0; i < 3; i++) R[i] = control.output.R[i];
    for (int i = 0; i < 3; i++) Zz[i] = control.output.Zz[i];
    for (int i = 0; i < 3; i++) Fe_L[i] = control.output.Fe_L[i];
    for (int i = 0; i < 8; i++) A[i] = control.output.A[i];
    for (int i = 0; i < 4; i++) Ak[i] = control.output.Ak[i];
    for (int i = 0; i < 3; i++) Bk[i] = control.output.Bk[i];
    target_distance = control.output.Referencia;
    for (int i = 0; i < 4; i++) desired_currents[i] = control.output.CorrienteReferencia[i];
    for (int i = 0; i < 5; i++) state[i] = control.output.Estados[i];
    for (int i = 0; i < 4; i++) local_airgaps[i] = control.output.GapsLocales[i];

    // Slave state synced via Frame (StateMachineBase::get_uplink_layout -> current_state)
    slave_state = static_cast<DataPackets::slave_state_machine>(
        LCU_SM::slave_state_machine.current_state
    );

    if (report.get_seq_num() != last_report_seq_num) {
        Diagnostics::Hub::publish(const_cast<const Diagnostics::DiagnosticRecord&>(report.get_record()));
        last_report_seq_num = report.get_seq_num();
    }
}

void update() {
    LCU_Master::eth.update();
    process_orders();
    clear_flags();

    spi_comms.update();
}

} // namespace Communications
