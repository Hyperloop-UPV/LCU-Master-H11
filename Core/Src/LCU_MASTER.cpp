#include "LCU_MASTER.hpp"

namespace LCU_Master {

void init() {
    Board::init();

    master_fault.turn_on();

    MDMA::start();

    LCU_SM::start();

    // Initialize Frame (order must match FrameType: lpu_array, airgap_array, state_machine, control, report)
    Frame::init(
        lpu_array,
        airgap_array,
        LCU_SM::slave_state_machine,
        Communications::control,
        Communications::report
    );

    Communications::init();
    Watchdog::watchdog_time = std::chrono::milliseconds(100);
    Watchdog::start();
    Diagnostics::install_ethernet_sink(OrderPackets::vcu_tcp);
    FaultController::register_fault_propagation(OrderPackets::vcu_tcp, OrderPackets::FAULT_order);
}

void update() {
    Communications::update();
    FaultController::check_transitions();
    LCU_SM::update();
    Scheduler::update();
    MDMA::update();
    Board::evaluate_protections();
    Diagnostics::Hub::flush();
    Watchdog::refresh();
}

} // namespace LCU_Master
