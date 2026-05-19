#ifndef COMMUNICATIONS_HPP
#define COMMUNICATIONS_HPP

#include "ConfigShared.hpp"
#include "SpiCommunications.hpp"
#include "ReportShared.hpp"
#include "Communications/Packets/DataPackets.hpp"
#include "Communications/Packets/OrderPackets.hpp"

namespace Communications {

inline ReportBase report{};
inline ControlBase control{};
inline uint32_t last_report_seq_num = 0;
inline bool operational_state = false;
inline auto master_state_machine_state = DataPackets::master_state_machine::Connecting;
inline volatile bool is_resetting_slave = false;

void init();
void update();
bool is_connected();
void reset_slave();
void read_slave_data();

} // namespace Communications

#endif // COMMUNICATIONS_HPP
