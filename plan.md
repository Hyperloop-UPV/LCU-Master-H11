# LCU-Master Refactoring Plan

## Goal
Refactor LCU-Master to match LCU-Slave's architecture, using the same Shared base classes and Frame-based SPI sync.

## Architecture Mapping

### What Stays the Same (Master-specific)
- **Pinout**: Master-specific pins (LPU fault/ready/rst, no PWM/ADC)
- **Ethernet**: Code_generation packets (DataPackets/OrderPackets) for control station
- **StateMachine behavior**: Connecting/Operational/Idle/Levitating states

### What Gets Refactored (to match Slave pattern)
| File | Old | New |
|------|-----|-----|
| `LCU_MASTER_TYPES.hpp` | Separate types file | Merged into `LCU_MASTER.hpp` |
| `LCU_MASTER.hpp` | Includes types, inline init/update | Single header with all hardware declarations |
| `LCU_MASTER.cpp` | (doesn't exist) | init/update implementation |
| `LPU.hpp` | Simple class, no base | Inherits LPUBase, LpuArray with layout methods |
| `Airgap.hpp` | Simple class, no base | Inherits AirgapBase, AirgapArray with layout methods |
| `Communications.hpp` | 743-line monolith, old SystemFrame | Split into hpp/cpp, use SpiCommunications from Shared |
| `StateMachine.hpp` | Uses DataPackets enums only | Add StateMachineBase for Frame sync |
| `main.cpp` | Direct LCU_Master calls | Same pattern as Slave |

### Shared Base Classes Used
- `StateMachineBase` - bidirectional state sync
- `ControlBase` - Master sends RefZ/RefCurrent, receives control outputs
- `ReportBase` - receives diagnostics from Slave
- `LPUBase` - sends is_fixed_vbat/fixed_vbat/fixed_duty_cycle, receives vbat/shunt/duty
- `LpuArrayBase` - aggregates LPU layouts
- `AirgapBase` - receives airgap_v from Slave
- `AirgapArrayBase` - aggregates airgap layouts
- `SpiCommunications` - SPI protocol with callbacks
- `Frame<true, ...>` - Master-side MDMA frame

### Frame Template (Master side)
```cpp
using Frame = FrameType<true,
    decltype(state_machine),  // StateMachineBase
    decltype(control),        // ControlBase
    decltype(lpu_array),      // MasterLpuArray
    decltype(airgap_array),   // MasterAirgapArray
    decltype(report)          // ReportBase
>;
```

### Key Behavioral Differences from Slave
1. **SPI Role**: MASTER mode (initiates transfers)
2. **SPI Callbacks**: Uses EXTI-based slave_ready detection
3. **LPU**: No PWM/ADC, just digital I/O for fault/ready/rst
4. **Airgap**: Empty container (data comes from Slave via SPI)
5. **Control**: Doesn't run Simulink, just forwards RefZ/RefCurrent
6. **StateMachine**: Connecting->Operational->Idle/Levitating (simpler than Slave)
7. **Ethernet**: Extra layer for control station communication
8. **Slave reset**: Master toggles master_fault pin to reset Slave

### Implementation Order
1. Pinout.hpp (add inline)
2. Config/LCUHardwareConfig.hpp (create)
3. LPU.hpp (refactor with LPUBase)
4. Airgap.hpp (refactor with AirgapBase)
5. LCU_MASTER.hpp (consolidate hardware declarations)
6. LCU_MASTER.cpp (create with init/update)
7. Communications.hpp (refactor with SpiCommunications)
8. Communications.cpp (create)
9. StateMachine.hpp (add StateMachineBase)
10. StateMachine.cpp (adapt)
11. main.cpp (update)
12. CMakeLists.txt (add 3DOF, cleanup)
13. Delete LCU_MASTER_TYPES.hpp
14. Clean up unused files
