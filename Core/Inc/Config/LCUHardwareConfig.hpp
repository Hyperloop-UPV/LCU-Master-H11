#ifndef LCU_HARDWARE_CONFIG_HPP
#define LCU_HARDWARE_CONFIG_HPP

#include "C++Utilities/CppImports.hpp"
#include "ST-LIB.hpp"

namespace LCUConfig {

// ============================================
// DOF Configuration Selection
// ============================================

enum class DOFConfig : uint8_t { DOF_1 = 1, DOF_3 = 3, DOF_5 = 5 };

#if defined(USE_5_DOF)
constexpr DOFConfig ACTIVE_DOF = DOFConfig::DOF_5;
#elif defined(USE_3_DOF)
constexpr DOFConfig ACTIVE_DOF = DOFConfig::DOF_3;
#else
constexpr DOFConfig ACTIVE_DOF = DOFConfig::DOF_1;
#endif

// ============================================
// Hardware Constants (always defined)
// ============================================

constexpr uint8_t MAX_LPU_COUNT = 10;
constexpr uint8_t MAX_AIRGAP_COUNT = 8;
constexpr uint8_t MAX_RST_PIN_COUNT = 5;

// ============================================
// DOF-specific active counts
// ============================================

constexpr uint8_t ACTIVE_LPU_COUNT = []() {
    if constexpr (ACTIVE_DOF == DOFConfig::DOF_5)
        return 10;
    else if constexpr (ACTIVE_DOF == DOFConfig::DOF_3)
        return 4;
    else
        return 1;
}();

constexpr uint8_t ACTIVE_AIRGAP_COUNT = []() {
    if constexpr (ACTIVE_DOF == DOFConfig::DOF_5)
        return 8;
    else if constexpr (ACTIVE_DOF == DOFConfig::DOF_3)
        return 4;
    else
        return 1;
}();

// ============================================
// DOF-specific connector mappings
// Maps virtual index (0..N-1) to physical connector index
// ============================================

constexpr auto lpu_virtual_to_connector(uint8_t virtual_idx) {
    if constexpr (ACTIVE_DOF == DOFConfig::DOF_5) {
        if (virtual_idx >= 10) {
            ST_LIB::compile_error("Bad LPU virtual_idx");
        }
        return virtual_idx; // 1:1 mapping for 5-DOF

    } else if constexpr (ACTIVE_DOF == DOFConfig::DOF_3) {
        switch (virtual_idx) {
        case 0:
            return 9;
            break;
        case 1:
            return 3;
            break;
        case 2:
            return 7;
            break;
        case 3:
            return 5;
            break;
        default:
            ST_LIB::compile_error("Bad LPU virtual_idx");
        }

    } else {
        if (virtual_idx != 0) {
            ST_LIB::compile_error("Bad LPU virtual_idx");
        }
        return 0;
    }
}

// Airgap virtual -> physical mapping
constexpr auto airgap_virtual_to_connector(uint8_t virtual_idx) {
    if constexpr (ACTIVE_DOF == DOFConfig::DOF_5) {
        if (virtual_idx >= 8) {
            ST_LIB::compile_error("Bad airgap virtual_idx");
        }
        return virtual_idx;
    } else if constexpr (ACTIVE_DOF == DOFConfig::DOF_3) {
        switch (virtual_idx) {
        case 0:
            return 1;
            break;
        case 1:
            return 4;
            break;
        case 2:
            return 3;
            break;
        case 3:
            return 2;
            break;
        default:
            ST_LIB::compile_error("Bad LPU virtual_idx");
        }

    } else {
        return virtual_idx;
    }
}

} // namespace LCUConfig

#endif // LCU_HARDWARE_CONFIG_HPP
