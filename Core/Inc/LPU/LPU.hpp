#ifndef LPU_HPP
#define LPU_HPP

#include "C++Utilities/CppImports.hpp"
#include "LPUShared.hpp"
#include "ST-LIB_LOW/DigitalInput2.hpp"

class LPU : public LPUBase {
   public:
    LPU(ST_LIB::DigitalInputDomain::Instance& ready, ST_LIB::DigitalInputDomain::Instance& fault) :
        ready_pin(ready), fault_pin(fault) {}

    bool update() {
        if (ready_pin.read() == GPIO_PinState::GPIO_PIN_SET) ready = true; else ready = false;
        if (fault_pin.read() == GPIO_PinState::GPIO_PIN_SET) fault = true; else fault = false;
        if (fault) {
            return false;
        }
        return true;
    }
    
   private:
    ST_LIB::DigitalInputDomain::Instance& ready_pin;
    ST_LIB::DigitalInputDomain::Instance& fault_pin;
};



template <typename LPUTuple, typename ResetPinTuple>
class LpuArray;

template <typename... LPUs, typename... ResetPins>
class LpuArray<std::tuple<LPUs...>, std::tuple<ResetPins...>> {    
    static constexpr size_t LpuCount = sizeof...(LPUs);
    static constexpr size_t PinCount = sizeof...(ResetPins);

    static_assert(LpuCount == PinCount * 2 || (LpuCount == 1 && PinCount == 1), "Configuration Error: Must have exactly 2 LPUs per Enable Pin or have only 1 LPU and 1 Enable Pin (1DOF).");

    using LPUPtrTuple = std::tuple<std::remove_reference_t<LPUs>*...>;
    using PinPtrTuple = std::tuple<std::remove_reference_t<ResetPins>*...>;

    LPUPtrTuple lpus;
    PinPtrTuple reset_pins;

    bool all_ok = true;

   public:
    LpuArray(std::tuple<LPUs&...> _lpus, std::tuple<ResetPins&...> _pins) {
        lpus = std::apply([](auto&... lpu) { return std::make_tuple(&lpu...); }, _lpus);
        reset_pins = std::apply([](auto&... pin) { return std::make_tuple(&pin...); }, _pins);
    }

    void reset_all() {
        std::apply([](auto&... pin) { (pin->turn_on(), ...); }, reset_pins);
    }

    template <size_t LpuIndex>
    void disable_pair() {
        if constexpr (LpuCount == 1) {
            std::get<0>(reset_pins)->turn_off();
            return;
        }
        constexpr size_t PinIndex = LpuIndex / 2;
        std::get<PinIndex>(reset_pins)->turn_on();
    }

    template <size_t LpuIndex>
    void enable_pair() {
        if constexpr (LpuCount == 1) {
            std::get<0>(reset_pins)->turn_on();
            return;
        }
        constexpr size_t PinIndex = LpuIndex / 2;
        std::get<PinIndex>(reset_pins)->turn_off();
    }

    void disable_all() {
        std::apply([](auto&... pin) { (pin->turn_on(), ...); }, reset_pins);
    }

    void enable_all() {
        std::apply([](auto&... pin) { (pin->turn_off(), ...); }, reset_pins);
    }

    bool update_all() {
        all_ok = true;
        std::apply([&](auto&... lpu) { ((all_ok &= lpu->update()), ...); }, lpus);
        return all_ok;
    }

    template <size_t Index>
    auto& get_lpu() {
        return *std::get<Index>(lpus);
    }

    bool is_all_ok() const {
        return all_ok;
    }
};

// Deduction guide for LpuArray
template <typename... LPUs, typename... ResetPins>
LpuArray(std::tuple<LPUs&...>, std::tuple<ResetPins&...>) -> LpuArray<std::tuple<LPUs...>, std::tuple<ResetPins...>>;



#endif // LPU_HPP