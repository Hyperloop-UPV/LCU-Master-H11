#ifndef LPU_HPP
#define LPU_HPP

#include "C++Utilities/CppImports.hpp"
#include "LPUShared.hpp"
#include "ST-LIB_LOW/DigitalInput2.hpp"

class LPU : public LPUBase {
public:
    LPU(ST_LIB::DigitalInputDomain::Instance& ready, ST_LIB::DigitalInputDomain::Instance& fault)
        : ready_pin(ready), fault_pin(fault) {}

    bool update() {
        return true; // Temporary bypass for testing without hardware. Replace with actual logic
                     // below.
        if (ready_pin.read() == GPIO_PinState::GPIO_PIN_SET)
            ready = true;
        else
            ready = false;
        if (fault_pin.read() == GPIO_PinState::GPIO_PIN_SET)
            fault = true;
        else
            fault = false;
        if (fault) {
            return false;
        }
        return true;
    }

private:
    ST_LIB::DigitalInputDomain::Instance& ready_pin;
    ST_LIB::DigitalInputDomain::Instance& fault_pin;
};

template <typename LPUTuple, typename ResetPinTuple> class LpuArray;

template <typename... LPUs, typename... ResetPins>
class LpuArray<std::tuple<LPUs...>, std::tuple<ResetPins...>> {
    static constexpr size_t LpuCount = sizeof...(LPUs);
    static constexpr size_t PinCount = sizeof...(ResetPins);

    static_assert(
        LpuCount == PinCount * 2 || (LpuCount == 1 && PinCount == 1),
        "Configuration Error: Must have exactly 2 LPUs per Enable Pin or have only 1 LPU and 1 "
        "Enable Pin (1DOF)."
    );

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
        std::apply([](auto&... pin) { (pin->turn_off(), ...); }, reset_pins);
    }

    template <size_t LpuIndex> void disable_pair() {
        if constexpr (LpuCount == 1) {
            std::get<0>(reset_pins)->turn_off();
            return;
        }
        constexpr size_t PinIndex = LpuIndex;
        std::get<PinIndex>(reset_pins)->turn_off();
    }

    template <size_t LpuIndex> void enable_pair() {
        if constexpr (LpuCount == 1) {
            std::get<0>(reset_pins)->turn_on();
            return;
        }
        constexpr size_t PinIndex = LpuIndex;
        std::get<PinIndex>(reset_pins)->turn_on();
    }

    void disable_all() {
        std::apply([](auto&... pin) { (pin->turn_off(), ...); }, reset_pins);
    }

    void enable_all() {
        std::apply([](auto&... pin) { (pin->turn_on(), ...); }, reset_pins);
    }

    bool update_all() {
        all_ok = true;
        std::apply([&](auto&... lpu) { ((all_ok &= lpu->update()), ...); }, lpus);
        return all_ok;
    }

    template <size_t Index> auto& get_lpu() { return *std::get<Index>(lpus); }

    void enable_pair(size_t lpu_index) {
        if constexpr (LpuCount == 1) {
            std::get<0>(reset_pins)->turn_on();
            return;
        }
        if (lpu_index >= LpuCount) return; // Out of bounds check
        size_t pin_index = lpu_index;
        apply_to_pin(pin_index, [](auto pin) { pin->turn_on(); });
    }

    void disable_pair(size_t lpu_index) {
        if constexpr (LpuCount == 1) {
            std::get<0>(reset_pins)->turn_off();
            return;
        }
        if (lpu_index >= LpuCount) return; // Out of bounds check
        size_t pin_index = lpu_index;
        apply_to_pin(pin_index, [](auto pin) { pin->turn_off(); });
    }

    bool is_all_ok() const { return all_ok; }

private:
    template <typename Func> void apply_to_pin(size_t pin_index, Func&& func) {
        apply_to_pin_impl(pin_index, std::forward<Func>(func), std::index_sequence_for<ResetPins...>{});
    }

    template <typename Func, size_t... Is>
    void apply_to_pin_impl(size_t pin_index, Func&& func, std::index_sequence<Is...>) {
        (void)pin_index;
        (void)func;
        (((Is == pin_index) && (func(std::get<Is>(reset_pins)), true)) || ...);
    }
};

// Deduction guide for LpuArray
template <typename... LPUs, typename... ResetPins>
LpuArray(std::tuple<LPUs&...>, std::tuple<ResetPins&...>)
    -> LpuArray<std::tuple<LPUs...>, std::tuple<ResetPins...>>;

#endif // LPU_HPP
