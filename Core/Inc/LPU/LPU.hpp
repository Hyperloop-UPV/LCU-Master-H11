#ifndef LPU_HPP
#define LPU_HPP

#include "C++Utilities/CppImports.hpp"
#include "LPUShared.hpp"
#include "ST-LIB_LOW/DigitalInput2.hpp"

class LPU : public LPUBase {
public:
    LPU(ST_LIB::DigitalInputDomain::Instance& ready, ST_LIB::DigitalInputDomain::Instance& fault)
        : ready_pin(ready), fault_pin(fault) {}

    void update() {
        return; // Bypass
        auto ready = (ready_pin.read() == GPIO_PinState::GPIO_PIN_SET);
        auto fault = (fault_pin.read() == GPIO_PinState::GPIO_PIN_SET);

        if (!ready) {
            FAULT("LPU not ready");
        }
        if (fault) {
            FAULT("LPU fault detected");
        }
    }

    void set_fixed_duty_cycle(float duty) {
        fixed_duty_cycle = duty;
    }

private:
    ST_LIB::DigitalInputDomain::Instance& ready_pin;
    ST_LIB::DigitalInputDomain::Instance& fault_pin;
};

template <typename LPUTuple, typename ResetPinTuple> class LpuArray;

template <typename... LPUs, typename... ResetPins>
class LpuArray<std::tuple<LPUs...>, std::tuple<ResetPins...>> : public LpuArrayBase<std::tuple<LPUs...>> {
    std::tuple<ResetPins...>& reset_pins;

public:
    explicit LpuArray(std::tuple<LPUs...>& lpu_refs, std::tuple<ResetPins...>& reset_pin_refs)
        : LpuArrayBase<std::tuple<LPUs...>>(lpu_refs), reset_pins(reset_pin_refs) {}

    void enable_all() {
        std::apply([](auto&... pin) { (pin.turn_on(), ...); }, this->reset_pins);
    }
    void disable_all() {
        std::apply([](auto&... pin) { (pin.turn_off(), ...); }, this->reset_pins);
    }

    void update_all() {
        std::apply([&](auto&... lpu) { ((lpu.update()), ...); }, this->lpus);
    }

    std::array<float, sizeof...(LPUs)> get_all_vbat() {
        std::array<float, sizeof...(LPUs)> vbats;
        std::apply([&](auto&... lpu) { ((vbats[&lpu - &std::get<0>(this->lpus)] = lpu.vbat_v), ...); }, this->lpus);
        return vbats;
    }

    std::array<float, sizeof...(LPUs)> get_all_shunt() {
        std::array<float, sizeof...(LPUs)> shunts;
        std::apply([&](auto&... lpu) { ((shunts[&lpu - &std::get<0>(this->lpus)] = lpu.shunt_v), ...); }, this->lpus);
        return shunts;
    }

    std::array<float, sizeof...(LPUs)> get_all_duty_cycle() {
        std::array<float, sizeof...(LPUs)> duty_cycles;
        std::apply([&](auto&... lpu) { ((duty_cycles[&lpu - &std::get<0>(this->lpus)] = lpu.duty_cycle), ...); }, this->lpus);
        return duty_cycles;
    }

    void set_fixed_vbat_all(float vbat) {
        std::apply([&](auto&... lpu) { ((lpu.is_fixed_vbat = true, lpu.fixed_vbat = vbat), ...); }, this->lpus);
    }

    void unset_fixed_vbat_all() {
        std::apply([&](auto&... lpu) { ((lpu.is_fixed_vbat = false, lpu.fixed_vbat = 0.0f), ...); }, this->lpus);
    }

    void set_fixed_duty_cycle_all(float duty_cycle) {
        std::apply([&](auto&... lpu) { ((lpu.set_fixed_duty_cycle(duty_cycle)), ...); }, this->lpus);
    }

    void set_fixed_duty_cycle_to(float duty_cycle, size_t idx) {
        auto set_at = [&](auto seq) {
            [&]<size_t... I>(std::index_sequence<I...>) {
                ((I == idx
                      ? (void)(std::get<I>(this->lpus).set_fixed_duty_cycle(duty_cycle))
                      : (void)0),
                 ...);
            }(seq);
        };

        set_at(std::index_sequence_for<LPUs...>{});
    }
};

template <typename... LPUs, typename... ResetPins>
LpuArray(std::tuple<LPUs...>&, std::tuple<ResetPins...>&)
    -> LpuArray<std::tuple<LPUs...>, std::tuple<ResetPins...>>;

#endif // LPU_HPP
