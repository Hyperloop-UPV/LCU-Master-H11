#ifndef AIRGAP_HPP
#define AIRGAP_HPP

#include "C++Utilities/CppImports.hpp"
#include "AirgapShared.hpp"

class Airgap : public AirgapBase {
public:
    Airgap() = default;
};

template <typename AirgapTuple> class AirgapArray;

template <typename... AirgapInstances>
class AirgapArray<std::tuple<AirgapInstances...>>
    : public AirgapArrayBase<std::tuple<AirgapInstances...>> {
public:
    explicit AirgapArray(std::tuple<AirgapInstances...>& instance_refs)
        : AirgapArrayBase<std::tuple<AirgapInstances...>>(instance_refs) {}

    std::array<float, sizeof...(AirgapInstances)> get_all_airgap() {
        std::array<float, sizeof...(AirgapInstances)> airgaps;
        std::apply([&](auto&... airgap) { ((airgaps[&airgap - &std::get<0>(this->airgaps)] = airgap.airgap_v), ...); }, this->airgaps);
        return airgaps;
    }
};

#endif // AIRGAP_HPP
