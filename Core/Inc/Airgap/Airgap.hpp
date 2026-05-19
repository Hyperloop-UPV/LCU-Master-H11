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

    auto get_all_airgap() const {
        return [this]<size_t... Is>(std::index_sequence<Is...>) {
            return std::array<float, sizeof...(Is)>{std::get<Is>(this->airgaps).airgap_v...};
        }(std::make_index_sequence<sizeof...(AirgapInstances)>{});
    }
};

#endif // AIRGAP_HPP
