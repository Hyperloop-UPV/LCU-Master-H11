#ifndef AIRGAP_HPP
#define AIRGAP_HPP

#include "C++Utilities/CppImports.hpp"
#include "AirgapShared.hpp"

class Airgap : public AirgapBase {
public:
    Airgap() {}
};

template <typename AirgapTuple> class AirgapArray;

template <typename... AirgapInstances>
class AirgapArray<std::tuple<AirgapInstances...>> {
    static constexpr size_t AirgapCount = sizeof...(AirgapInstances);

    using AirgapPtrTuple = std::tuple<std::remove_reference_t<AirgapInstances>*...>;

    AirgapPtrTuple airgap_instances;

public:
    AirgapArray(std::tuple<AirgapInstances&...> _instances) {
        airgap_instances =
            std::apply([](auto&... instance) { return std::make_tuple(&instance...); }, _instances);
    }

    template <size_t Index> auto& get_airgap() {
        static_assert(Index < AirgapCount, "Index out of bounds in AirgapArray::get_airgap()");
        return *std::get<Index>(airgap_instances);
    }
};

#endif // AIRGAP_HPP
