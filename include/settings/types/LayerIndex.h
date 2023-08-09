// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef LAYERINDEX_H
#define LAYERINDEX_H

#include "utils/types/numeric_facade.h"

#include <functional>

namespace cura
{

/*
 * \brief Struct behaving like a layer number.
 *
 * This is a facade. It behaves exactly like an integer but is used to indicate
 * that it is a layer number.
 */
struct LayerIndex : public utils::NumericFacade<int64_t>
{
    using base_type = utils::NumericFacade<int64_t>;
    using base_type::NumericFacade;

    constexpr LayerIndex(const base_type& base) noexcept
        : base_type{ base } {};
};

} // namespace cura

namespace std
{
template<>
struct hash<cura::LayerIndex>
{
    auto operator()(const cura::LayerIndex& layer_index) const
    {
        return hash<decltype(layer_index.value)>()(layer_index.value);
    }
};
} // namespace std

#endif // LAYERINDEX_H
