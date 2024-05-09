// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PrimeTower/PrimeTowerNormal.h"

#include "sliceDataStorage.h"

namespace cura
{

PrimeTowerNormal::PrimeTowerNormal(SliceDataStorage& storage, size_t extruder_count)
    : PrimeTower(storage, extruder_count)
{
}

ExtruderPrime PrimeTowerNormal::getExtruderPrime(
    const std::vector<bool>& extruder_is_used_on_this_layer,
    size_t extruder_nr,
    size_t last_extruder,
    const SliceDataStorage& storage,
    const LayerIndex& layer_nr) const
{
    if (extruderRequiresPrime(extruder_is_used_on_this_layer, extruder_nr, last_extruder))
    {
        return ExtruderPrime::Prime;
    }
    else if (layer_nr < storage.max_print_height_second_to_last_extruder)
    {
        return ExtruderPrime::Sparse;
    }
    else
    {
        return ExtruderPrime::None;
    }
}

} // namespace cura
