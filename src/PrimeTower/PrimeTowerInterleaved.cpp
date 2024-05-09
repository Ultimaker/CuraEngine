// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PrimeTower/PrimeTowerInterleaved.h"

#include "sliceDataStorage.h"

namespace cura
{

PrimeTowerInterleaved::PrimeTowerInterleaved(SliceDataStorage& storage, size_t extruder_count)
    : PrimeTower(storage, extruder_count)
{
}

ExtruderPrime PrimeTowerInterleaved::getExtruderPrime(
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
    else
    {
        return ExtruderPrime::None;
    }
}

void PrimeTowerInterleaved::polishExtruderUse(std::vector<ExtruderUse>& extruder_use, const SliceDataStorage& storage, const LayerIndex& layer_nr) const
{
    if (extruder_use.size() == 1 && extruder_use.front().prime == ExtruderPrime::None && layer_nr <= storage.max_print_height_second_to_last_extruder)
    {
        extruder_use.front().prime = ExtruderPrime::Sparse;
    }
}

} // namespace cura
