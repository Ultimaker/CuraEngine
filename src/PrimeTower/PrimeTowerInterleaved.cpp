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
    const SliceDataStorage& /*storage*/,
    const LayerIndex& /*layer_nr*/) const
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

void PrimeTowerInterleaved::polishExtrudersUse(LayerVector<std::vector<ExtruderUse>>& extruders_use, const SliceDataStorage& storage) const
{
    for (LayerIndex layer_nr = -Raft::getTotalExtraLayers(); layer_nr < storage.print_layer_count; ++layer_nr)
    {
        std::vector<ExtruderUse>& extruders_use_at_layer = extruders_use[layer_nr];
        if (extruders_use_at_layer.size() == 1 && extruders_use_at_layer.front().prime == ExtruderPrime::None && layer_nr <= storage.max_print_height_second_to_last_extruder)
        {
            extruders_use_at_layer.front().prime = ExtruderPrime::Sparse;
        }
    }
}

} // namespace cura
