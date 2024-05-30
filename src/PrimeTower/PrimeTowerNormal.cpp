// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PrimeTower/PrimeTowerNormal.h"

#include "Application.h"
#include "LayerPlan.h"
#include "Scene.h"
#include "Slice.h"
#include "sliceDataStorage.h"

namespace cura
{

PrimeTowerNormal::PrimeTowerNormal(size_t extruder_count)
    : PrimeTower()
{
    const Scene& scene = Application::getInstance().current_slice_->scene;

    // First make a basic list of used extruders numbers
    for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        extruder_order_[extruder_nr] = extruder_nr;
    }

    // Then sort from high adhesion to low adhesion.
    std::stable_sort(
        extruder_order_.begin(),
        extruder_order_.end(),
        [&scene](const unsigned int& extruder_nr_a, const unsigned int& extruder_nr_b) -> bool
        {
            const Ratio adhesion_a = scene.extruders[extruder_nr_a].settings_.get<Ratio>("material_adhesion_tendency");
            const Ratio adhesion_b = scene.extruders[extruder_nr_b].settings_.get<Ratio>("material_adhesion_tendency");
            return adhesion_a < adhesion_b;
        });
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

std::map<LayerIndex, std::vector<PrimeTower::ExtruderMoves>>
    PrimeTowerNormal::generateExtrusionsMoves(const LayerVector<std::vector<ExtruderUse>>& extruders_use, const SliceDataStorage& storage)
{
    return {};
}

} // namespace cura
