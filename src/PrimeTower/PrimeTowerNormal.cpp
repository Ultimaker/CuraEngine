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
    : PrimeTower(extruder_count)
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

std::map<LayerIndex, std::map<size_t, Shape>> PrimeTowerNormal::generateExtrusionsMoves(const LayerVector<std::vector<ExtruderUse>>& extruders_use, const SliceDataStorage& storage)
{
    return {};
}

bool PrimeTowerNormal::requiresBaseExtraPrint(size_t extruder_nr) const
{
    // Generate base extra rings only for the outermost printed extruder
    return extruder_nr == extruder_order_.front();
}

bool PrimeTowerNormal::requiresFirstLayerExtraInnerPrint(size_t extruder_nr) const
{
    // Generate extra inner rings only for the innermost printed extruder
    return extruder_nr == extruder_order_.back();
}

std::map<size_t, std::map<size_t, Shape>> PrimeTowerNormal::generateSparseInfillImpl(const std::vector<coord_t>& rings_radii) const
{
    const Scene& scene = Application::getInstance().current_slice_->scene;

    // Generate a sparse infill for each extruder
    std::map<size_t, std::map<size_t, Shape>> sparse_pattern_per_extruders;
    for (size_t extruder_idx = 0; extruder_idx < extruder_order_.size(); ++extruder_idx)
    {
        const size_t extruder_nr = extruder_order_[extruder_idx];
        const size_t extruders_combination = (1 << extruder_nr);
        const coord_t line_width = scene.extruders[extruder_nr].settings_.get<coord_t>("prime_tower_line_width");

        std::map<size_t, Shape> infills_for_combination;
        Shape infill = generatePath_sparseInfill(extruder_idx, extruder_idx, rings_radii, line_width, extruder_nr);
        infills_for_combination[extruder_nr] = infill;

        sparse_pattern_per_extruders[extruders_combination] = infills_for_combination;
    }

    return sparse_pattern_per_extruders;
}

std::vector<size_t> PrimeTowerNormal::findExtrudersSparseInfill(
    const LayerPlan& /*gcode_layer*/,
    const std::vector<ExtruderUse>& /*required_extruder_prime*/,
    const std::vector<size_t>& initial_list_idx) const
{
    // In normal mode we only print what is required
    return initial_list_idx;
}

void PrimeTowerNormal::processExtruderNoPrime(const size_t extruder_nr, LayerPlan& gcode_layer) const
{
    gcode_layer.setPrimeTowerIsPlanned(extruder_nr);
}

} // namespace cura
