// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PrimeTower/PrimeTowerInterleaved.h"

#include "Application.h"
#include "LayerPlan.h"
#include "Scene.h"
#include "Slice.h"
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

bool PrimeTowerInterleaved::requiresBaseExtraPrint(size_t /*extruder_nr*/) const
{
    // We don't know yet which extruder is going to be used to print the base outer rings, so generate them for all extruders
    return true;
}

bool PrimeTowerInterleaved::requiresFirstLayerExtraInnerPrint(size_t extruder_nr) const
{
    // We don't know yet which extruder is going to be used to print the extra inner rings, so generate them for all extruders
    return true;
}

std::map<size_t, std::map<size_t, Shape>> PrimeTowerInterleaved::generateSparseInfillImpl(const std::vector<coord_t>& rings_radii) const
{
    const Scene& scene = Application::getInstance().current_slice_->scene;

    struct ActualExtruder
    {
        size_t number;
        coord_t line_width;
    };

    std::vector<ActualExtruder> actual_extruders;
    actual_extruders.reserve(extruder_order_.size());
    for (size_t extruder_nr : extruder_order_)
    {
        const coord_t line_width = scene.extruders[extruder_nr].settings_.get<coord_t>("prime_tower_line_width");
        actual_extruders.push_back({ extruder_nr, line_width });
    }

    // Generate all possible extruders combinations, e.g. if there are 4 extruders, we have combinations
    // 0 / 0-1 / 0-1-2 / 0-1-2-3 / 1 / 1-2 / 1-2-3 / 2 / 2-3 / 3
    // A combination is represented by a bitmask
    std::map<size_t, std::map<size_t, Shape>> sparse_pattern_per_extruders;
    for (size_t first_extruder_idx = 0; first_extruder_idx < extruder_order_.size(); ++first_extruder_idx)
    {
        for (size_t last_extruder_idx = first_extruder_idx; last_extruder_idx < extruder_order_.size(); ++last_extruder_idx)
        {
            size_t extruders_combination = 0;
            for (size_t extruder_idx = first_extruder_idx; extruder_idx <= last_extruder_idx; ++extruder_idx)
            {
                size_t extruder_nr = extruder_order_.at(extruder_idx);
                extruders_combination |= (1 << extruder_nr);
            }

            std::map<size_t, Shape> infills_for_combination;
            for (const ActualExtruder& actual_extruder : actual_extruders)
            {
                Shape infill = generatePath_sparseInfill(first_extruder_idx, last_extruder_idx, rings_radii, actual_extruder.line_width, actual_extruder.number);
                infills_for_combination[actual_extruder.number] = infill;
            }

            sparse_pattern_per_extruders[extruders_combination] = infills_for_combination;
        }
    }

    return sparse_pattern_per_extruders;
}

std::vector<size_t> PrimeTowerInterleaved::findExtrudersSparseInfill(
    LayerPlan& gcode_layer,
    const std::vector<ExtruderUse>& required_extruder_prime,
    const std::vector<size_t>& initial_list_idx) const
{
    std::vector<size_t> extruders_to_prime_idx;

    for (size_t extruder_idx = 0; extruder_idx < extruder_order_.size(); extruder_idx++)
    {
        auto iterator_initial_list = std::find(initial_list_idx.begin(), initial_list_idx.end(), extruder_idx);
        bool is_in_initial_list = iterator_initial_list != initial_list_idx.end();

        if (is_in_initial_list)
        {
            extruders_to_prime_idx.push_back(extruder_idx);
        }
        else
        {
            // If extruder is not the current used one, try to see if it requires a sparse prime that we could do
            // at the same time
            size_t extruder_nr = extruder_order_.at(extruder_idx);
            if (! gcode_layer.getPrimeTowerIsPlanned(extruder_nr))
            {
                auto iterator_required_list = std::find_if(
                    required_extruder_prime.begin(),
                    required_extruder_prime.end(),
                    [extruder_nr](const ExtruderUse& extruder_use)
                    {
                        return extruder_use.extruder_nr == extruder_nr && extruder_use.prime == ExtruderPrime::Prime;
                    });
                bool is_in_required_list = iterator_required_list != required_extruder_prime.end();

                if (! is_in_required_list)
                {
                    extruders_to_prime_idx.push_back(extruder_idx);
                }
            }
        }
    }

    return extruders_to_prime_idx;
}

} // namespace cura
