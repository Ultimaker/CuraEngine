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

PrimeTowerInterleaved::PrimeTowerInterleaved(size_t extruder_count)
    : PrimeTower(extruder_count)
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

std::map<LayerIndex, std::map<size_t, Shape>>
    PrimeTowerInterleaved::generateExtrusionsMoves(const LayerVector<std::vector<ExtruderUse>>& extruders_use, const SliceDataStorage& storage)
{
    const Scene& scene = Application::getInstance().current_slice_->scene;
    const Settings& mesh_group_settings = scene.current_mesh_group->settings;
    const coord_t tower_radius = mesh_group_settings.get<coord_t>("prime_tower_size") / 2;
    coord_t support_radius = tower_radius;
    std::map<LayerIndex, std::map<size_t, Shape>> moves;

    // Now loop again, but from top bo bottom, so that the required support increases with what is actually required
    for (LayerIndex layer_nr = storage.max_print_height_second_to_last_extruder; layer_nr >= -Raft::getTotalExtraLayers(); --layer_nr)
    {
        const std::vector<ExtruderUse>& extruders_use_at_layer = extruders_use[layer_nr];
        std::map<size_t, Shape> moves_at_layer;

        // Now generate actual priming patterns
        coord_t outer_radius = tower_radius;
        for (const ExtruderUse& extruder_use : extruders_use_at_layer)
        {
            if (extruder_use.prime == ExtruderPrime::Prime)
            {
                Shape extruder_moves;
                std::tie(extruder_moves, outer_radius) = generatePrimeMoves(extruder_use.extruder_nr, outer_radius);
                moves_at_layer[extruder_use.extruder_nr] = extruder_moves;
            }
        }

        // Generate extra "support" sparse pattern if required
        if (support_radius < outer_radius)
        {
            Shape support_moves = generateSupportMoves(extruders_use_at_layer.back().extruder_nr, outer_radius, support_radius);
            moves_at_layer[extruders_use_at_layer.back().extruder_nr].push_back(support_moves);
        }

        // Now decrease support radius if required
        support_radius = std::min(support_radius, outer_radius);

        moves[layer_nr] = moves_at_layer;
    }

    return moves;
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
    const LayerPlan& gcode_layer,
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

void PrimeTowerInterleaved::processExtruderNoPrime(const size_t /*extruder_nr*/, LayerPlan& /*gcode_layer*/) const
{
    // Do nothing because we want to know which extruder has been additionally processed
}

void PrimeTowerInterleaved::polishExtrudersUses(LayerVector<std::vector<ExtruderUse>>& extruders_use, const SliceDataStorage& storage, const size_t start_extruder)
{
    size_t last_used_extruder = start_extruder;

    // Loop through the extruders uses from bottom to top to find the last used extruder at each layer, and make sure we always have some support to print
    for (LayerIndex layer_nr = -Raft::getTotalExtraLayers(); layer_nr <= storage.max_print_height_second_to_last_extruder; ++layer_nr)
    {
        std::vector<ExtruderUse>& extruders_use_at_layer = extruders_use[layer_nr];

        // Make sure we always have something to print
        if (extruders_use_at_layer.empty())
        {
            extruders_use_at_layer.emplace_back(last_used_extruder, ExtruderPrime::Sparse);
        }
        else if (std::all_of(
                     extruders_use_at_layer.begin(),
                     extruders_use_at_layer.end(),
                     [](const ExtruderUse& extruder_use)
                     {
                         return extruder_use.prime == ExtruderPrime::None;
                     }))
        {
            extruders_use_at_layer.back().prime = ExtruderPrime::Sparse;
        }

        last_used_extruder = extruders_use_at_layer.back().extruder_nr;
    }
}

} // namespace cura
