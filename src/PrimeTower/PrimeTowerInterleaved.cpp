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

PrimeTowerInterleaved::PrimeTowerInterleaved()
    : PrimeTower()
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

std::map<LayerIndex, std::vector<PrimeTower::ExtruderMoves>> PrimeTowerInterleaved::generateExtrusionsMoves(const LayerVector<std::vector<ExtruderUse>>& extruders_use)
{
    const Scene& scene = Application::getInstance().current_slice_->scene;
    const Settings& mesh_group_settings = scene.current_mesh_group->settings;
    const coord_t tower_radius = mesh_group_settings.get<coord_t>("prime_tower_size") / 2;
    coord_t support_radius = tower_radius;
    std::map<LayerIndex, std::vector<ExtruderMoves>> moves;

    // Loop from top bo bottom, so that the required support increases with what is actually required
    for (auto iterator = extruders_use.rbegin(); iterator != extruders_use.rend(); ++iterator)
    {
        const LayerIndex layer_nr = extruders_use.getLayer(iterator);
        const std::vector<ExtruderUse>& extruders_use_at_layer = extruders_use[layer_nr];
        std::vector<ExtruderMoves> moves_at_layer;
        size_t last_extruder_sparse = 0;

        // Now generate actual priming patterns
        coord_t outer_radius = tower_radius;
        for (const ExtruderUse& extruder_use : extruders_use_at_layer)
        {
            if (extruder_use.prime == ExtruderPrime::Prime)
            {
                Shape extruder_moves;
                std::tie(extruder_moves, outer_radius) = generatePrimeMoves(extruder_use.extruder_nr, outer_radius);
                moves_at_layer.emplace_back(extruder_use.extruder_nr, extruder_moves);
            }
            else if (extruder_use.prime == ExtruderPrime::Sparse)
            {
                last_extruder_sparse = extruder_use.extruder_nr;
            }
        }

        // Generate extra "support" sparse pattern if required
        if (support_radius < outer_radius)
        {
            if (moves_at_layer.empty())
            {
                moves_at_layer.emplace_back(last_extruder_sparse, Shape());
            }

            ExtruderMoves& last_extruder_moves = moves_at_layer.back();
            Shape support_moves = generateSupportMoves(last_extruder_moves.extruder_nr, outer_radius, support_radius);
            last_extruder_moves.moves.push_back(support_moves);
        }

        // Now decrease support radius if required
        support_radius = std::min(support_radius, outer_radius);

        moves[layer_nr] = moves_at_layer;
    }

    return moves;
}

void PrimeTowerInterleaved::polishExtrudersUses(LayerVector<std::vector<ExtruderUse>>& extruders_use, const size_t start_extruder)
{
    size_t last_used_extruder = start_extruder;

    // Loop through the extruders uses from bottom to top to find the last used extruder at each layer, and make sure we always have some support to print
    for (auto iterator = extruders_use.begin(); iterator != extruders_use.end(); ++iterator)
    {
        std::vector<ExtruderUse>& extruders_use_at_layer = *iterator;

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
