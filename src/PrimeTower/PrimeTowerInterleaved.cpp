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
    // For now, just calculate prime or not. Support extrusion requires the whole extruders list to be calculted, and
    // will be processed later.
    if (extruderRequiresPrime(extruder_is_used_on_this_layer, extruder_nr, last_extruder))
    {
        return ExtruderPrime::Prime;
    }
    else
    {
        return ExtruderPrime::None;
    }
}

std::map<LayerIndex, std::vector<PrimeTower::ExtruderToolPaths>> PrimeTowerInterleaved::generateToolPaths(const LayerVector<std::vector<ExtruderUse>>& extruders_use)
{
    const Scene& scene = Application::getInstance().current_slice_->scene;
    const Settings& mesh_group_settings = scene.current_mesh_group->settings;
    const coord_t tower_radius = mesh_group_settings.get<coord_t>("prime_tower_size") / 2;
    coord_t support_radius = tower_radius;
    std::map<LayerIndex, std::vector<ExtruderToolPaths>> toolpaths;

    // Loop from top bo bottom, so that the required support increases with what is actually required
    for (auto iterator = extruders_use.rbegin(); iterator != extruders_use.rend(); ++iterator)
    {
        const LayerIndex layer_nr = extruders_use.getLayer(iterator);
        const std::vector<ExtruderUse>& extruders_use_at_layer = extruders_use[layer_nr];
        std::vector<ExtruderToolPaths> toolpaths_at_layer;
        size_t last_extruder_support = 0;

        // Now generate actual priming patterns
        coord_t outer_radius = tower_radius;
        for (const ExtruderUse& extruder_use : extruders_use_at_layer)
        {
            if (extruder_use.prime == ExtruderPrime::Prime)
            {
                ExtruderToolPaths extruder_toolpaths;
                extruder_toolpaths.outer_radius = outer_radius;
                extruder_toolpaths.extruder_nr = extruder_use.extruder_nr;

                std::tie(extruder_toolpaths.toolpaths, extruder_toolpaths.inner_radius) = generatePrimeToolpaths(extruder_use.extruder_nr, outer_radius);
                toolpaths_at_layer.push_back(extruder_toolpaths);

                outer_radius = extruder_toolpaths.inner_radius;
            }
            else if (extruder_use.prime == ExtruderPrime::Support)
            {
                last_extruder_support = extruder_use.extruder_nr;
            }
        }

        // Generate extra support pattern if required
        if (support_radius < outer_radius)
        {
            if (toolpaths_at_layer.empty())
            {
                toolpaths_at_layer.emplace_back(last_extruder_support, ClosedLinesSet(), outer_radius, support_radius);
            }

            ExtruderToolPaths& last_extruder_toolpaths = toolpaths_at_layer.back();
            ClosedLinesSet support_toolpaths = generateSupportToolpaths(last_extruder_toolpaths.extruder_nr, outer_radius, support_radius);
            last_extruder_toolpaths.toolpaths.push_back(support_toolpaths);
            last_extruder_toolpaths.inner_radius = support_radius;
        }

        // Now decrease support radius if required
        support_radius = std::min(support_radius, outer_radius);

        toolpaths[layer_nr] = toolpaths_at_layer;
    }

    return toolpaths;
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
            extruders_use_at_layer.emplace_back(last_used_extruder, ExtruderPrime::Support);
        }
        else if (std::all_of(
                     extruders_use_at_layer.begin(),
                     extruders_use_at_layer.end(),
                     [](const ExtruderUse& extruder_use)
                     {
                         return extruder_use.prime == ExtruderPrime::None;
                     }))
        {
            extruders_use_at_layer.back().prime = ExtruderPrime::Support;
        }

        last_used_extruder = extruders_use_at_layer.back().extruder_nr;
    }
}

} // namespace cura
