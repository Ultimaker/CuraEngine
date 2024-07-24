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

PrimeTowerNormal::PrimeTowerNormal(const std::vector<size_t>& used_extruders)
    : PrimeTower()
    , used_extruders_(used_extruders)
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
    else if (layer_nr <= storage.max_print_height_second_to_last_extruder)
    {
        return ExtruderPrime::Support;
    }
    else
    {
        return ExtruderPrime::None;
    }
}

std::map<LayerIndex, std::vector<PrimeTower::ExtruderToolPaths>> PrimeTowerNormal::generateToolPaths(const LayerVector<std::vector<ExtruderUse>>& extruders_use)
{
    const Scene& scene = Application::getInstance().current_slice_->scene;
    const Settings& mesh_group_settings = scene.current_mesh_group->settings;
    const coord_t tower_radius = mesh_group_settings.get<coord_t>("prime_tower_size") / 2;
    std::map<LayerIndex, std::vector<ExtruderToolPaths>> toolpaths;

    // First take all the used extruders numbers, unsorted
    std::vector<size_t> extruder_order = used_extruders_;

    // Then sort from high adhesion to low adhesion. This will give us the outside to inside extruder processing order.
    std::sort(
        extruder_order.begin(),
        extruder_order.end(),
        [&scene](const size_t extruder_nr_a, const size_t extruder_nr_b)
        {
            const Ratio adhesion_a = scene.extruders[extruder_nr_a].settings_.get<Ratio>("material_adhesion_tendency");
            const Ratio adhesion_b = scene.extruders[extruder_nr_b].settings_.get<Ratio>("material_adhesion_tendency");
            return adhesion_a < adhesion_b;
        });

    // For each extruder, generate the prime and support patterns, which will always be the same across layers
    coord_t current_radius = tower_radius;
    std::map<size_t, ExtruderToolPaths> extruders_prime_toolpaths;
    std::map<size_t, ExtruderToolPaths> extruders_support_toolpaths;
    for (size_t extruder_nr : extruder_order)
    {
        ExtruderToolPaths extruder_prime_toolpaths;
        extruder_prime_toolpaths.extruder_nr = extruder_nr;
        extruder_prime_toolpaths.outer_radius = current_radius;
        std::tie(extruder_prime_toolpaths.toolpaths, extruder_prime_toolpaths.inner_radius) = generatePrimeToolpaths(extruder_nr, current_radius);
        extruders_prime_toolpaths[extruder_nr] = extruder_prime_toolpaths;

        ExtruderToolPaths extruder_support_toolpaths = extruder_prime_toolpaths;
        extruder_support_toolpaths.toolpaths = generateSupportToolpaths(extruder_nr, current_radius, extruder_prime_toolpaths.inner_radius);
        extruders_support_toolpaths[extruder_nr] = extruder_support_toolpaths;

        current_radius = extruder_prime_toolpaths.inner_radius;
    }

    // Now fill the extruders toolpaths according to their use
    for (auto iterator = extruders_use.begin(); iterator != extruders_use.end(); ++iterator)
    {
        const LayerIndex layer_nr = extruders_use.getLayer(iterator);
        std::vector<ExtruderUse> extruders_use_at_layer = *iterator;

        // Sort to fit the global order, in order to insert the toolpaths in outside to inside order
        std::sort(
            extruders_use_at_layer.begin(),
            extruders_use_at_layer.end(),
            [extruder_order](const ExtruderUse& extruder_use1, const ExtruderUse& extruder_use2)
            {
                return std::find(extruder_order.begin(), extruder_order.end(), extruder_use1.extruder_nr)
                     < std::find(extruder_order.begin(), extruder_order.end(), extruder_use2.extruder_nr);
            });

        // Now put the proper toolpaths for each extruder use
        std::vector<ExtruderToolPaths> toolpaths_at_layer;
        for (const ExtruderUse& extruder_use : extruders_use_at_layer)
        {
            switch (extruder_use.prime)
            {
            case ExtruderPrime::None:
                break;

            case ExtruderPrime::Prime:
                toolpaths_at_layer.push_back(extruders_prime_toolpaths[extruder_use.extruder_nr]);
                break;

            case ExtruderPrime::Support:
                toolpaths_at_layer.push_back(extruders_support_toolpaths[extruder_use.extruder_nr]);
                break;
            }
        }

        toolpaths[layer_nr] = toolpaths_at_layer;
    }

    return toolpaths;
}

} // namespace cura
