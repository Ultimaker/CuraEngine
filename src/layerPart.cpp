// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "layerPart.h"

#include "geometry/OpenPolyline.h"
#include "progress/Progress.h"
#include "settings/EnumSettings.h" //For ESurfaceMode.
#include "settings/Settings.h"
#include "sliceDataStorage.h"
#include "slicer.h"
#include "utils/OpenPolylineStitcher.h"
#include "utils/Simplify.h" //Simplifying the layers after creating them.
#include "utils/ThreadPool.h"

/*
The layer-part creation step is the first step in creating actual useful data for 3D printing.
It takes the result of the Slice step, which is an unordered list of polygons_, and makes groups of polygons_,
each of these groups is called a "part", which sometimes are also known as "islands". These parts represent
isolated areas in the 2D layer with possible holes.

Creating "parts" is an important step, as all elements in a single part should be printed before going to another part.
And all every bit inside a single part can be printed without the nozzle leaving the boundary of this part.

It's also the first step that stores the result in the "data storage" so all other steps can access it.
*/

namespace cura
{

void createLayerWithParts(const Settings& settings, SliceLayer& storageLayer, SlicerLayer* layer, const Shape& bottom_parts, const Shape& top_parts)
{
    OpenPolylineStitcher::stitch(layer->open_polylines_, storageLayer.open_polylines, layer->polygons_, settings.get<coord_t>("wall_line_width_0"));

    storageLayer.open_polylines = Simplify(settings).polyline(storageLayer.open_polylines);

    const bool union_all_remove_holes = settings.get<bool>("meshfix_union_all_remove_holes");
    if (union_all_remove_holes)
    {
        for (unsigned int i = 0; i < layer->polygons_.size(); i++)
        {
            if (layer->polygons_[i].orientation())
                layer->polygons_[i].reverse();
        }
    }

    std::vector<SingleShape> result;
    const bool union_layers = settings.get<bool>("meshfix_union_all");
    const ESurfaceMode surface_only = settings.get<ESurfaceMode>("magic_mesh_surface_mode");
    if (surface_only == ESurfaceMode::SURFACE && ! union_layers)
    { // Don't do anything with overlapping areas; no union nor xor
        result.reserve(layer->polygons_.size());
        for (const Polygon& poly : layer->polygons_)
        {
            if (poly.empty())
            {
                continue;
            }
            result.emplace_back();
            result.back().push_back(poly);
        }
    }
    else
    {
        result = layer->polygons_.splitIntoParts(union_layers || union_all_remove_holes);
    }

    for (auto& main_part : result)
    {
        std::vector<std::pair<SliceLayerPart::WallExposedType, std::vector<SingleShape>>> parts_by_type = {
            { SliceLayerPart::WallExposedType::BOTTOM_0, bottom_parts.splitIntoParts() },
            { SliceLayerPart::WallExposedType::TOP, top_parts.difference(bottom_parts).splitIntoParts() },
            { SliceLayerPart::WallExposedType::SIDE_ONLY, main_part.difference(bottom_parts).difference(top_parts).splitIntoParts() },
        };

        for (auto& [wall_exposed, parts] : parts_by_type)
        {
            for (auto& part : parts)
            {
                storageLayer.parts.emplace_back();
                if (part.empty())
                {
                    continue;
                }
                auto& back_part = storageLayer.parts.back();
                back_part.wall_exposed = wall_exposed;
                back_part.outline = part;
                back_part.boundaryBox.calculate(back_part.outline);
                if (back_part.outline.empty())
                {
                    storageLayer.parts.pop_back();
                }
            }
        }
    }
}

Shape getTopOrBottom(int direction, const std::string& setting_name, size_t layer_nr, const std::vector<SlicerLayer>& slayers, const Settings& settings)
{
    auto result = Shape();
    if (settings.get<size_t>(setting_name) != settings.get<size_t>("wall_line_count") && ! settings.get<bool>("magic_spiralize"))
    {
        result = slayers[layer_nr].polygons_;
        const auto next_layer = layer_nr + direction;
        if (next_layer >= 0 && next_layer < slayers.size())
        {
            constexpr coord_t EPSILON = 5;
            const auto wall_line_width = settings.get<coord_t>(layer_nr == 0 ? "wall_line_width_0" : "wall_line_width") - 5;
            result = result.offset(-wall_line_width).difference(slayers[next_layer].polygons_).offset(wall_line_width);
        }
    }
    return result;
}

/*!
 * \brief Split a layer into parts.
 * \param settings The settings to get the settings from (whether to union or
 * not).
 * \param storageLayer Where to store the parts.
 * \param layer The layer to split.
 */
void createLayerParts(SliceMeshStorage& mesh, Slicer* slicer)
{
    const auto total_layers = slicer->layers.size();
    assert(mesh.layers.size() == total_layers);

    cura::parallel_for<size_t>(
        0,
        total_layers,
        [slicer, &mesh](size_t layer_nr)
        {
            SliceLayer& layer_storage = mesh.layers[layer_nr];
            SlicerLayer& slice_layer = slicer->layers[layer_nr];
            createLayerWithParts(
                mesh.settings,
                layer_storage,
                &slice_layer,
                layer_nr == 0 ? getTopOrBottom(-1, "wall_line_count_0", layer_nr, slicer->layers, mesh.settings) : Shape(),
                getTopOrBottom(+1, "wall_line_count_top", layer_nr, slicer->layers, mesh.settings));
        });

    for (LayerIndex layer_nr = total_layers - 1; layer_nr >= 0; layer_nr--)
    {
        SliceLayer& layer_storage = mesh.layers[layer_nr];
        if (layer_storage.parts.size() > 0 || (mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::NORMAL && layer_storage.open_polylines.size() > 0))
        {
            mesh.layer_nr_max_filled_layer = layer_nr; // last set by the highest non-empty layer
            break;
        }
    }
}

} // namespace cura
