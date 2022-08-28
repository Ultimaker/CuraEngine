//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "sliceDataStorage.h"
#include "slicer.h"
#include "settings/EnumSettings.h" //For ESurfaceMode.
#include "settings/Settings.h"
#include "utils/ThreadPool.h"

#include "utils/PolylineStitcher.h"
#include "utils/Simplify.h" //Simplifying the layers after creating them.

/*
The layer-part creation step is the first step in creating actual useful data for 3D printing.
It takes the result of the Slice step, which is an unordered list of polygons, and makes groups of polygons,
each of these groups is called a "part", which sometimes are also known as "islands". These parts represent
isolated areas in the 2D layer with possible holes.

Creating "parts" is an important step, as all elements in a single part should be printed before going to another part.
And all every bit inside a single part can be printed without the nozzle leaving the boundary of this part.

It's also the first step that stores the result in the "data storage" so all other steps can access it.
*/

namespace cura {

void createLayerWithParts(const Settings& settings, SliceLayer& storageLayer, SlicerLayer* layer)
{
    PolylineStitcher<Polygons, Polygon, Point>::stitch(layer->openPolylines, storageLayer.openPolyLines, layer->polygons, settings.get<coord_t>("wall_line_width_0"));

    storageLayer.openPolyLines = Simplify(settings).polyline(storageLayer.openPolyLines);

    const bool union_all_remove_holes = settings.get<bool>("meshfix_union_all_remove_holes");
    if (union_all_remove_holes)
    {
        for(unsigned int i=0; i<layer->polygons.size(); i++)
        {
            if (layer->polygons[i].orientation())
                layer->polygons[i].reverse();
        }
    }

    std::vector<PolygonsPart> result;
    const bool union_layers = settings.get<bool>("meshfix_union_all");
    const ESurfaceMode surface_only = settings.get<ESurfaceMode>("magic_mesh_surface_mode");
    if (surface_only == ESurfaceMode::SURFACE && !union_layers)
    { // Don't do anything with overlapping areas; no union nor xor
        result.reserve(layer->polygons.size());
        for (const PolygonRef poly : layer->polygons)
        {
            result.emplace_back();
            result.back().add(poly);
        }
    }
    else
    {
        result = layer->polygons.splitIntoParts(union_layers || union_all_remove_holes);
    }
    const coord_t hole_offset = settings.get<coord_t>("hole_xy_offset");
    for(auto & part : result)
    {
        storageLayer.parts.emplace_back();
        if (hole_offset != 0)
        {
            // holes are to be expanded or shrunk
            Polygons outline;
            Polygons holes;
            for (const PolygonRef poly : part)
            {
                if (poly.orientation())
                {
                    outline.add(poly);
                }
                else
                {
                    holes.add(poly.offset(hole_offset));
                }
            }
            storageLayer.parts.back().outline.add(outline.difference(holes.unionPolygons()));
        }
        else
        {
            storageLayer.parts.back().outline = part;
        }
        storageLayer.parts.back().boundaryBox.calculate(storageLayer.parts.back().outline);
        if (storageLayer.parts.back().outline.empty())
        {
            storageLayer.parts.pop_back();
        }
    }
}
void createLayerParts(SliceMeshStorage& mesh, Slicer* slicer)
{
    const auto total_layers = slicer->layers.size();
    assert(mesh.layers.size() == total_layers);

    cura::parallel_for<size_t>(0, total_layers, [slicer, &mesh](size_t layer_nr)
    {
        SliceLayer& layer_storage = mesh.layers[layer_nr];
        SlicerLayer& slice_layer = slicer->layers[layer_nr];
        createLayerWithParts(mesh.settings, layer_storage, &slice_layer);
    });

    for (LayerIndex layer_nr = total_layers - 1; layer_nr >= 0; layer_nr--)
    {
        SliceLayer& layer_storage = mesh.layers[layer_nr];
        if (layer_storage.parts.size() > 0 || (mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::NORMAL && layer_storage.openPolyLines.size() > 0))
        {
            mesh.layer_nr_max_filled_layer = layer_nr; // last set by the highest non-empty layer
            break;
        }
    }
}

}//namespace cura
