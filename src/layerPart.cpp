/** Copyright (C) 2013 Ultimaker - Released under terms of the AGPLv3 License */

#include "layerPart.h"
#include "settings/settings.h"
#include "progress/Progress.h"

#include "utils/SVG.h" // debug output

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

void createLayerWithParts(SliceLayer& storageLayer, SlicerLayer* layer, bool union_layers, bool union_all_remove_holes)
{
    storageLayer.openPolyLines = layer->openPolylines;

    if (union_all_remove_holes)
    {
        for(unsigned int i=0; i<layer->polygons.size(); i++)
        {
            if (layer->polygons[i].orientation())
                layer->polygons[i].reverse();
        }
    }
    
    std::vector<PolygonsPart> result;
    result = layer->polygons.splitIntoParts(union_layers || union_all_remove_holes);
    for(unsigned int i=0; i<result.size(); i++)
    {
        storageLayer.parts.emplace_back();
        storageLayer.parts[i].outline = result[i];
        storageLayer.parts[i].boundaryBox.calculate(storageLayer.parts[i].outline);
    }
}
void createLayerParts(SliceMeshStorage& mesh, Slicer* slicer, bool union_layers, bool union_all_remove_holes)
{
    const auto total_layers = slicer->layers.size();
    assert(mesh.layers.size() == total_layers);
#pragma omp parallel for default(none) shared(mesh,slicer) firstprivate(union_layers,union_all_remove_holes) schedule(dynamic)
    for (unsigned int layer_nr = 0; layer_nr < total_layers; layer_nr++)
    {
        SliceLayer& layer_storage = mesh.layers[layer_nr];
        SlicerLayer& slice_layer = slicer->layers[layer_nr];
        createLayerWithParts(layer_storage, &slice_layer, union_layers, union_all_remove_holes);
    }

    for (unsigned int layer_nr = total_layers - 1; static_cast<int>(layer_nr) != -1; layer_nr--)
    {
        SliceLayer& layer_storage = mesh.layers[layer_nr];
        if (layer_storage.parts.size() > 0 || (mesh.getSettingAsSurfaceMode("magic_mesh_surface_mode") != ESurfaceMode::NORMAL && layer_storage.openPolyLines.size() > 0) )
        {
            mesh.layer_nr_max_filled_layer = layer_nr; // last set by the highest non-empty layer
            break;
        }
    }
}

}//namespace cura
