//Copyright (c) 2016 Tim Kuipers
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ConicalOverhang.h"
#include "mesh.h"
#include "slicer.h"
#include "settings/types/AngleRadians.h" //To process the overhang angle.

namespace cura {

void ConicalOverhang::apply(Slicer* slicer, const Mesh& mesh)
{
    const AngleRadians angle = mesh.settings.get<AngleRadians>("conical_overhang_angle");
    const double tan_angle = tan(angle);  // the XY-component of the angle
    const coord_t layer_thickness = mesh.settings.get<coord_t>("layer_height");
    coord_t max_dist_from_lower_layer = tan_angle * layer_thickness; // max dist which can be bridged

    for (unsigned int layer_nr = slicer->layers.size() - 2; static_cast<int>(layer_nr) >= 0; layer_nr--)
    {
        SlicerLayer& layer = slicer->layers[layer_nr];
        SlicerLayer& layer_above = slicer->layers[layer_nr + 1];
        if (std::abs(max_dist_from_lower_layer) < 5)
        { // magically nothing happens when max_dist_from_lower_layer == 0
            // below magic code solves that
            int safe_dist = 20;
            Polygons diff = layer_above.polygons.difference(layer.polygons.offset(-safe_dist));
            layer.polygons = layer.polygons.unionPolygons(diff);
            layer.polygons = layer.polygons.smooth(safe_dist);
            layer.polygons.simplify(safe_dist, safe_dist * safe_dist / 4);
            // somehow layer.polygons get really jagged lines with a lot of vertices
            // without the above steps slicing goes really slow
        }
        else
        {
            layer.polygons = layer.polygons.unionPolygons(layer_above.polygons.offset(-max_dist_from_lower_layer));
        }
    }
}

}//namespace cura
