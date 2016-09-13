/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#include "ConicalOverhang.h"


namespace cura {


void ConicalOverhang::apply(Slicer* slicer, double angle, int layer_thickness)
{
    double tanAngle = tan(angle);  // the XY-component of the angle
    int max_dist_from_lower_layer = tanAngle * layer_thickness; // max dist which can be bridged

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
            layer.polygons = layer.polygons.smooth(safe_dist, 100*100);
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
