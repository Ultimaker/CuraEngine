/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#include "ConicalOverhang.h"


namespace cura {


void ConicalOverhang::apply(Slicer* slicer, double angle, int layer_thickness)
{
    double tanAngle = tan(angle) - 0.01;  // the XY-component of the angle
    int max_dist_from_lower_layer = tanAngle * layer_thickness; // max dist which can be bridged

    for (unsigned int layer_nr = slicer->layers.size() - 2; static_cast<int>(layer_nr) >= 0; layer_nr--)
    {
        SlicerLayer& layer = slicer->layers[layer_nr];
        SlicerLayer& layer_above = slicer->layers[layer_nr + 1];
        layer.polygons = layer.polygons.unionPolygons(layer_above.polygons.offset(-max_dist_from_lower_layer));
    }
}

}//namespace cura
