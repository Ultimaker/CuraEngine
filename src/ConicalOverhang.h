/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef CONICAL_OVERHANG_H
#define CONICAL_OVERHANG_H

#include "slicer.h"


namespace cura {

class ConicalOverhang
{
public:
    static void apply(Slicer* slicer, double angle, int layer_thickness)
    {
        double tanAngle = tan(angle) - 0.01;  // the XY-component of the angle
        int max_dist_from_lower_layer = tanAngle * layer_thickness; // max dist which can be bridged

        for (unsigned int layer_nr = slicer->layers.size() - 2; static_cast<int>(layer_nr) >= 0; layer_nr--)
        {
            SlicerLayer& layer = slicer->layers[layer_nr];
            SlicerLayer& layer_above = slicer->layers[layer_nr + 1];
            layer.polygonList = layer.polygonList.unionPolygons(layer_above.polygonList.offset(-max_dist_from_lower_layer));
        }
    }
};

}//namespace cura

#endif // CONICAL_OVERHANG_H
