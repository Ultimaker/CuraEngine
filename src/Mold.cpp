/** Copyright (C) 2017 Ultimaker - Released under terms of the AGPLv3 License */
#include "Mold.h"
#include "utils/intpoint.h"

namespace cura
{

void Mold::process(Slicer& slicer, coord_t layer_height, double angle, coord_t width, coord_t open_polyline_width)
{
    Polygons mold_outline_above; // the outside of the mold

    coord_t inset = tan(angle / 180 * M_PI) * layer_height;
    for (int layer_nr = slicer.layers.size() - 1; layer_nr >= 0; layer_nr--)
    {
        SlicerLayer& layer = slicer.layers[layer_nr];
        Polygons layer_outlines = layer.polygons.unionPolygons(layer.openPolylines.offsetPolyLine(open_polyline_width / 2));
        if (angle >= 90)
        {
            layer.polygons = layer_outlines.offset(width).difference(layer_outlines);
        }
        else
        {
            mold_outline_above = mold_outline_above.offset(-inset).unionPolygons(layer_outlines);
            layer.polygons = mold_outline_above.difference(layer_outlines);
        }
    }

}


}//namespace cura
