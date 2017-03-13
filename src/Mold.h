/** Copyright (C) 2017 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef MOLD_H
#define MOLD_H

#include "slicer.h"
#include "utils/intpoint.h"

namespace cura
{

/*!
 * Class for transforming the outlines of a model into a mold into which the model can be cast.
 */
class Mold
{
public:
    static void process(Slicer& slicer, coord_t layer_height, double angle, coord_t width, coord_t open_polyline_width);
private:
};

}//namespace cura

#endif//MOLD_H
