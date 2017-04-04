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
    /*!
     * Convert the polygons (and open polylines) in \p slicer
     * in order to make a mold with a cutout in the shape of the outlines.
     * 
     * The open polylines of the model are also used to cut out the shape inside the mold.
     * 
     * \param slicer The container for the sliced polygons (and open polylines)
     * \param layer_height The overall layer height used (used to compute an offset from the \p angle)
     * \param angle The overhang angle for the outer outlines of the mold
     * \param width The minmal width of the mold; the minmal distance between the outside and inside of the mold in horizontal direction
     * \param open_polyline_width The width with which the open polylines are cut out of the mold
     */
    static void process(Slicer& slicer, coord_t layer_height, double angle, coord_t width, coord_t open_polyline_width);
private:
};

}//namespace cura

#endif//MOLD_H
