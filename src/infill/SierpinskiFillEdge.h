/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef INFILL_SIERPINSKI_FILL_EDGE_H
#define INFILL_SIERPINSKI_FILL_EDGE_H

#include "../utils/intpoint.h"

namespace cura
{

/*!
 * The type of an edge of a triangle in the space subdivision
 */
struct SierpinskiFillEdge
{
    bool direction; //!< Whether the edge is diagonal, rather than horizontal or vertical
    Point l; //!< The vertex of the edge to the left / inside of the curve
    Point r; //!< The vertex of the edge to the right / outside of the curve
    unsigned int depth; //!< The iteration at which the triangle enclosed by this edge and the next edge was created
    SierpinskiFillEdge(const bool direction, const Point l, const Point r, unsigned int depth) // basic constructor
    : direction(direction)
    , l(l)
    , r(r)
    , depth(depth)
    {
    }
};

} // namespace cura


#endif // INFILL_SIERPINSKI_FILL_EDGE_H
