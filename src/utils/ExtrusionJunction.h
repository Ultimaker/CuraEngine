//Copyright (c) 2019 Ultimaker B.V.


#ifndef UTILS_EXTRUSION_JUNCTION_H
#define UTILS_EXTRUSION_JUNCTION_H

#include "IntPoint.h"

namespace arachne
{

struct ExtrusionJunction
{
    Point p;
    coord_t w;
    size_t perimeter_index;
    ExtrusionJunction(Point p, coord_t w, coord_t perimeter_index)
    : p(p), w(w), perimeter_index(perimeter_index) {}
    bool operator==(const ExtrusionJunction& other) const
    {
        return p == other.p
            && w == other.w
            && perimeter_index == other.perimeter_index;
    }
};




} // namespace arachne
#endif // UTILS_EXTRUSION_JUNCTION_H
