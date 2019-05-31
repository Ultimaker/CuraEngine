//Copyright (c) 2019 Ultimaker B.V.


#ifndef UTILS_EXTRUSION_SEGMENT_H
#define UTILS_EXTRUSION_SEGMENT_H

#include <utility>

#include "utils/IntPoint.h"
#include "utils/polygon.h"

namespace arachne
{

/*!
 * extrusion bead
 * suports varying width
 */
class ExtrusionSegment
{
public:
    Point from;
    coord_t from_width;
    Point to;
    coord_t to_width;

    ExtrusionSegment(Point from, coord_t from_width, Point to, coord_t to_width)
    : from(from)
    , from_width(from_width)
    , to(to)
    , to_width(to_width)
    {}

    Polygons toPolygons()
    {
        Polygons ret;
        PolygonRef from_circle = ret.newPoly();
        for (float a = 0; a < 360; a += 30)
        {
            from_circle.emplace_back(from + Point(from_width / 2 * cos(a/180.0 * M_PI), from_width / 2 * sin(a/180.0 * M_PI)));
        }
        PolygonRef to_circle = ret.newPoly();
        for (float a = 0; a < 360; a += 30)
        {
            to_circle.emplace_back(to + Point(to_width / 2 * cos(a/180.0 * M_PI), to_width / 2 * sin(a/180.0 * M_PI)));
        }
        ret = ret.approxConvexHull();
        return ret;
    }
};




} // namespace arachne
#endif // UTILS_EXTRUSION_SEGMENT_H
