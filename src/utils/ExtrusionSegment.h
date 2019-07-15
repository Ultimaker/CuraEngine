//Copyright (c) 2019 Ultimaker B.V.


#ifndef UTILS_EXTRUSION_SEGMENT_H
#define UTILS_EXTRUSION_SEGMENT_H

#include <utility>

#include "utils/IntPoint.h"
#include "utils/polygon.h"
#include "utils/polygonUtils.h"

namespace arachne
{

/*!
 * extrusion bead
 * suports varying width
 */
class ExtrusionSegment
{
    static constexpr float a_step = 15 / 180.0 * M_PI;
public:
    Point from;
    coord_t from_width;
    Point to;
    coord_t to_width;

    coord_t index; //!< perimeter number; outer perimeter is zero
    bool is_odd; //!< Whether this is a polyline segment rather than a polygonal segment

    ExtrusionSegment(Point from, coord_t from_width, Point to, coord_t to_width, coord_t index, bool is_odd)
    : from(from)
    , from_width(from_width)
    , to(to)
    , to_width(to_width)
    , index(index)
    , is_odd(is_odd)
    {}

    Polygons toPolygons()
    {
        Polygons ret;
        PolygonUtils::makeCircle(from, from_width / 2, ret, a_step);
        PolygonUtils::makeCircle(to, to_width / 2, ret, a_step);
        Polygons rect;
        PolygonRef r = rect.newPoly();
        Point n = normal(turn90CCW(to - from), from_width / 2);
        r.add(from + n);
        r.add(from - n);
        n = normal(turn90CCW(to - from), to_width / 2);
        r.add(to - n);
        r.add(to + n);
        ret = ret.unionPolygons(rect);
        return ret;
    }
};




} // namespace arachne
#endif // UTILS_EXTRUSION_SEGMENT_H
