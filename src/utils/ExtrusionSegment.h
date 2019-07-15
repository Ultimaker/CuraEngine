//Copyright (c) 2019 Ultimaker B.V.


#ifndef UTILS_EXTRUSION_SEGMENT_H
#define UTILS_EXTRUSION_SEGMENT_H

#include <utility>

#include "utils/IntPoint.h"
#include "utils/polygon.h"
#include "utils/polygonUtils.h"
#include "utils/ExtrusionJunction.h"

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
    ExtrusionJunction from;
    ExtrusionJunction to;

    bool is_odd; //!< Whether this is a polyline segment rather than a polygonal segment

    ExtrusionSegment(ExtrusionJunction from, ExtrusionJunction to, bool is_odd)
    : from(from)
    , to(to)
    , is_odd(is_odd)
    {}

    Polygons toPolygons()
    {
        Polygons ret;
        PolygonUtils::makeCircle(from.p, from.w / 2, ret, a_step);
        PolygonUtils::makeCircle(to.p, to.w / 2, ret, a_step);
        Polygons rect;
        PolygonRef r = rect.newPoly();
        Point n = normal(turn90CCW(to.p - from.p), from.w / 2);
        r.add(from.p + n);
        r.add(from.p - n);
        n = normal(turn90CCW(to.p - from.p), to.w / 2);
        r.add(to.p - n);
        r.add(to.p + n);
        ret = ret.unionPolygons(rect);
        return ret;
    }
};




} // namespace arachne
#endif // UTILS_EXTRUSION_SEGMENT_H
