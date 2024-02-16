// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GEOMETRY_GENERIC_CLOSED_POLYLINE_H
#define GEOMETRY_GENERIC_CLOSED_POLYLINE_H

#include "geometry/point2ll.h"
#include "geometry/polyline.h"
#include "geometry/polyline_type.h"

namespace cura
{

template<PolylineType PolylineTypeVal>
class GenericClosedPolyline : public Polyline<PolylineTypeVal>
{
    friend class Polygons;

public:
    GenericClosedPolyline() = default;

    GenericClosedPolyline(const std::initializer_list<Point2LL>& initializer)
        : Polyline<PolylineTypeVal>(initializer)
    {
    }

    GenericClosedPolyline(const std::vector<Point2LL>& points)
        : Polyline<PolylineTypeVal>(points)
    {
    }

    GenericClosedPolyline& operator=(const GenericClosedPolyline& other)
    {
        Polyline<PolylineTypeVal>::operator=(other);
        return *this;
    }

    /*!
     * Check if we are inside the polygon. We do this by tracing from the point towards the positive X direction,
     * every line we cross increments the crossings counter. If we have an even number of crossings then we are not inside the polygon.
     * Care needs to be taken, if p.Y exactly matches a vertex to the right of p, then we need to count 1 intersect if the
     * outline passes vertically past; and 0 (or 2) intersections if that point on the outline is a 'top' or 'bottom' vertex.
     * The easiest way to do this is to break out two cases for increasing and decreasing Y ( from p0 to p1 ).
     * A segment is tested if pa.Y <= p.Y < pb.Y, where pa and pb are the points (from p0,p1) with smallest & largest Y.
     * When both have the same Y, no intersections are counted but there is a special test to see if the point falls
     * exactly on the line.
     *
     * Returns false if outside, true if inside; if the point lies exactly on the border, will return 'border_result'.
     *
     * \deprecated This function is no longer used, since the Clipper function is used by the function PolygonRef::inside(.)
     *
     * \param p The point for which to check if it is inside this polygon
     * \param border_result What to return when the point is exactly on the border
     * \return Whether the point \p p is inside this polygon (or \p border_result when it is on the border)
     */
    // bool _inside(Point2LL p, bool border_result = false) const;

    /*!
     * Clipper function.
     * Returns false if outside, true if inside; if the point lies exactly on the border, will return 'border_result'.
     *
     * http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Functions/PointInPolygon.htm
     */
    bool inside(Point2LL p, bool border_result = false) const;

    bool inside(const auto& polygon) const;
};

// ###########################################################
// Definitions of templated methods
// ###########################################################
template<PolylineType PolylineTypeVal>
bool GenericClosedPolyline<PolylineTypeVal>::inside(Point2LL p, bool border_result) const
{
    int res = ClipperLib::PointInPolygon(p, *this);
    if (res == -1)
    {
        return border_result;
    }
    return res == 1;
}

template<PolylineType PolylineTypeVal>
bool GenericClosedPolyline<PolylineTypeVal>::inside(const auto& polygon) const
{
    for (const auto& point : *this)
    {
        if (! ClipperLib::PointInPolygon(point, polygon))
        {
            return false;
        }
    }
    return true;
}

} // namespace cura

#endif // GEOMETRY_GENERIC_CLOSED_POLYLINE_H
