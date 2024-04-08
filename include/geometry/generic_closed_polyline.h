// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GEOMETRY_GENERIC_CLOSED_POLYLINE_H
#define GEOMETRY_GENERIC_CLOSED_POLYLINE_H

#include "geometry/point2ll.h"
#include "geometry/polyline.h"
#include "geometry/polyline_type.h"

namespace cura
{
/*
template<PolylineType PolylineTypeVal>
class GenericClosedPolyline : public Polyline<PolylineTypeVal>
{
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
};
*/
} // namespace cura

#endif // GEOMETRY_GENERIC_CLOSED_POLYLINE_H
