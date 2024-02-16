// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GEOMETRY_OPEN_POLYLINE_H
#define GEOMETRY_OPEN_POLYLINE_H

#include "geometry/polyline.h"

namespace cura
{

class OpenPolyline : public Polyline<PolylineType::Open>
{
public:
    OpenPolyline() = default;

    OpenPolyline(const OpenPolyline& other) = default;

    OpenPolyline(OpenPolyline&& other) = default;

    OpenPolyline(const std::initializer_list<Point2LL>& initializer)
        : Polyline<PolylineType::Open>(initializer)
    {
    }

    OpenPolyline(const std::vector<Point2LL>& points)
        : Polyline<PolylineType::Open>(points)
    {
    }

    OpenPolyline(std::vector<Point2LL>&& points)
        : Polyline<PolylineType::Open>(points)
    {
    }

    OpenPolyline& operator=(const OpenPolyline& other)
    {
        Polyline<PolylineType::Open>::operator=(other);
        return *this;
    }

    OpenPolyline& operator=(OpenPolyline&& other)
    {
        Polyline<PolylineType::Open>::operator=(other);
        return *this;
    }
};

} // namespace cura

#endif // GEOMETRY_OPEN_POLYLINE_H
