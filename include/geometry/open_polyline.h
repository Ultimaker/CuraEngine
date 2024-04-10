// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GEOMETRY_OPEN_POLYLINE_H
#define GEOMETRY_OPEN_POLYLINE_H

#include "geometry/polyline.h"

namespace cura
{

class OpenPolyline : public Polyline
{
public:
    OpenPolyline() = default;

    OpenPolyline(const OpenPolyline& other) = default;

    OpenPolyline(OpenPolyline&& other) = default;

    OpenPolyline(const std::initializer_list<Point2LL>& initializer)
        : Polyline(initializer)
    {
    }

    OpenPolyline(const ClipperLib::Path& points)
        : Polyline(points)
    {
    }

    OpenPolyline(ClipperLib::Path&& points)
        : Polyline(std::move(points))
    {
    }

    virtual bool addClosingSegment() const override
    {
        return false; // Definitely not
    }

    virtual size_t segmentsCount() const override
    {
        return size() > 1 ? size() - 1 : 0;
    }

    OpenPolyline& operator=(OpenPolyline&& other)
    {
        Polyline::operator=(std::move(other));
        return *this;
    }

    OpenPolyline& operator=(const OpenPolyline& other)
    {
        Polyline::operator=(other);
        return *this;
    }
};

} // namespace cura

#endif // GEOMETRY_OPEN_POLYLINE_H
