// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "geometry/ClosedPolyline.h"

#include <range/v3/algorithm/all_of.hpp>
#include <range/v3/algorithm/rotate.hpp>

#include "geometry/OpenPolyline.h"

namespace cura
{

size_t ClosedPolyline::segmentsCount() const
{
    if (explicitly_closed_)
    {
        return size() >= 3 ? size() - 1 : 0;
    }
    return size() >= 2 ? size() : 0;
}

bool ClosedPolyline::isValid() const
{
    return size() >= (explicitly_closed_ ? 4 : 3);
}

bool ClosedPolyline::inside(const Point2LL& p, bool border_result) const
{
    int res = ClipperLib::PointInPolygon(p, getPoints());
    if (res == -1)
    {
        return border_result;
    }
    return res == 1;
}

bool ClosedPolyline::inside(const ClipperLib::Path& polygon) const
{
    return ranges::all_of(
        *this,
        [&polygon](const auto& point)
        {
            return ClipperLib::PointInPolygon(point, polygon);
        });
}

OpenPolyline ClosedPolyline::toPseudoOpenPolyline() const
{
    OpenPolyline open_polyline(getPoints());
    if (hasClosingSegment())
    {
        open_polyline.push_back(open_polyline.getPoints().front());
    }
    return open_polyline;
}

void ClosedPolyline::shiftVerticesToStartPoint(const size_t start_index)
{
    if (! isValid() || start_index == 0 || start_index >= size())
    {
        return;
    }

    if (explicitly_closed_)
    {
        getPoints().erase(getPoints().end() - 1);
    }

    ranges::rotate(getPoints(), getPoints().begin() + start_index);

    if (explicitly_closed_)
    {
        insert(end(), getPoints().front());
    }
}

} // namespace cura
