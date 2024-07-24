// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/PolygonsPointIndex.h"

#include "geometry/Shape.h"

namespace cura
{

template<>
const Polygon& PathsPointIndex<Shape>::getPolygon() const
{
    return (*polygons_)[poly_idx_];
}

std::pair<Point2LL, Point2LL> PolygonsPointIndexSegmentLocator::operator()(const PolygonsPointIndex& val) const
{
    const Polygon& poly = (*val.polygons_)[val.poly_idx_];
    Point2LL start = poly[val.point_idx_];
    size_t next_point_idx = (val.point_idx_ + 1ul) % poly.size();
    Point2LL end = poly[next_point_idx];
    return std::pair<Point2LL, Point2LL>(start, end);
}

} // namespace cura
