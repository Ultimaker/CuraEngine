// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/geometry/PolygonsSegmentIndex.h"

namespace curaengine::types::geometry
{

using namespace types;
using namespace utils;

PolygonsSegmentIndex::PolygonsSegmentIndex()
    : PolygonsPointIndex()
{
}

PolygonsSegmentIndex::PolygonsSegmentIndex(const Shape* polygons, unsigned int poly_idx, unsigned int point_idx)
    : PolygonsPointIndex(polygons, poly_idx, point_idx)
{
}

Point2LL PolygonsSegmentIndex::from() const
{
    return PolygonsPointIndex::p();
}

Point2LL PolygonsSegmentIndex::to() const
{
    return PolygonsSegmentIndex::next().p();
}

} // namespace curaengine::types::geometry
