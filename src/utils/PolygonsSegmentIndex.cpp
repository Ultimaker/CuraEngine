//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/PolygonsSegmentIndex.h"

namespace cura
{

PolygonsSegmentIndex::PolygonsSegmentIndex() : PolygonsPointIndex() {}

PolygonsSegmentIndex::PolygonsSegmentIndex(const Polygons* polygons, unsigned int poly_idx, unsigned int point_idx) : PolygonsPointIndex(polygons, poly_idx, point_idx) {}

Point PolygonsSegmentIndex::from() const
{
    return PolygonsPointIndex::p();
}
    
Point PolygonsSegmentIndex::to() const
{
    return PolygonsSegmentIndex::next().p();
}

}
