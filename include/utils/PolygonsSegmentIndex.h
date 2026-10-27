// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "PathsPointIndex.h"

namespace cura
{

/*!
 * A class for iterating over the points in one of the polygons in a \ref Polygons object
 */
class PolygonsSegmentIndex : public PolygonsPointIndex
{
public:
    PolygonsSegmentIndex();
    PolygonsSegmentIndex(const Shape* polygons, unsigned int poly_idx, unsigned int point_idx);

    Point2LL from() const;

    Point2LL to() const;
};


} // namespace cura
