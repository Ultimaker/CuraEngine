// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "PathsPointIndex.h"

namespace curaengine::utils
{

/*!
 * A class for iterating over the points in one of the polygons in a \ref Polygons object
 */
class PolygonsSegmentIndex : public PolygonsPointIndex
{
public:
    PolygonsSegmentIndex();
    PolygonsSegmentIndex(const geometry::Shape* polygons, unsigned int poly_idx, unsigned int point_idx);

    geometry::Point2LL from() const;

    geometry::Point2LL to() const;
};


} // namespace curaengine::utils
