// Copyright (c) 2020 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_POLYGONS_SEGMENT_INDEX_H
#define UTILS_POLYGONS_SEGMENT_INDEX_H

#include <vector>

#include "PolygonsPointIndex.h"

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


#endif // UTILS_POLYGONS_SEGMENT_INDEX_H
