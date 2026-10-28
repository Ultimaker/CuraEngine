// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/PathsPointIndex.h"

#include "geometry/Shape.h"

namespace curaengine
{

template<>
const Polygon& PathsPointIndex<Shape>::getPolygon() const
{
    return (*polygons_)[poly_idx_];
}

} // namespace curaengine
