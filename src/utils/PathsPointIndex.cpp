// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/PathsPointIndex.h"

#include "geometry/Shape.h"

namespace cura
{

template<>
const Polygon& PathsPointIndex<Shape>::getPolygon() const
{
    return (*polygons_)[poly_idx_];
}

} // namespace cura
