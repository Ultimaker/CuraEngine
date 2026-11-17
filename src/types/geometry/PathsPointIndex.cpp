// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/geometry/PathsPointIndex.h"

#include "types/geometry/Shape.h"

namespace curaengine::types::geometry
{

using namespace types;
using namespace utils;

template<>
const Polygon& PathsPointIndex<Shape>::getPolygon() const
{
    return (*polygons_)[poly_idx_];
}

} // namespace curaengine::types::geometry
