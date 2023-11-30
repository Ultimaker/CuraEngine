// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/PolygonsPointIndex.h"

namespace cura
{

template<>
ConstPolygonRef PathsPointIndex<Polygons>::getPolygon() const
{
    return polygons_->at(poly_idx_);
}

} // namespace cura
