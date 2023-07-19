// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "utils/PolygonsPointIndex.h"

namespace cura
{

template<>
ConstPolygonRef PathsPointIndex<Polygons>::getPolygon() const
{
    return (*polygons)[poly_idx];
}

} // namespace cura
