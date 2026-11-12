// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "types/geometry/PathsPointIndex.h"

namespace curaengine::types::geometry
{

/*!
 * Locator of a \ref PolygonsPointIndex
 */
template<typename Paths>
struct PathsPointIndexLocator
{
    types::geometry::Point2LL operator()(const PathsPointIndex<Paths>& val) const
    {
        using types::geometry::make_point;
        return make_point(val.p());
    }
};

} // namespace curaengine::types::geometry
