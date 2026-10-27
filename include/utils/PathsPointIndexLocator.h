// Copyright (c) 2018 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "utils/PathsPointIndex.h"

namespace cura
{

/*!
 * Locator of a \ref PolygonsPointIndex
 */
template<typename Paths>
struct PathsPointIndexLocator
{
    Point2LL operator()(const PathsPointIndex<Paths>& val) const
    {
        return make_point(val.p());
    }
};

} // namespace cura
