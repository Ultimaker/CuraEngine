// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "utils/PathsPointIndex.h"

namespace curaengine::utils
{

/*!
 * Locator of a \ref PolygonsPointIndex
 */
template<typename Paths>
struct PathsPointIndexLocator
{
    ::curaengine::geometry::Point2LL operator()(const PathsPointIndex<Paths>& val) const
    {
        using ::curaengine::geometry::make_point;
        return make_point(val.p());
    }
};

} // namespace curaengine::utils
