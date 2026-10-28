// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "geometry/Point2LL.h"

namespace curaengine::utils
{

/*!
 * A point within a polygon and the index of which segment in the polygon the point lies on.
 */
struct GivenDistPoint
{
    geometry::Point2LL location; //!< Result location
    int pos; //!< Index to the first point in the polygon of the line segment on which the result was found
};

} // namespace curaengine::utils
