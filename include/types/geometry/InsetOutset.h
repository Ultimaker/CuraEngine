// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <vector>

#include "types/Coord_t.h"
#include "types/geometry/Shape.h"

namespace curaengine::types::geometry
{

/*! Helper structure to return both the generated walls and the width of the final contour, when generating insets/outsets */
struct InsetOutset
{
    std::vector<Shape> walls;
    coord_t final_contour_offset{ 0 };
};

} // namespace curaengine::types::geometry
