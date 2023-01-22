// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef INCLUDE_UTILS_VIEWS_COORD_H
#define INCLUDE_UTILS_VIEWS_COORD_VIEW_H

#include "utils/views/coord.h"

namespace cura::views
{
constexpr auto flatten_point_view = ranges::make_view_closure( ranges::views::transform( [](const point auto& point) { return coord_view( point ); } ));
} // namespace cura::views

#endif //INCLUDE_UTILS_VIEWS_COORD_H
