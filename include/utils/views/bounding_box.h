// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_VIEWS_BOUNDING_BOX_H
#define UTILS_VIEWS_BOUNDING_BOX_H

#include <type_traits>

#include "utils/AABB.h"
#include "utils/views/convert.h"


namespace cura::views
{
/* # cura::views::bounding_box
 * The convert view converts an input range of `T` into an output range of `U` by calling the Projection on every element of the input range.
 * and convert them to `V`
 *
 * ## Syntax
 * ```cpp
 * std::vector<ExtrusionLine*> input_range { ... };
 * auto output_range = input_range | ranges::views::convert<Polygons>(&ExtrusionLine::toPolygon);
 * ```
 * or
 * ```cpp
 * std::vector<Polygon> input_range { ... };
 * auto output_range = input_range | ranges::views::convert<Polygons>(ranges::identity{});
 * ```
 *
 * ## Parameters
 * \param proj Projection used to obtain the elements
 *
 * \returns a transformed view with elements of type `AABB`
 */
constexpr auto bounding_box(auto&& proj)
{
    return convert<AABB>(std::forward<decltype(proj)>(proj));
}
} // namespace cura::views

#endif // UTILS_VIEWS_BOUNDING_BOX_H