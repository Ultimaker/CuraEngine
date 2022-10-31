// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_VIEWS_BOUNDING_BOX_H
#define UTILS_VIEWS_BOUNDING_BOX_H

#include "utils/AABB.h"

#include <type_traits>

#include <range/v3/view/transform.hpp>
#include <range/v3/view/view.hpp>

namespace cura::views
{
/* # cura::views::bounding_box
 * The bounding_box view converts an input range of `T` into an output range of `U` by calling the Projection on every element of the input range.
 *
 * ## Syntax
 * ```cpp
 * auto output_range = input_range | ranges::views::bounding_box(&ExtrusionLine::toPolygon);
 * ```
 *
 * ## Parameters
 * \param proj Projection used to obtain the polygon(s), defaults to `std::identity`
 *
 * \returns a transformed view with the elements of type `AABB`
 */
constexpr auto bounding_box(auto&& proj = ranges::identity{})
{
    return ranges::make_view_closure(ranges::views::transform([proj](auto item) { return AABB{ std::invoke(proj, item) }; }));
}
} // namespace cura::views

#endif // UTILS_VIEWS_BOUNDING_BOX_H