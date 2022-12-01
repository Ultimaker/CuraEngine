// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_VIEWS_CONVERT_H
#define UTILS_VIEWS_CONVERT_H

#include <type_traits>

#include <range/v3/view/transform.hpp>
#include <range/v3/view/view.hpp>

namespace cura::views
{
/* # cura::views::convert<V>
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
 * std::vector<AABB> input_range { ... };
 * auto output_range = input_range | ranges::views::convert<Polygons>(ranges::identity{});
 * ```
 *
 * ## Parameters
 * \param proj Projection used to obtain the elements
 *
 * \returns a transformed view with elements of type `V`
 */
template<typename V>
constexpr auto convert(auto&& proj)
{
    return ranges::make_view_closure(ranges::views::transform([proj](auto item) { return V { std::invoke(proj, item) }; }));
}
} // namespace cura::views

#endif // UTILS_VIEWS_CONVERT_H
