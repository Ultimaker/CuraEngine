// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_VIEWS_GET_H
#define UTILS_VIEWS_GET_H

#include <type_traits>

#include <range/v3/view/transform.hpp>
#include <range/v3/view/view.hpp>

namespace cura::views
{
/* # cura::views::get
 * The convert view converts an input range of `T` into an output range of `U` by calling the Projection on every element of the input range.
 *
 * ## Syntax
 * ```cpp
 * std::vector<ExtrusionLine*> input_range { ... };
 * auto output_range = input_range | ranges::views::get(&ExtrusionLine::toPolygon);
 * // Note: in this case it will get a range of ClipperLib::IntPoint
 * ```
 * or
 * ```cpp
 * struct NamedAABB
 * {
 *      AABB box;
 *      std::string name;
 * };
 *
 * std::vector<NamedAABB> input_range { ... };
 * auto output_range = input_range | ranges::views::get(&NamedAABB::box);
 * ```
 *
 * ## Parameters
 * \param proj Projection used to obtain the elements
 *
 * \returns a transformed view with elements of type `T::value_type`
 */
constexpr auto get(auto&& proj)
{
    return ranges::make_view_closure(ranges::views::transform([proj](auto&& item) { return std::invoke(proj, std::forward<decltype(item)>(item)); }));
}
} // namespace cura::views

#endif // UTILS_VIEWS_GET_H
