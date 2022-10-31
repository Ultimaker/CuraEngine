// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef MAPPED_BOUNDING_BOX_H
#define MAPPED_BOUNDING_BOX_H

#include "utils/views/bounding_box.h"

#include <range/v3/view/transform.hpp>
#include <range/v3/view/view.hpp>
#include <range/v3/view/zip.hpp>

namespace cura::views
{
/* # cura::views::mapped_bounding_box
 * The mapped_bounding_box view converts an input range of `T` into a zipped output range of `U` by calling the Projection on every
 * element of the input range to produce a boundary box of the polygon(s) from that element and a pointer to the element
 *
 * ## Syntax
 * ```cpp
 * auto output_range = ranges::views::mapped_bounding_box(input_range, &ExtrusionLine::toPolygon);
 * ```
 *
 * ## Parameters
 * \param rng the input range of elements of reference type: `T` to tranform into a `std::tuple<T::value_type*, AABB>`
 * \param proj Projection used to obtain the polygon(s), defaults to `std::identity`
 *
 * \returns a zipped view of elements of `std::tuple<T::value_type*, AABB>`
 */
auto mapped_bounding_box(auto&& rng, auto&& proj = ranges::identity{})
{
    auto bounding_box_view = rng | bounding_box(std::forward<decltype(proj)>(proj));
    auto pointer_view = rng | ranges::views::transform([](const auto& value) { return &value; });
    return ranges::make_view_closure(ranges::views::zip(pointer_view, bounding_box_view));
}

} // namespace cura::views

#endif // MAPPED_BOUNDING_BOX_H
