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
The bounding_box view converts an input range of `T` into an output range of `U` by calling the Projection on every
element of the input range.

## Example
\code{.cpp}
auto bounding_box_view = all_paths | views::bounding_box_view(&ExtrusionLine::toPolygon) | ranges::views::sliding(2);
for (auto sub_range : bounding_box_view)
{
    if (sub_range[0].contains(sub_range[1])
        ... // Do something if the next extrusion line falls within the previous extrusion line
}
\endcode

## Syntax
```cpp
auto output_range = input_range | ranges::views::bounding_box(&ExtrusionLine::toPolygon);
```

## Parameters
\param proj Projection used to obtain the polygon(s), defaults to `std::identity`

<pre><b>input_range</b></pre>
  - The range of elements to transform
  - Reference type: `T`

<pre><b>output_range</b></pre>
  - The range of output values
  - Reference type: `U`
  - Value type: `decay_t<U>`
  - This range will have the same category as the input range (excluding
  contiguous ranges). Contiguous ranges are reduced to random access ranges.
*/
constexpr auto bounding_box(auto&& proj = ranges::identity{})
{
    return ranges::make_view_closure(ranges::views::transform([proj](auto item) { return AABB{ std::invoke(proj, item) }; }));
}
} // namespace cura::views

#endif // UTILS_VIEWS_BOUNDING_BOX_H