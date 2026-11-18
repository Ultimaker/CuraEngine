// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/geometry/ExtrusionLine.h"

#include <algorithm>
#include <iterator>

#include "types/geometry/Simplify.h"
#include "utils/linear_alg_2d.h"
#include "utils/math.h"

namespace curaengine::types::geometry
{

using namespace types;
using namespace utils;


coord_t ExtrusionLine::length() const
{
    if (junctions_.empty())
    {
        return 0;
    }
    coord_t len = 0;
    ExtrusionJunction prev = junctions_.front();
    for (const ExtrusionJunction& next : junctions_)
    {
        len += types::geometry::vSize(next.p_ - prev.p_);
        prev = next;
    }
    if (is_closed_)
    {
        len += types::geometry::vSize(front().p_ - back().p_);
    }
    return len;
}

coord_t ExtrusionLine::getMinimalWidth() const
{
    return std::min_element(
               junctions_.cbegin(),
               junctions_.cend(),
               [](const ExtrusionJunction& l, const ExtrusionJunction& r)
               {
                   return l.w_ < r.w_;
               })
        ->w_;
}

bool ExtrusionLine::shorterThan(const coord_t check_length) const
{
    const ExtrusionJunction* p0 = &back();
    int64_t length = 0;
    for (const ExtrusionJunction& p1 : (*this))
    {
        length += types::geometry::vSize(*p0 - p1);
        if (length >= check_length)
        {
            return false;
        }
        p0 = &p1;
    }
    return true;
}

size_t ExtrusionLine::insertJunction(const size_t after_index, const Point2LL& position)
{
    const size_t insert_index = (after_index + 1) % size();
    const ExtrusionJunction& junction_before = junctions_[after_index];
    const ExtrusionJunction& junction_after = junctions_[insert_index];

    const coord_t extrusion_size = vSize(junction_after.p_ - junction_before.p_);
    const coord_t before_to_position = vSize(position - junction_before.p_);
    const double position_factor = inverse_lerp(coord_t{ 0 }, extrusion_size, before_to_position);
    const coord_t width = lerp(junction_before.w_, junction_after.w_, position_factor);

    emplace(std::next(begin(), insert_index), position, width, junction_before.perimeter_index_);
    return insert_index;
}

} // namespace curaengine::types::geometry
