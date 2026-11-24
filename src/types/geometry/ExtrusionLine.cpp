// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/geometry/ExtrusionLine.h"

#include <algorithm>
#include <iterator>

#include <range/v3/algorithm/rotate.hpp>
#include <spdlog/spdlog.h>

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

void ExtrusionLine::setStartJunction(const size_t new_start_junction)
{
    if (! is_closed_)
    {
        spdlog::error("Can not change the start junction of a non-closed extrusion line");
        return;
    }

    // Closed lines are usually explicitly closed, which means that the last junction is a copy of the first one
    const bool is_explicitly_closed = size() > 1 && front().p_ == back().p_;
    const size_t unique_junctions_count = is_explicitly_closed ? size() - 1 : size();
    const size_t start_index = new_start_junction % unique_junctions_count;

    if (start_index == 0)
    {
        return;
    }

    if (is_explicitly_closed)
    {
        if (front().w_ != back().w_)
        {
            spdlog::warn("The first and last junctions of the extrusion line have different widths, the new closing junction will take the width of the new start junction");
        }

        junctions_.pop_back();
    }

    // Re-order the toolpath so that it starts with the most appropriate seam position
    ranges::rotate(*this, std::next(begin(), start_index));

    if (is_explicitly_closed)
    {
        // Close the line again, so that it ends where it starts
        junctions_.push_back(junctions_.front());
    }
}

} // namespace curaengine::types::geometry
