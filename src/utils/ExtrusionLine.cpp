// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/ExtrusionLine.h"

#include <algorithm>

#include "utils/LinearAlg2D.h"
#include "utils/Simplify.h"

namespace curaengine::utils
{


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
        len += ::curaengine::geometry::vSize(next.p_ - prev.p_);
        prev = next;
    }
    if (is_closed_)
    {
        len += ::curaengine::geometry::vSize(front().p_ - back().p_);
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
        length += ::curaengine::geometry::vSize(*p0 - p1);
        if (length >= check_length)
        {
            return false;
        }
        p0 = &p1;
    }
    return true;
}

} // namespace curaengine::utils
