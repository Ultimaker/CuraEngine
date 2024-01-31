// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/ExtrusionLine.h"

#include <algorithm>

#include "utils/Simplify.h"
#include "utils/linearAlg2D.h"

namespace cura
{


coord_t ExtrusionLine::getLength() const
{
    if (junctions_.empty())
    {
        return 0;
    }
    coord_t len = 0;
    ExtrusionJunction prev = junctions_.front();
    for (const ExtrusionJunction& next : junctions_)
    {
        len += vSize(next.p_ - prev.p_);
        prev = next;
    }
    if (is_closed_)
    {
        len += vSize(front().p_ - back().p_);
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

} // namespace cura
