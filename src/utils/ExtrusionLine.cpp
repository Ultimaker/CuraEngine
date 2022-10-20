//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <algorithm>

#include "utils/ExtrusionLine.h"
#include "utils/linearAlg2D.h"
#include "utils/Simplify.h"

namespace cura
{

ExtrusionLine::ExtrusionLine(const size_t inset_idx, const bool is_odd)
: inset_idx(inset_idx)
, is_odd(is_odd)
, is_closed(false)
{}

coord_t ExtrusionLine::getLength() const
{
    if (junctions.empty())
    {
        return 0;
    }
    coord_t len = 0;
    ExtrusionJunction prev = junctions.front();
    for (const ExtrusionJunction& next : junctions)
    {
        len += vSize(next.p - prev.p);
        prev = next;
    }
    if (is_closed)
    {
        len += vSize(front().p - back().p);
    }
    return len;
}

coord_t ExtrusionLine::getMinimalWidth() const
{
    return std::min_element(junctions.cbegin(), junctions.cend(),
                            [](const ExtrusionJunction& l, const ExtrusionJunction& r)
                            {
                                return l.w < r.w;
                            })->w;
}

}
