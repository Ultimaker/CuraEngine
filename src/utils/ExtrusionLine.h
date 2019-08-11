//Copyright (c) 2019 Ultimaker B.V.


#ifndef UTILS_EXTRUSION_LINE_H
#define UTILS_EXTRUSION_LINE_H

#include "ExtrusionJunction.h"

namespace arachne
{

struct ExtrusionLine
{
    coord_t inset_idx;
    bool is_odd;
    std::list<ExtrusionJunction> junctions;
    ExtrusionLine(coord_t inset_idx, bool is_odd)
    : inset_idx(inset_idx)
    , is_odd(is_odd)
    {}
    coord_t computeLength()
    {
        if (junctions.size() <= 1) return 0;
        coord_t len = 0;
        ExtrusionJunction prev = junctions.front();
        for (ExtrusionJunction& next : junctions)
        {
            len += vSize(next.p - prev.p);
            prev = next;
        }
        return len;
    }
};


} // namespace arachne
#endif // UTILS_EXTRUSION_LINE_H
