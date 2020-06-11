//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ExtrusionLine.h"

namespace arachne
{

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
    return len;
}

}