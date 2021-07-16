//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ExtrusionJunction.h"

namespace cura
{

bool ExtrusionJunction::operator ==(const ExtrusionJunction& other) const
{
    return p == other.p
        && w == other.w
        && perimeter_index == other.perimeter_index;
}

}