// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "utils/ExtrusionJunction.h"

namespace cura
{

bool ExtrusionJunction::operator==(const ExtrusionJunction& other) const
{
    return p == other.p && w == other.w && perimeter_index == other.perimeter_index;
}

ExtrusionJunction::ExtrusionJunction(const Point p, const coord_t w, const coord_t perimeter_index) : p(p), w(w), perimeter_index(perimeter_index)
{
}

} // namespace cura
