// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/ExtrusionJunction.h"

namespace cura
{

bool ExtrusionJunction::operator==(const ExtrusionJunction& other) const
{
    return p_ == other.p_ && w_ == other.w_ && perimeter_index_ == other.perimeter_index_;
}

ExtrusionJunction::ExtrusionJunction(const Point2LL p, const coord_t w, const coord_t perimeter_index)
    : p_(p)
    , w_(w)
    , perimeter_index_(perimeter_index)
{
}

} // namespace cura
