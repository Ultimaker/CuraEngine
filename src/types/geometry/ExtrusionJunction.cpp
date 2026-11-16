// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "types/geometry/ExtrusionJunction.h"

namespace curaengine::types::geometry
{

using namespace types;
using namespace utils;

bool ExtrusionJunction::operator==(const ExtrusionJunction& other) const
{
    return p_ == other.p_ && w_ == other.w_ && perimeter_index_ == other.perimeter_index_;
}

ExtrusionJunction::ExtrusionJunction(const Point2LL& p, const coord_t w, const coord_t perimeter_index)
    : p_(p)
    , w_(w)
    , perimeter_index_(perimeter_index)
{
}

} // namespace curaengine::types::geometry
