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

ExtrusionJunction::ExtrusionJunction(const Point p, const coord_t w, const coord_t perimeter_index, const size_t region_id)
    : p(p),
      w(w),
      perimeter_index(perimeter_index),
      region_id(region_id)
{}

void ExtrusionJunction::Swap(ExtrusionJunction* other) {
  if (other == this) return;
  InternalSwap(other);
}
void ExtrusionJunction::InternalSwap(ExtrusionJunction* other) {
  using std::swap;

  swap(p, other->p);
  swap(w, other->w);
  swap(perimeter_index, other->perimeter_index);
  swap(region_id, other->region_id);
}

}
