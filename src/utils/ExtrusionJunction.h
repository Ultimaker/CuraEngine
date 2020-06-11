//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.


#ifndef UTILS_EXTRUSION_JUNCTION_H
#define UTILS_EXTRUSION_JUNCTION_H

#include "IntPoint.h"

namespace arachne
{
    using namespace cura;

/*!
 * This struct represents one vertex in an extruded path.
 *
 * It contains information on how wide the extruded path must be at this point,
 * and which perimeter it represents.
 */
struct ExtrusionJunction
{
	/*!
	 * The position of the centreline of the path when it reaches this junction.
	 * This is the position that should end up in the g-code eventually.
	 */
    Point p;

	/*!
	 * The width of the extruded path at this junction.
	 */
    coord_t w;

	/*!
	 * Which perimeter this junction is part of.
	 *
	 * Perimeters are counted from the outside inwards. The outer wall has index
	 * 0.
	 */
    size_t perimeter_index;

    ExtrusionJunction(const Point p, const coord_t w, const coord_t perimeter_index)
    : p(p), w(w), perimeter_index(perimeter_index) {}

    bool operator==(const ExtrusionJunction& other) const
    {
        return p == other.p
            && w == other.w
            && perimeter_index == other.perimeter_index;
    }
};


} // namespace arachne
#endif // UTILS_EXTRUSION_JUNCTION_H
