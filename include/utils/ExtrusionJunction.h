// Copyright (c) 2021 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.


#ifndef UTILS_EXTRUSION_JUNCTION_H
#define UTILS_EXTRUSION_JUNCTION_H

#include "geometry/Point2LL.h"

namespace cura
{

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
    Point2LL p_;

    /*!
     * The width of the extruded path at this junction.
     */
    coord_t w_;

    /*!
     * Which perimeter this junction is part of.
     *
     * Perimeters are counted from the outside inwards. The outer wall has index
     * 0.
     */
    size_t perimeter_index_;

    ExtrusionJunction(const Point2LL p, const coord_t w, const coord_t perimeter_index);

    bool operator==(const ExtrusionJunction& other) const;
};

inline Point2LL operator-(const ExtrusionJunction& a, const ExtrusionJunction& b)
{
    return a.p_ - b.p_;
}

// Identity function, used to be able to make templated algorithms that do their operations on 'point-like' input.
inline const Point2LL& make_point(const ExtrusionJunction& ej)
{
    return ej.p_;
}

using LineJunctions = std::vector<ExtrusionJunction>; //<! The junctions along a line without further information. See \ref ExtrusionLine for a more extensive class.

} // namespace cura
#endif // UTILS_EXTRUSION_JUNCTION_H
