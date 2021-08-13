//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.


#ifndef UTILS_EXTRUSION_JUNCTION_H
#define UTILS_EXTRUSION_JUNCTION_H

#include "IntPoint.h"

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

    /*!
     * Which region this juntion is part of. A solid polygon without holes has only one region.
     * A polygon with holes has 2. Disconnected parts of the polygon are also separate regions.
     * Will be 0 if no region was given.
     */
    size_t region_id;

    ExtrusionJunction(const Point p, const coord_t w, const coord_t perimeter_index, const size_t region_id = 0);

    bool operator==(const ExtrusionJunction& other) const;
};


} // namespace cura

namespace cura
{
    inline Point operator-(const ExtrusionJunction& a, const ExtrusionJunction& b)
    {
        return a.p - b.p;
    }

    // Identity function, used to be able to make templated algorithms that do their operations on 'point-like' input.
    inline const Point& make_point(const ExtrusionJunction& ej)
    {
        return ej.p;
    }

    using LineJunctions = std::vector<ExtrusionJunction>; //<! Vector of Lines
    using PathJunctions = std::vector<LineJunctions>; //<! Vector of paths
    using BinJunctions = std::vector<PathJunctions>; //<! Vector of insets (bins)
}
#endif // UTILS_EXTRUSION_JUNCTION_H
