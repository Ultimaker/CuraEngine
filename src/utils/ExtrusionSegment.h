//Copyright (c) 2019 Ultimaker B.V.


#ifndef UTILS_EXTRUSION_SEGMENT_H
#define UTILS_EXTRUSION_SEGMENT_H

#include <utility>

#include "utils/IntPoint.h"
#include "utils/polygon.h"
#include "utils/polygonUtils.h"
#include "utils/ExtrusionJunction.h"

namespace arachne
{

/*!
 * extrusion bead
 * suports varying width
 */
class ExtrusionSegment
{
    static constexpr float a_step = 15 / 180.0 * M_PI;
public:
    ExtrusionJunction from;
    ExtrusionJunction to;

    bool is_odd; //!< Whether this is a polyline segment rather than a polygonal segment
    bool is_reduced; //!< Whether to remove the circle from the to-location because it will be included in the next extrusion move

    ExtrusionSegment(ExtrusionJunction from, ExtrusionJunction to, bool is_odd, bool is_reduced)
    : from(from)
    , to(to)
    , is_odd(is_odd)
    , is_reduced(is_reduced)
    {}

    Polygons toPolygons(); 

    /*!
     * \param reduced Whether to remove the circle from the to-location because it will be included in the next extrusion move
     * 
     * Overrides class field is_reduced
     */
    Polygons toPolygons(bool reduced);

    std::vector<ExtrusionSegment> discretize(coord_t step_size);
};




} // namespace arachne
#endif // UTILS_EXTRUSION_SEGMENT_H
