//Copyright (c) 2020 Ultimaker B.V.
#ifndef BOOST_INTERFACE_HPP
#define BOOST_INTERFACE_HPP

#include <boost/polygon/voronoi.hpp>

#include "utils/IntPoint.h"
#include "utils/PolygonsSegmentIndex.h"

namespace boost {
namespace polygon {
    
    
using Segment = arachne::PolygonsSegmentIndex;

template <>
struct geometry_concept<arachne::Point>
{
    typedef point_concept type;
};

template <>
struct point_traits<arachne::Point>
{
    typedef int coordinate_type;

    static inline coordinate_type get(
            const arachne::Point& point, orientation_2d orient)
    {
        return (orient == HORIZONTAL) ? point.X : point.Y;
    }
};

template <>
struct geometry_concept<Segment>
{
    typedef segment_concept type;
};

template <>
struct segment_traits<Segment>
{
    typedef arachne::coord_t coordinate_type;
    typedef arachne::Point point_type;
    static inline point_type get(const Segment& segment, direction_1d dir) {
        return dir.to_int() ? segment.p() : segment.next().p();
    }
};
}    // polygon
}    // boost

#endif // BOOST_INTERFACE_HPP
