//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef BOOST_INTERFACE_HPP
#define BOOST_INTERFACE_HPP

#include <boost/polygon/voronoi.hpp>
#include <boost/polygon/polygon.hpp>

#include "utils/IntPoint.h"
#include "utils/PolygonsSegmentIndex.h"
#include "utils/polygon.h"


using CSegment = cura::PolygonsSegmentIndex;
using CPolygon = boost::polygon::polygon_data<cura::coord_t>;
using CPolygonSet = std::vector<CPolygon>;

namespace boost {
namespace polygon {


template <>
struct geometry_concept<cura::Point>
{
    typedef point_concept type;
};

template <>
struct point_traits<cura::Point>
{
    typedef cura::coord_t coordinate_type;

    static inline coordinate_type get(
            const cura::Point& point, orientation_2d orient)
    {
        return (orient == HORIZONTAL) ? point.X : point.Y;
    }
};

template <>
struct geometry_concept<CSegment>
{
    typedef segment_concept type;
};

template <>
struct segment_traits<CSegment>
{
    typedef cura::coord_t coordinate_type;
    typedef cura::Point point_type;
    static inline point_type get(const CSegment& CSegment, direction_1d dir) {
        return dir.to_int() ? CSegment.p() : CSegment.next().p();
    }
};



}    // polygon
}    // boost

#endif // BOOST_INTERFACE_HPP
