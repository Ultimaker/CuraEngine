// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <boost/polygon/polygon.hpp>

#include "geometry/Point2LL.h"
#include "utils/PolygonsSegmentIndex.h"


namespace curaengine
{

using CSegment = PolygonsSegmentIndex;
using CPolygon = boost::polygon::polygon_data<coord_t>;
using CPolygonSet = std::vector<CPolygon>;

} // namespace curaengine

namespace boost
{
namespace polygon
{


template<>
struct geometry_concept<curaengine::Point2LL>
{
    typedef point_concept type;
};

template<>
struct point_traits<curaengine::Point2LL>
{
    typedef curaengine::coord_t coordinate_type;

    static inline coordinate_type get(const curaengine::Point2LL& point, orientation_2d orient)
    {
        return (orient == HORIZONTAL) ? point.X : point.Y;
    }
};

template<>
struct geometry_concept<curaengine::CSegment>
{
    typedef segment_concept type;
};

template<>
struct segment_traits<curaengine::CSegment>
{
    typedef curaengine::coord_t coordinate_type;
    typedef curaengine::Point2LL point_type;
    static inline point_type get(const curaengine::CSegment& CSegment, direction_1d dir)
    {
        return dir.to_int() ? CSegment.p() : CSegment.next().p();
    }
};


} // namespace polygon
} // namespace boost
