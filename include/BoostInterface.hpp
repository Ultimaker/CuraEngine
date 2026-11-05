// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <boost/polygon/polygon.hpp>

#include "types/geometry/Point2LL.h"
#include "types/geometry/PolygonsSegmentIndex.h"


namespace curaengine
{

using CSegment = types::geometry::PolygonsSegmentIndex;
using CPolygon = boost::polygon::polygon_data<types::coord_t>;
using CPolygonSet = std::vector<CPolygon>;

} // namespace curaengine

namespace boost
{
namespace polygon
{


template<>
struct geometry_concept<::curaengine::types::geometry::Point2LL>
{
    typedef point_concept type;
};

template<>
struct point_traits<::curaengine::types::geometry::Point2LL>
{
    typedef ::curaengine::types::coord_t coordinate_type;

    static inline coordinate_type get(const ::curaengine::types::geometry::Point2LL& point, orientation_2d orient)
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
    typedef ::curaengine::types::coord_t coordinate_type;
    typedef ::curaengine::types::geometry::Point2LL point_type;
    static inline point_type get(const curaengine::CSegment& CSegment, direction_1d dir)
    {
        return dir.to_int() ? CSegment.p() : CSegment.next().p();
    }
};


} // namespace polygon
} // namespace boost
