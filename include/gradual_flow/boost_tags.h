// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GRADUAL_FLOW_BOOST_TAGS_H
#define GRADUAL_FLOW_BOOST_TAGS_H


#ifndef INFILL_BOOST_TAGS_H
#define INFILL_BOOST_TAGS_H

#include <polyclipping/clipper.hpp>
#include <vector>

#include <boost/geometry/core/closure.hpp>
#include <boost/geometry/core/point_order.hpp>
#include <boost/geometry/core/tags.hpp>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/strategies/cartesian.hpp>

#include "gradual_flow/concepts.h"
#include "gradual_flow/point_container.h"

namespace boost::geometry::traits
{

template<>
struct tag<ClipperLib::IntPoint>
{
    using type = point_tag;
};

template<>
struct dimension<ClipperLib::IntPoint> : boost::mpl::int_<2>
{
};

template<>
struct coordinate_type<ClipperLib::IntPoint>
{
    using type = ClipperLib::cInt;
};

template<>
struct coordinate_system<ClipperLib::IntPoint>
{
    using type = boost::geometry::cs::cartesian;
};

template<std::size_t Index>
struct access<ClipperLib::IntPoint, Index>
{
    static_assert(Index < 2, "Out of range");
    using Point = ClipperLib::IntPoint;
    using CoordinateType = typename coordinate_type<Point>::type;
    constexpr static inline CoordinateType get(Point const& p) noexcept
    {
        return Index == 0 ? p.X : p.Y;
    }

    constexpr static inline void set(Point& p, CoordinateType const& value) noexcept
    {
        if (Index == 0)
        {
            p.X = value;
        }
        else
        {
            p.Y = value;
        }
    }
};

template<>
struct tag<plugin::gradual_flow::geometry::polyline<>>
{
    using type = linestring_tag;
};

template<>
struct point_order<plugin::gradual_flow::geometry::polygon_outer<>>
{
    static const order_selector value = clockwise;
};

template<>
struct closure<plugin::gradual_flow::geometry::polygon_outer<>>
{
    static const closure_selector value = open;
};

template<>
struct tag<plugin::gradual_flow::geometry::polygon_outer<>>
{
    using type = ring_tag;
};

template<>
struct point_order<plugin::gradual_flow::geometry::polygon_inner<>>
{
    static const order_selector value = counterclockwise;
};

template<>
struct closure<plugin::gradual_flow::geometry::polygon_inner<>>
{
    static const closure_selector value = open;
};

template<>
struct tag<plugin::gradual_flow::geometry::polygon_inner<>>
{
    using type = ring_tag;
};

template<>
struct point_order<ClipperLib::Path>
{
    static const order_selector value = counterclockwise;
};

template<>
struct closure<ClipperLib::Path>
{
    static const closure_selector value = open;
};

template<>
struct tag<ClipperLib::Path>
{
    using type = ring_tag;
};

} // namespace boost::geometry::traits

#endif // GRADUAL_FLOW_BOOST_TAGS_H
