// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_GEOMETRY_BOOST_TAGS_H
#define UTILS_GEOMETRY_BOOST_TAGS_H

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/core/closure.hpp>
#include <boost/geometry/core/point_order.hpp>
#include <boost/geometry/core/tags.hpp>
#include <boost/geometry/strategies/cartesian.hpp>

#include "utils/geometry/point_container.h"

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
    static inline CoordinateType get(Point const& p)
    {
        return Index == 0 ? p.X : p.Y;
    }
    static inline void set(Point& p, CoordinateType const& value)
    {
        if (Index == 0)
            p.X = value;
        else
            p.Y = value;
    }
};

template<cura::concepts::point P, bool IsClosed, cura::direction Direction, template<class> class Container>
struct tag<cura::geometry::point_container<P, IsClosed, Direction, Container>>
{
    using type = linestring_tag;
};

template<cura::concepts::point P, bool IsClosed, cura::direction Direction, template<class> class Container>
struct point_order<cura::geometry::point_container<P, IsClosed, Direction, Container>>
{
    static const order_selector value = Direction == cura::direction::CW ? clockwise : counterclockwise;
};

template<cura::concepts::point P, bool IsClosed, cura::direction Direction, template<class> class Container>
struct closure<cura::geometry::point_container<P, IsClosed, Direction, Container>>
{
    static const closure_selector value = IsClosed ? closed : open;
};

template<>
struct tag<cura::geometry::polygon_outer<ClipperLib::IntPoint, std::vector>>
{
    using type = linestring_tag;
};
} // namespace boost::geometry::traits

#endif // UTILS_GEOMETRY_BOOST_TAGS_H
