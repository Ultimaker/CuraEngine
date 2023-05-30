// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_GEOMETRY_BOOST_TAGS_H
#define UTILS_GEOMETRY_BOOST_TAGS_H

#include <boost/geometry/core/closure.hpp>
#include <boost/geometry/core/point_order.hpp>
#include <boost/geometry/core/tags.hpp>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/strategies/cartesian.hpp>

#include "geometry/point_container.h"
#include "geometry/winding.h"
#include "utils/types/geometry.h"

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
struct tag<cura::geometry::open_path<ClipperLib::IntPoint, std::vector>>
{
    using type = linestring_tag;
};

template<>
struct point_order<cura::geometry::closed_path<ClipperLib::IntPoint, cura::geometry::winding::CW, std::vector>>
{
    static const order_selector value = clockwise;
};

template<>
struct closure<cura::geometry::closed_path<ClipperLib::IntPoint, cura::geometry::winding::CW, std::vector>>
{
    static const closure_selector value = open; // TODO: closed?, but when it is closed the order of the points changes
};

template<>
struct tag<cura::geometry::closed_path<ClipperLib::IntPoint, cura::geometry::winding::CW, std::vector>>
{
    using type = ring_tag;
};

template<>
struct point_order<cura::geometry::closed_path<ClipperLib::IntPoint, cura::geometry::winding::NA, std::vector>>
{
    static const order_selector value = clockwise;
};

template<>
struct closure<cura::geometry::closed_path<ClipperLib::IntPoint, cura::geometry::winding::NA, std::vector>>
{
    static const closure_selector value = open; // TODO: closed?, but when it is closed the order of the points changes
};

template<>
struct tag<cura::geometry::closed_path<ClipperLib::IntPoint, cura::geometry::winding::NA, std::vector>>
{
    using type = ring_tag;
};

template<>
struct point_order<cura::geometry::closed_path<ClipperLib::IntPoint, cura::geometry::winding::CCW, std::vector>>
{
    static const order_selector value = counterclockwise;
};

template<>
struct closure<cura::geometry::closed_path<ClipperLib::IntPoint, cura::geometry::winding::CCW, std::vector>>
{
    static const closure_selector value = open; // TODO: closed?, but when it is closed the order of the points changes
};

template<>
struct tag<cura::geometry::closed_path<ClipperLib::IntPoint, cura::geometry::winding::CCW, std::vector>>
{
    using type = ring_tag;
};

} // namespace boost::geometry::traits

#endif // UTILS_GEOMETRY_BOOST_TAGS_H
