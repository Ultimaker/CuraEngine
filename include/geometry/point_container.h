// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GEOMETRY_POINT_CONTAINER_H
#define GEOMETRY_POINT_CONTAINER_H

#include <initializer_list>
#include <memory>
#include <vector>

#include <range/v3/range/operations.hpp>
#include <range/v3/view/drop.hpp>
#include <range/v3/view/transform.hpp>

#include "geometry/winding.h"
#include "utils/IntPoint.h"
#include "utils/polygon.h"
#include "utils/types/geometry.h"

namespace cura::geometry
{
/*! The base clase of all point based container types
 *
 * @tparam P
 * @tparam IsClosed
 * @tparam Direction
 * @tparam Container
 */
template<utils::point P, bool IsClosed, winding Direction, bool IsFilled, template<class> class Container>
requires utils::point<typename Container<P>::value_type> struct point_container : public Container<P>
{
    inline static constexpr bool is_filled = IsFilled;
    inline static constexpr bool is_closed = IsClosed;
    inline static constexpr winding direction = Direction;
};

template<utils::point P = Point, template<class> class Container = std::vector>
struct open_path : public point_container<P, false, winding::NA, false, Container>
{
    constexpr open_path() noexcept = default;
    constexpr open_path(std::initializer_list<P> points) noexcept : point_container<P, false, winding::NA, false, Container>(points)
    {
    }
};

template<utils::point P = Point, template<class> class Container = std::vector>
open_path(std::initializer_list<P>) -> open_path<P, Container>;

template<utils::point P, winding Direction, template<class> class Container>
struct closed_path : public point_container<P, true, Direction, false, Container>
{
    constexpr closed_path() noexcept = default;
    constexpr closed_path(std::initializer_list<P> points) noexcept : point_container<P, true, Direction, false, Container>(points)
    {
    }

    constexpr closed_path(const Polygon& polygon) noexcept : point_container<P, true, Direction, false, Container>(polygon.poly)
    {
        // TODO: Remove once we finally get rid of Polygon
    }

    explicit operator Polygon() const
    {
        // TODO: Remove once we finally get rid of Polygon
        Polygon result;
        result.poly = static_cast<std::vector<Point>>(*this);
        return result;
    }
};

template<utils::point P = Point, template<class> class Container = std::vector>
closed_path(std::initializer_list<P>) -> closed_path<P, winding::NA, Container>;
closed_path(Polygon) -> closed_path<Point, winding::NA, std::vector>; // TODO: Remove once we finally get rid of Polygon

template<utils::point P, winding Direction, template<class> class Container>
struct filled_path : public closed_path<P, Direction, Container>
{
    constexpr filled_path() noexcept = default;
    constexpr filled_path(std::initializer_list<P> points) noexcept : closed_path<P, Direction, Container>(points)
    {
    }
};

template<utils::point P = Point, template<class> class Container = std::vector>
filled_path(std::initializer_list<P>) -> filled_path<P, winding::NA, Container>;
filled_path(Polygon) -> filled_path<Point, winding::NA, std::vector>; // TODO: Remove once we finally get rid of Polygon

template<utils::point P = Point, template<class> class Container = std::vector>
struct filled_path_outer : public filled_path<P, winding::CW, Container>
{
    constexpr filled_path_outer() noexcept = default;
    constexpr filled_path_outer(std::initializer_list<P> points) noexcept : filled_path<P, winding::CW, Container>(points)
    {
    }
};

template<utils::point P = Point, template<class> class Container = std::vector>
filled_path_outer(std::initializer_list<P>) -> filled_path_outer<P, Container>;
filled_path_outer(std::vector<Point>) -> filled_path_outer<Point, std::vector>; // TODO: Remove once we finally get rid of Polygon
filled_path_outer(Polygon) -> filled_path_outer<Point, std::vector>; // TODO: Remove once we finally get rid of Polygon

template<utils::point P = Point, template<class> class Container = std::vector>
struct filled_path_inner : public filled_path<P, winding::CCW, Container>
{
    constexpr filled_path_inner() noexcept = default;
    constexpr filled_path_inner(std::initializer_list<P> points) noexcept : filled_path<P, winding::CCW, Container>(points)
    {
    }
};

template<utils::point P = Point, template<class> class Container = std::vector>
filled_path_inner(std::initializer_list<P>) -> filled_path_inner<P, Container>;
filled_path_inner(std::vector<Point>) -> filled_path_inner<Point, std::vector>; // TODO: Remove once we finally get rid of Polygon
filled_path_inner(Polygon) -> filled_path_inner<Point, std::vector>; // TODO: Remove once we finally get rid of Polygon

template<utils::point P, template<class> class Container>
struct ranged_paths : public Container<point_container<P, false, winding::NA, false, Container>>
{
    constexpr ranged_paths() noexcept = default;
    constexpr ranged_paths(std::initializer_list<point_container<P, false, winding::NA, false, Container>> paths) noexcept : Container<point_container<P, false, winding::NA, false, Container>>(paths)
    {
    }

    constexpr ranged_paths(const Polygons& polygons) noexcept
    {
        for (const auto& polygon : polygons)
        {
            this->emplace_back(polygon);
        }
    }
};
ranged_paths(Polygons) -> ranged_paths<Point, std::vector>; // TODO: Remove once we finally get rid of Polygon

} // namespace cura::geometry

#endif // GEOMETRY_POINT_CONTAINER_H
