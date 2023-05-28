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

template<utils::point P, winding Direction, template<class> class Container>
struct closed_path : public point_container<P, true, Direction, false, Container>
{
    constexpr closed_path() noexcept = default;
    constexpr closed_path(std::initializer_list<P> points) noexcept : point_container<P, true, Direction, false, Container>(points)
    {
    }

    closed_path(const std::vector<Point>& poly) : point_container<P, true, Direction, false, Container>(poly)
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

template<utils::point P = Point, template<class> class Container = std::vector>
struct filled_path_outer : public filled_path<P, winding::CW, Container>
{
    constexpr filled_path_outer() noexcept = default;
    constexpr filled_path_outer(std::initializer_list<P> points) noexcept : filled_path<P, winding::CW, Container>(points)
    {
    }

    filled_path_outer(const std::vector<Point>& poly) : point_container<P, true, winding::CW, false, Container>(poly)
    {
        // TODO: Remove once we finally get rid of Polygon
    }
};

template<utils::point P = Point, template<class> class Container = std::vector>
struct filled_path_inner : public filled_path<P, winding::CCW, Container>
{
    constexpr filled_path_inner() noexcept = default;
    constexpr filled_path_inner(std::initializer_list<P> points) noexcept : filled_path<P, winding::CCW, Container>(points)
    {
    }

    filled_path_inner(const std::vector<Point>& poly) : point_container<P, true, winding::CCW, false, Container>(poly)
    {
        // TODO: Remove once we finally get rid of Polygon
    }
};

template<utils::point P = Point, template<class> class Container = std::vector>
struct polygon
{
    filled_path_outer<P, Container> outer;
    Container<filled_path_inner<P, Container>> inners;

    constexpr polygon() noexcept = default;

    polygon(const std::vector<std::vector<Point>>& poly) : outer(filled_path_outer<P, Container>(ranges::front(poly)))
    {
        // TODO: remove once we finally get rid of Polygon
        auto holes = ranges::views::drop(poly, 1);
        inners = Container<filled_path_inner<P, Container>>(ranges::begin(holes), ranges::end(holes));
    }

    explicit operator Polygons() const
    {
        // TODO: Remove once we finally get rid of Polygons
        Polygons result;
        result.reserve(1 + inners.size());
        result.emplace_back(static_cast<Polygon>(outer));
        for (const auto& inner : inners)
        {
            result.emplace_back(static_cast<Polygon>(inner));
        }
        return result;
    }
};

template<utils::point P = Point, template<class> class Container = std::vector>
struct polygons : public Container<polygon<P, Container>>
{
    constexpr polygons() noexcept = default;
    constexpr polygons(std::initializer_list<polygon<P, Container>> polygons) noexcept : Container<polygon<P, Container>>(polygons)
    {
    }
};

template<utils::point P = Point, template<class> class Container = std::vector>
struct polytree
{
    Container<polygon<P, Container>> polygons;
    Container<closed_path<P, winding::NA, Container>> closed_paths;
    Container<open_path<P, Container>> open_paths;
};

} // namespace cura::geometry

#endif // GEOMETRY_POINT_CONTAINER_H
