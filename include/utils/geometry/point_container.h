// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_GEOMETRY_POINT_CONTAINER_H
#define UTILS_GEOMETRY_POINT_CONTAINER_H

#include <initializer_list>
#include <memory>
#include <vector>

#include <range/v3/view/drop.hpp>
#include <range/v3/view/transform.hpp>

#include "utils/IntPoint.h"
#include "utils/concepts/geometry.h"

namespace cura::geometry
{


/*! The base clase of all point based container types
 *
 * @tparam P
 * @tparam IsClosed
 * @tparam Direction
 * @tparam Container
 */
template<concepts::point P, bool IsClosed, direction Direction, template<class> class Container>
requires concepts::point<typename Container<P>::value_type> struct point_container : public Container<P>
{
    inline static constexpr bool is_closed = IsClosed;
    inline static constexpr direction winding = Direction;
};

template<concepts::point P = Point, template<class> class Container = std::vector>
struct polyline : public point_container<P, false, direction::NA, Container>
{
    constexpr polyline() noexcept = default;
    constexpr explicit polyline(std::initializer_list<P> points) noexcept : point_container<P, false, direction::NA, Container>(points)
    {
    }
};

template<concepts::point P, direction Direction, template<class> class Container>
struct polygon : public point_container<P, true, Direction, Container>
{
    constexpr polygon() noexcept = default;
    constexpr polygon(std::initializer_list<P> points) noexcept : point_container<P, true, Direction, Container>(points)
    {
    }
};

template<concepts::point P = Point, template<class> class Container = std::vector>
polygon(std::initializer_list<P>)->polygon<P, direction::NA, Container>;

template<concepts::point P = Point, template<class> class Container = std::vector>
struct polygon_outer : public point_container<P, true, direction::CW, Container>
{
    constexpr polygon_outer() noexcept = default;
    constexpr explicit polygon_outer(std::initializer_list<P> points) noexcept : point_container<P, true, direction::CW, Container>(points)
    {
    }
};

template<concepts::point P = Point, template<class> class Container = std::vector>
struct polygon_inner : public point_container<P, true, direction::CCW, Container>
{
    constexpr polygon_inner() noexcept = default;
    constexpr explicit polygon_inner(std::initializer_list<P> points) noexcept : point_container<P, true, direction::CCW, Container>(points)
    {
    }
};

template<concepts::point P = Point, template<class> class Container = std::vector>
requires concepts::point<typename Container<P>::value_type> struct polygons : public Container<polygon<P, direction::NA, Container>*>
{
    constexpr polygons() noexcept = default;
    constexpr explicit polygons(std::initializer_list<polygon<P, direction::NA, Container>*> polygons) noexcept : Container<polygon<P, direction::NA, Container>*>(polygons)
    {
    }

    constexpr auto outer() noexcept
    {
        return polygon_outer{ this->front() };
    }

    constexpr auto inners() noexcept
    {
        return ranges::views::drop(this->base(), 1) | ranges::views::transform([](auto& p) { return polygon_inner{ p }; });
    }
};

} // namespace cura::geometry

#endif // UTILS_GEOMETRY_POINT_CONTAINER_H
