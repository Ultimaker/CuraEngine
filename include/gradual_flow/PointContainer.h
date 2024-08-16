// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GRADUAL_FLOW_POINT_CONTAINER_H
#define GRADUAL_FLOW_POINT_CONTAINER_H

#include <initializer_list>
#include <memory>
#include <polyclipping/clipper.hpp>
#include <vector>

#include <range/v3/view/drop.hpp>
#include <range/v3/view/transform.hpp>

#include "geometry/Point2LL.h"
#include "gradual_flow/Concepts.h"

namespace cura::gradual_flow::geometry
{

/*! The base clase of all point based container types
 *
 * @tparam P
 * @tparam IsClosed
 * @tparam Direction
 * @tparam Container
 */
template<concepts::Point P, bool IsClosed, Direction direction>
struct PointContainer : public std::vector<P>
{
    inline static constexpr bool is_closed = IsClosed;
    inline static constexpr Direction winding = direction;

    constexpr PointContainer() noexcept = default;
    constexpr PointContainer(std::initializer_list<P> points) noexcept
        : std::vector<P>(points)
    {
    }
};

template<concepts::Point P = Point2LL>
struct Polyline : public PointContainer<P, false, Direction::NA>
{
    constexpr Polyline() noexcept = default;
    constexpr Polyline(std::initializer_list<P> points) noexcept
        : PointContainer<P, false, Direction::NA>(points)
    {
    }
};

template<concepts::Point P, Direction direction>
struct Polygon : public PointContainer<P, true, direction>
{
    constexpr Polygon() noexcept = default;
    constexpr Polygon(std::initializer_list<P> points) noexcept
        : PointContainer<P, true, direction>(points)
    {
    }
};

template<concepts::Point P = Point2LL>
Polygon(std::initializer_list<P>) -> Polygon<P, Direction::NA>;

template<concepts::Point P = Point2LL>
struct PolygonOuter : public PointContainer<P, true, Direction::CW>
{
    constexpr PolygonOuter() noexcept = default;
    constexpr PolygonOuter(std::initializer_list<P> points) noexcept
        : PointContainer<P, true, Direction::CW>(points)
    {
    }
};

template<concepts::Point P = Point2LL>
PolygonOuter(std::initializer_list<P>) -> PolygonOuter<P>;

template<concepts::Point P = Point2LL>
struct PolygonInner : public PointContainer<P, true, Direction::CCW>
{
    constexpr PolygonInner() noexcept = default;
    constexpr PolygonInner(std::initializer_list<P> points) noexcept
        : PointContainer<P, true, Direction::CCW>(points)
    {
    }
};

template<concepts::Point P = Point2LL>
PolygonInner(std::initializer_list<P>) -> PolygonInner<P>;

template<concepts::Point P = Point2LL>
struct Polygons : public std::vector<Polygon<P, Direction::NA>*>
{
    constexpr Polygons() noexcept = default;
    constexpr Polygons(std::initializer_list<Polygon<P, Direction::NA>*> polygons) noexcept
        : std::vector<Polygon<P, Direction::NA>*>(polygons)
    {
    }

    constexpr auto outer() noexcept
    {
        return PolygonOuter{ this->front() };
    }

    constexpr auto inners() noexcept
    {
        return ranges::views::drop(this->base(), 1)
             | ranges::views::transform(
                   [](auto& p)
                   {
                       return PolygonInner{ p };
                   });
    }
};

template<concepts::Point P = Point2LL>
Polygons(PolygonOuter<P>, std::initializer_list<PolygonInner<P>>) -> Polygons<P>;

} // namespace cura::gradual_flow::geometry

#endif // GRADUAL_FLOW_POINT_CONTAINER_H
