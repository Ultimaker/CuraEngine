// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GRADUAL_FLOW_CONCEPTS_H
#define GRADUAL_FLOW_CONCEPTS_H

#include <range/v3/range/concepts.hpp>

#if __has_include(<concepts>)
#include <concepts>
#elif __has_include(<experimental/concepts>)
#include <experimental/concepts>
#define USE_EXPERIMENTAL_CONCEPTS
#endif

#include <string>
#include <type_traits>

namespace cura::gradual_flow
{

enum class Direction
{
    NA,
    CW,
    CCW
};

namespace concepts
{

template<class T>
concept Closable = requires(T t)
{
    requires ranges::convertible_to<decltype(t.is_closed), bool>;
};

template<class T>
concept IsClosedPointContainer = Closable<T> && requires(T t)
{
    t.is_closed == true;
};

template<class T>
concept IsOpenPointContainer = Closable<T> && requires(T t)
{
    t.is_closed == false;
};

template<class T>
concept Directional = requires(T t)
{
    requires std::is_same_v<decltype(t.winding), Direction>;
};

template<class T>
concept IsClockwisePointContainer = Directional<T> && requires(T t)
{
    t.winding == Direction::CW;
};

template<class T>
concept IsCounterclockwisePointContainer = Directional<T> && requires(T t)
{
    t.winding == Direction::CCW;
};

template<class T>
concept Point2DNamed = requires(T point)
{
    point.X;
    point.Y;
};

/*!
 * @brief A 2D point, defined either as a named object with X and Y attributes, or as a range of two integral values.
 * @details This concept is used to check if a type is a 2D point. A 2D point is a type that has a X and Y member or a type that is a range of integral types with a size of 2.
 * @tparam T Type to check
 */
template<class T>
concept Point2D = Point2DNamed<T> ||(ranges::range<T>&& ranges::integral<typename T::value_type>&& std::tuple_size_v<T> == 2);

template<class T>
concept Point3DNamed = requires(T point)
{
    point.x;
    point.y;
    point.z;
};

/*!
 * @brief  A 3D point, defined either as a named object with x, y, and z attributes, or as a range of three integral values.
 * @details This concept is used to check if a type is a 3D point. A 3D point is a type that has a x, y and z member or a type that is a range of integral types with a size of 3.
 * @tparam T Type to check
 */
template<class T>
concept Point3D = Point3DNamed<T> ||(ranges::range<T>&& ranges::integral<typename T::value_type>&& std::tuple_size_v<T> == 3);

template<class T>
concept PointNamed = Point2DNamed<T> || Point3DNamed<T>;

/*!
 * @brief Either a Point2D or a Point3D
 * @details This concept is used to check if a type is a point. A point is a type that is a 2D or 3D point.
 * @tparam T Type to check
 */
template<class T>
concept Point = Point2D<T> || Point3D<T>;

template<class T>
concept PointRanged = Point<T> && ! Point2DNamed<T> && ! Point3DNamed<T>;

template<class T>
concept Polyline = ranges::range<T> && IsOpenPointContainer<T> && Point<typename T::value_type>;

template<class T>
concept Polygon = ranges::range<T> && IsClosedPointContainer<T> && Point<typename T::value_type>;

template<class T>
concept Polygons = ranges::range<T> && Polygon<typename T::value_type>;

template<class T>
concept PolyRange = Polygon<T> || Polyline<T>;

} // namespace concepts
} // namespace cura::gradual_flow

#endif // GRADUAL_FLOW_CONCEPTS_H
