// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_CONCEPTS_GEOMETRY_H
#define UTILS_CONCEPTS_GEOMETRY_H

#include <concepts>
#include <string>
#include <type_traits>

#include <range/v3/range/concepts.hpp>

namespace cura
{
enum class direction
{
    NA,
    CW,
    CCW
};

template<typename T>
struct is_clockwise
{
    static const bool value = T::winding == direction::CW;
};

template<typename T>
constexpr bool is_clockwise_v = is_clockwise<T>::value;

template<typename T>
struct is_counterclockwise
{
    static const bool value = T::winding == direction::CCW;
};

template<typename T>
constexpr bool is_counterclockwise_v = is_counterclockwise<T>::value;

template<typename T>
struct is_closed
{
    static const bool value = T::is_closed;
};

template<typename T>
constexpr bool is_closed_v = is_closed<T>::value;

namespace concepts
{
template<class T>
concept point2d_named = requires(T point)
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
concept point2d = point2d_named<T> || (ranges::range<T> && std::integral<typename T::value_type> && std::tuple_size_v<T> == 2);

template<class T>
concept point3d_named = requires(T point)
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
concept point3d = point3d_named<T> || (ranges::range<T> && std::integral<typename T::value_type> && std::tuple_size_v<T> == 3);

template<class T>
concept point_named = point2d_named<T> || point3d_named<T>;

/*!
 * @brief Either a Point2D or a Point3D
 * @details This concept is used to check if a type is a point. A point is a type that is a 2D or 3D point.
 * @tparam T Type to check
 */
template<class T>
concept point = point2d<T> || point3d<T>;

template<class T>
concept point_ranged = point<T> && ! point2d_named<T> && ! point3d_named<T>;

template<class T>
concept polyline = ranges::range<T> && point<typename T::value_type> && is_closed<T>::value;

template<class T>
concept polygon = ranges::range<T> && point<typename T::value_type> && ! is_closed<T>::value;

template<class T>
concept polygons = ranges::range<T> && polygon<typename T::value_type>;

template<class T>
concept poly_range = polygon<T> || polyline<T>;

} // namespace concepts

} // namespace cura
#endif // UTILS_CONCEPTS_GEOMETRY_H