// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_TYPES_GEOMETRY_H
#define UTILS_TYPES_GEOMETRY_H

#include <concepts>
#include <string>
#include <type_traits>

#include <range/v3/range/concepts.hpp>

#include "geometry/winding.h"

namespace cura::utils
{

template<class T>
concept closable = requires(T t)
{
    requires std::convertible_to<decltype(t.is_closed), bool>;
};

template<class T>
concept fillable = requires(T t)
{
    requires std::convertible_to<decltype(t.is_filled), bool>;
};


template<class T>
concept directional = requires(T t)
{
    requires std::convertible_to<decltype(t.direction), geometry::winding>;
};

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
concept clipper_path = ranges::range<T> && point<typename T::value_type>;

template<class T>
concept open_path = ranges::range<T> && closable<T> && point<typename T::value_type> && !T::is_closed;

template<class T>
concept closed_path = ranges::range<T> && closable<T> && point<typename T::value_type> && T::is_closed;

template<class T>
concept filled_path = closed_path<T> && fillable<T> && T::is_filled;

template<class T>
concept ranged_path = ranges::range<T> && (filled_path<typename T::value_type> || open_path<typename T::value_type> || closed_path<typename T::value_type> || clipper_path<typename T::value_type>);

template<class T>
concept segment = requires(T segment)
{
    { std::get<0>(segment) } -> point;
    { std::get<1>(segment) } -> point;
};

template<class T>
concept segment_container = ranges::range<T> && segment<typename T::value_type>;

} // namespace cura::utils

#endif // UTILS_TYPES_GEOMETRY_H