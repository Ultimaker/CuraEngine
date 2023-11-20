// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

/*! @file geometry.h
 *  @brief Defines geometric concepts like Point2D, Point3D, Segment etc.
 */

#ifndef UTILS_TYPES_GEOMETRY_H
#define UTILS_TYPES_GEOMETRY_H

#include "utils/types/generic.h"

#include <range/v3/range/concepts.hpp>
#include <range/v3/range/operations.hpp>

#include <concepts>
#include <string>
#include <tuple>
#include <type_traits>

namespace cura::utils
{
// clang-format off

/*!
 * @concept point2d_tuple
 * @brief Checks whether T is a 2D point represented as a tuple with two integral elements
 * @tparam T The type to check
 */
template<typename T>
concept point2d_tuple = requires(T t)
{
    requires std::is_same_v<T, std::tuple<typename std::tuple_element<0, T>::type, typename std::tuple_element<0, T>::type>>;
    requires utils::numeric<std::tuple_element_t<0, T>>;
};

/*!
 * @concept point2d_ranged
 * @brief Checks whether T is a 2D point represented as a range with two integral elements
 * @tparam T The type to check
 */
template<class T>
concept point2d_ranged = ranges::range<T> && requires(T point)
{
    requires ranges::size(point) == 2;
    requires utils::numeric<ranges::range_value_t<T>>;
};


/*!
 * @concept point2d_named
 * @brief Checks whether T is a 2D point represented as an object with X and Y integral fields
 * @tparam T The type to check
 */
template<class T>
concept point2d_named = requires(T point)
{
    requires utils::numeric<decltype(point.X)>;
    requires utils::numeric<decltype(point.Y)>;
};

/*!
 * @concept point2d
 * @brief Checks whether T is a 2D point. A type satisfying any of point2d_tuple, point2d_ranged, or point2d_named
 * @tparam T The type to check
 */
template<class T>
concept point2d = point2d_named<T> || point2d_ranged<T> || point2d_tuple<T>;

/*!
 * @concept point3d_tuple
 * @brief Checks whether T is a 3D point represented as a tuple with three integral elements
 * @tparam T The type to check
 */
template<typename T>
concept point3d_tuple = requires(T t)
{
    requires std::is_same_v<T, std::tuple<typename std::tuple_element<0, T>::type, typename std::tuple_element<0, T>::type, typename std::tuple_element<0, T>::type>>;
    requires utils::numeric<std::tuple_element_t<0, T>>;
};

/*!
 * @concept point3d_ranged
 * @brief Checks whether T is a 3D point represented as a range with three integral elements
 * @tparam T The type to check
 */
template<class T>
concept point3d_ranged = ranges::range<T> && requires(T point)
{
    requires ranges::size(point) == 3;
    requires utils::numeric<ranges::range_value_t<T>>;
};

/*!
 * @concept point3d_named
 * @brief Checks whether T is a 3D point represented as an object with X, Y and Z integral fields
 * @tparam T The type to check
 */
template<class T>
concept point3d_named = requires(T point)
{
    requires utils::numeric<decltype(point.x)>;
    requires utils::numeric<decltype(point.y)>;
    requires utils::numeric<decltype(point.z)>;
};

/*!
 * @concept point3d
 * @brief Checks whether T is a 3D point. A type satisfying any of point3d_tuple, point3d_ranged, or point3d_named
 * @tparam T The type to check
 */
template<class T>
concept point3d = point3d_named<T> || point3d_ranged<T> || point3d_tuple<T>;

/*!
 * @concept point_named
 * @brief Checks whether T is a point represented as an object with named fields
 * @tparam T The type to check
 */
template<class T>
concept point_named = point2d_named<T> || point3d_named<T>;

/*!
 * @concept point
 * @brief Checks whether T is a 2D or 3D point
 * @tparam T The type to check
 */
template<class T>
concept point = point2d<T> || point3d<T>;

/*!
 * @concept segment
 * @brief Checks whether T represents a segment as a pair of 2D or 3D points.
 * @tparam T The type to check
 */
template<class T>
concept segment = requires(T segment)
{
    requires point<decltype(std::get<0>(segment))>;
    requires point<decltype(std::get<1>(segment))>;
};

/*!
 * @concept segment_range
 * @brief Checks whether T is a range of valid segments
 * @tparam T The type to check
 */
template<class T>
concept segment_range = ranges::range<T> && requires(T segment_range)
{
    requires segment<decltype(ranges::front(segment_range))>;
};
// clang-format on
} // namespace cura::utils

#endif // UTILS_TYPES_GEOMETRY_H