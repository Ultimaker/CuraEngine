// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_TYPES_GET_H
#define UTILS_TYPES_GET_H

#include "utils/types/arachne.h"
#include "utils/types/char_range_literal.h"
#include "utils/types/geometry.h"

namespace std
{

/**
 * @brief Get a coordinate of a 2d point by index.
 *
 * @tparam N The index of the coordinate to get (0 for X, 1 for Y).
 * @param point The point to get the coordinate from.
 * @return auto& A reference to the requested coordinate.
 */
template<size_t N>
constexpr auto& get(cura::utils::point2d_named auto& point) noexcept
{
    static_assert(N < 2, "Index out of bounds");
    if constexpr (N == 0)
    {
        return point.X;
    }
    return point.Y;
}

/**
 * @brief Get a coordinate of a 2d point by character.
 *
 * @tparam C The character of the coordinate to get ('x' or 'X' for X-coordinate, 'y' or 'Y' for Y-coordinate).
 * @param point The point to get the coordinate from.
 * @return auto& A reference to the requested coordinate.
 */
template<cura::utils::CharRangeLiteral C>
constexpr auto& get(cura::utils::point2d auto& point) noexcept
{
    constexpr std::string_view idx = C.value;
    static_assert(idx.size() == 1, "Only one character allowed");
    static_assert(idx.starts_with("X") || idx.starts_with("x") || idx.starts_with("Y") || idx.starts_with("y"), "Index out of bounds");
    if constexpr (idx.starts_with("X") || idx.starts_with("x"))
    {
        return std::get<0>(point);
    }
    return std::get<1>(point);
}

/**
 * @brief Get a coordinate of a 2d point by character.
 *
 * @tparam C The character of the coordinate to get ('x' or 'X' for X-coordinate, 'y' or 'Y' for Y-coordinate).
 * @param point The point to get the coordinate from.
 * @return auto& A reference to the requested coordinate.
 */
template<cura::utils::CharRangeLiteral C>
constexpr const auto& get(const cura::utils::point2d auto& point) noexcept
{
    constexpr std::string_view idx = C.value;
    static_assert(idx.size() == 1, "Only one character allowed");
    static_assert(idx.starts_with("X") || idx.starts_with("x") || idx.starts_with("Y") || idx.starts_with("y"), "Index out of bounds");
    if constexpr (idx.starts_with("X") || idx.starts_with("x"))
    {
        return std::get<0>(point);
    }
    return std::get<1>(point);
}

/**
 * @brief Get a coordinate of a 3d point by index.
 *
 * @tparam N The index of the coordinate to get (0 for X, 1 for Y, 2 for Z).
 * @param point The point to get the coordinate from.
 * @return auto& A reference to the requested coordinate.
 */
template<size_t N>
constexpr auto& get(cura::utils::point3d_named auto& point) noexcept
{
    static_assert(N < 3, "Index out of bounds");
    if constexpr (N == 0)
    {
        return point.x;
    }
    else if constexpr (N == 1)
    {
        return point.y;
    }
    return point.z;
}

/**
 * @brief Get a coordinate of a 3d point by character.
 *
 * @tparam C The character of the coordinate to get ('x' or 'X' for X-coordinate, 'y' or 'Y' for Y-coordinate, 'z' or 'Z' for Z-coordinate).
 * @param point The point to get the coordinate from.
 * @return auto& A reference to the requested coordinate.
 */
template<cura::utils::CharRangeLiteral C>
constexpr auto& get(cura::utils::point3d auto& point) noexcept
{
    constexpr std::string_view idx = C.value;
    static_assert(idx.size() == 1, "Only one character allowed");
    static_assert(idx.starts_with("X") || idx.starts_with("x") || idx.starts_with("Y") || idx.starts_with("y") || idx.starts_with("Z") || idx.starts_with("z"), "Index out of bounds");
    if constexpr (idx.starts_with("X") || idx.starts_with("x"))
    {
        return std::get<0>(point);
    }
    if constexpr (idx.starts_with("Y") || idx.starts_with("y"))
    {
        return std::get<1>(point);
    }
    return std::get<2>(point);
}

/**
 * @brief Get a coordinate of a junction by index.
 *
 * @tparam N The index of the coordinate to get.
 * @param junction The junction to get the coordinate from.
 * @return auto& A reference to the requested coordinate.
 */
template<size_t N>
constexpr auto& get(cura::utils::junction auto& junction) noexcept
{
    return get<N>(junction.p);
}

/**
 * @brief Get a coordinate of a junction by character.
 *
 * @tparam C The character of the coordinate to get.
 * @param junction The junction to get the coordinate from.
 * @return auto& A reference to the requested coordinate.
 */
template<cura::utils::CharRangeLiteral C>
constexpr auto& get(cura::utils::junction auto& junction) noexcept
{
    return get<C>(junction.p);
}

} // namespace std


#endif // UTILS_TYPES_GET_H