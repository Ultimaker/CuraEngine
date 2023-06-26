// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_TYPES_GET_H
#define UTILS_TYPES_GET_H

#include "utils/types/arachne.h"
#include "utils/types/char_range_literal.h"
#include "utils/types/geometry.h"

namespace std
{
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

template<cura::utils::CharRangeLiteral C>
constexpr auto& get(cura::utils::point2d auto& point) noexcept
{
    constexpr std::string_view idx = C.value;
    static_assert(idx.starts_with("X") || idx.starts_with("x") || idx.starts_with("Y") || idx.starts_with("y"), "Index out of bounds");
    if constexpr (idx.starts_with("X") || idx.starts_with("x"))
    {
        return std::get<0>(point);
    }
    return std::get<1>(point);
}

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

template<cura::utils::CharRangeLiteral C>
constexpr auto& get(cura::utils::point3d auto& point) noexcept
{
    static_assert(C.value == "X" || C.value == "x" || C.value == "Y" || C.value == "y" || C.value == "Z" || C.value == "z", "Index out of bounds");
    constexpr std::string_view idx = C.value;
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

template<size_t N>
constexpr auto& get(cura::utils::junction auto& junction) noexcept
{
    return get<N>(junction.p);
}

template<cura::utils::CharRangeLiteral C>
constexpr auto& get(cura::utils::junction auto& junction) noexcept
{
    return get<C>(junction.p);
}

} // namespace std


#endif // UTILS_TYPES_GET_H