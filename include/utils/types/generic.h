// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_GENERIC_H
#define CURAENGINE_GENERIC_H

#include <concepts>
#include <functional>
#include <type_traits>

namespace cura::utils
{
template<typename T>
concept hashable = requires(T value)
{
    { std::hash<T>{}(value) } -> concepts::convertible_to<std::size_t>;
};

} // namespace cura

#ifdef RETARDED_APPLE_CLANG
namespace std
{
// https://stackoverflow.com/questions/71818683/stdintegral-not-found-in-clang13-c20-error
template<typename _Tp>
concept integral = std::is_integral_v<_Tp>;

template<typename _Tp>
concept floating_point = is_floating_point_v<_Tp>;
} // namespace std
#endif

#endif // CURAENGINE_GENERIC_H
