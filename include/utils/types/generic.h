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

namespace std
{
#if (__cplusplus > 201703L) && (!defined(_LIBCPP_VERSION) || (__clang_major__ > 13))
// https://stackoverflow.com/questions/71818683/stdintegral-not-found-in-clang13-c20-error
#else
template<typename _Tp>
concept integral = std::is_integral_v<_Tp>;
#endif
} // namespace std

#endif // CURAENGINE_GENERIC_H
