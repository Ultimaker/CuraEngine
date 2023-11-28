// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_GENERIC_H
#define CURAENGINE_GENERIC_H

#include <range/v3/range/concepts.hpp>

#include <concepts>
#include <functional>
#include <type_traits>

namespace cura::utils
{
// clang-format off
template<typename T>
concept hashable = requires(T value)
{
    { std::hash<T>{}(value) } -> concepts::convertible_to<std::size_t>;
};

template<typename T>
concept grpc_convertable = requires(T value)
{
    requires ranges::semiregular<T>;
    requires ranges::semiregular<typename T::value_type>;
    requires ranges::semiregular<typename T::native_value_type>;
};

#ifdef OLDER_APPLE_CLANG

// std::integral and std::floating_point are not implemented in older Apple Clang versions < 13
// https://stackoverflow.com/questions/71818683/stdintegral-not-found-in-clang13-c20-error
template<typename Tp>
concept integral =
    std::is_same_v<Tp, bool> ||
    std::is_same_v<Tp, char> ||
    std::is_same_v<Tp, signed char> ||
    std::is_same_v<Tp, unsigned char> ||
    std::is_same_v<Tp, wchar_t> ||
    std::is_same_v<Tp, char8_t> ||
    std::is_same_v<Tp, char16_t> ||
    std::is_same_v<Tp, char32_t> ||
    std::is_same_v<Tp, short> ||
    std::is_same_v<Tp, unsigned short> ||
    std::is_same_v<Tp, int> ||
    std::is_same_v<Tp, unsigned int> ||
    std::is_same_v<Tp, long> ||
    std::is_same_v<Tp, unsigned long> ||
    std::is_same_v<Tp, long long> ||
    std::is_same_v<Tp, unsigned long long>;

template<typename Tp>
concept floating_point = std::is_same_v<Tp, float> || std::is_same_v<Tp, double> || std::is_same_v<Tp, long double>;
#else
template<typename Tp>
concept integral = std::integral<Tp>;

template<typename Tp>
concept floating_point = std::floating_point<Tp>;
#endif
// clang-format on

template<typename Tp>
concept numeric = std::is_arithmetic_v<std::remove_cvref_t<Tp>>;
} // namespace cura::utils

#endif // CURAENGINE_GENERIC_H
