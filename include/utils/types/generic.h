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
concept floating_point =
    std::is_same_v<Tp, float> ||
    std::is_same_v<Tp, double> ||
    std::is_same_v<Tp, long double>;
} // namespace std
#endif

#endif // CURAENGINE_GENERIC_H
