// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_CONCEPTS_GENERIC_H
#define UTILS_CONCEPTS_GENERIC_H

#include <concepts>
#include <functional>

#include <google/protobuf/message.h>
#include <range/v3/range_concepts.hpp>

namespace cura
{
template<typename T>
concept hashable = requires(T value)
{
    { std::hash<T>{}(value) } -> ranges::convertible_to<std::size_t>;
};

template<typename T>
concept grpc_convertable = requires(T value)
{
    requires ranges::semiregular<T>;
    requires ranges::semiregular<typename T::value_type>;
    requires ranges::semiregular<typename T::native_value_type>;
};

} // namespace cura

#endif // UTILS_CONCEPTS_GENERIC_H
