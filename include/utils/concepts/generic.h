// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_CONCEPTS_GENERIC_H
#define UTILS_CONCEPTS_GENERIC_H

#include <concepts>
#include <functional>

#include <google/protobuf/message.h>

namespace cura
{
template<typename T>
concept hashable = requires(T value)
{
    { std::hash<T>{}(value) } -> std::convertible_to<std::size_t>;
};

template<typename T>
concept grpc_convertable = requires(T value)
{
    requires std::semiregular<T>;
    requires std::semiregular<typename T::value_type>;
    requires std::semiregular<typename T::native_value_type>;
};

} // namespace cura

#endif // UTILS_CONCEPTS_GENERIC_H
