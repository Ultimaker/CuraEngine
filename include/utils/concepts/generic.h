// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_GENERIC_H
#define CURAENGINE_GENERIC_H

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

template<typename C, typename M, typename R>
concept receive_callable = requires(C callable, M message)
{
    { callable(message) } -> std::same_as<R>;
    std::is_base_of_v<google::protobuf::Message, M>;
};

template<typename C, typename M, typename... S>
concept send_callable = requires(C callable, S... args)
{
    { callable(args...) } -> std::same_as<std::shared_ptr<M>>;
    std::is_base_of_v<google::protobuf::Message, M>;
};
} // namespace cura

#endif // CURAENGINE_GENERIC_H
