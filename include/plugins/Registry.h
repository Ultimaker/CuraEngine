// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
#include <concepts>
#include <type_traits>
#include <utility>

#include "cura/plugins/v0/slot_id.pb.h"
#include "plugins/Typelist.h"
#include "plugins/plugin_metadata.h"
#include "plugins/slot_metadata.h"

namespace cura::plugins::details
{
template<typename TList, template<typename> class Unit>
class Registry;

template<template<typename> class Unit>
class Registry<Typelist<>, Unit>
{
public:
    constexpr void connect([[maybe_unused]] auto&&... args) noexcept
    {
    }

    template<v0::SlotID S>
    constexpr void broadcast([[maybe_unused]] auto&&... args) noexcept
    {
    } // Base case, do nothing

    void forEachPlugin(const std::invocable<const slot_metadata&, const plugin_metadata&> auto& function) noexcept
    {
    }
};

template<typename T, typename... Types, template<typename> class Unit>
class Registry<Typelist<T, Types...>, Unit> : public Registry<Typelist<Types...>, Unit>
{
public:
    using ValueType = T;
    using Base = Registry<Typelist<Types...>, Unit>;
    using Base::broadcast;
    friend Base;

    template<v0::SlotID S>
    constexpr auto& get()
    {
        return get_type<S>().proxy;
    }

    template<v0::SlotID S>
    constexpr auto modify(auto& original_value, auto&&... args)
    {
        return get<S>().modify(original_value, std::forward<decltype(args)>(args)...);
    }

    template<v0::SlotID S>
    constexpr auto generate(auto&&... args)
    {
        return get<S>().generate(std::forward<decltype(args)>(args)...);
    }

    void connect(const v0::SlotID& slot_id, auto name, auto& version, auto&& channel)
    {
        if (slot_id == T::slot_id)
        {
            value_.proxy.addPlugin(name, version, std::forward<decltype(channel)>(channel));
            return;
        }
        Base::connect(slot_id, name, version, std::forward<decltype(channel)>(channel));
    }

    template<v0::SlotID S>
    void broadcast(auto&&... args)
    {
        value_.proxy.template broadcast<S>(std::forward<decltype(args)>(args)...);
        Base::template broadcast<S>(std::forward<decltype(args)>(args)...);
    }

    void forEachPlugin(const std::invocable<const slot_metadata&, const plugin_metadata&> auto& function)
    {
        value_.proxy.forEachPlugin(function);
        Base::forEachPlugin(std::forward<decltype(function)>(function));
    }

protected:
    template<v0::SlotID S>
    constexpr auto& get_type()
    {
        return get_helper<S>(std::bool_constant<S == ValueType::slot_id>{});
    }

    template<v0::SlotID S>
    constexpr auto& get_helper(std::true_type)
    {
        return value_;
    }

    template<v0::SlotID S>
    constexpr auto& get_helper(std::false_type)
    {
        return Base::template get_type<S>();
    }

    Unit<ValueType> value_;
};
} // namespace cura::plugins::details
#endif // ENABLE_PLUGINS
