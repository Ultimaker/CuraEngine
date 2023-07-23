// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_SLOTS_H
#define PLUGINS_SLOTS_H

#include "cura/plugins/slots/broadcast/v0/broadcast.grpc.pb.h"
#include "cura/plugins/slots/postprocess/v0/postprocess.grpc.pb.h"
#include "cura/plugins/slots/simplify/v0/simplify.grpc.pb.h"
#include "cura/plugins/v0/slot_id.pb.h"
#include "plugins/converters.h"
#include "plugins/slotproxy.h"
#include "plugins/types.h"
#include "plugins/validator.h"
#include "utils/IntPoint.h"
#include "utils/Simplify.h" // TODO: Remove once the simplify slot has been removed
#include "utils/types/char_range_literal.h"

#include <exception>
#include <memory>

namespace cura
{
namespace plugins
{
namespace details
{
struct default_process
{
    constexpr auto operator()(auto&& arg, auto&&...)
    {
        return std::forward<decltype(arg)>(arg);
    };
};

struct simplify_default
{
    auto operator()(auto&& arg, auto&&... args)
    {
        const Simplify simplify{ std::forward<decltype(args)>(args)... };
        return simplify.polygon(std::forward<decltype(arg)>(arg));
    }
};

/**
 * @brief Alias for the Simplify slot.
 *
 * This alias represents the Simplify slot, which is used for simplifying polygons.
 *
 * @tparam Default The default behavior when no plugin is registered.
 */
template<class Default = default_process>
using slot_simplify_ = SlotProxy<v0::SlotID::SIMPLIFY_MODIFY, "<=1.0.0", slots::simplify::v0::SimplifyModifyService::Stub, Validator, simplify_request, simplify_response, Default>;

/**
 * @brief Alias for the Postprocess slot.
 *
 * This alias represents the Postprocess slot, which is used for post-processing G-code.
 *
 * @tparam Default The default behavior when no plugin is registered.
 */
template<class Default = default_process>
using slot_postprocess_
    = SlotProxy<v0::SlotID::POSTPROCESS_MODIFY, "<=1.0.0", slots::postprocess::v0::PostprocessModifyService::Stub, Validator, postprocess_request, postprocess_response, Default>;

template<class Default = default_process>
using slot_settings_broadcast_
    = SlotProxy<v0::SlotID::SETTINGS_BROADCAST, "<=1.0.0", slots::broadcast::v0::BroadcastService::Stub, Validator, broadcast_settings_request, empty, Default>;

template<typename... Types>
struct Typelist
{
};

template<typename TList, template<typename> class Unit>
class Registry;

template<template<typename> class Unit>
class Registry<Typelist<>, Unit>
{
public:
    template<typename Tp>
    void broadcast(auto&&... args) {}  // Base case, do nothing
};

template<typename T, typename... Types, template<typename> class Unit>
class Registry<Typelist<T, Types...>, Unit> : public Registry<Typelist<Types...>, Unit>
{
public:
    using ValueType = T;
    using Base = Registry<Typelist<Types...>, Unit>;
    using Base::broadcast;
    friend Base;

    template<typename Tp>
    constexpr Tp& get()
    {
        return get_type<Tp>().proxy;
    }

    template<typename Tp>
    constexpr auto modify(auto&&... args)
    {
        return get<Tp>().modify(std::forward<decltype(args)>(args)...);
    }

    template<typename Tp>
    void connect(auto&& plugin)
    {
        get_type<Tp>().proxy = Tp{ std::forward<Tp>(std::move(plugin)) };
    }

    template<typename Tp>
    void broadcast(auto&&... args)
    {
        value_.proxy.template broadcast<Tp::slot_id>(std::forward<decltype(args)>(args)...);
        Base::template broadcast<Tp>(std::forward<decltype(args)>(args)...);
    }

protected:
    template<typename Tp>
    constexpr Unit<Tp>& get_type()
    {
        return get_helper<Tp>(std::is_same<Tp, ValueType>{});
    }

    template<typename Tp>
    constexpr Unit<Tp>& get_helper(std::true_type)
    {
        return value_;
    }

    template<typename Tp>
    constexpr Unit<Tp>& get_helper(std::false_type)
    {
        return Base::template get_type<Tp>();
    }

    Unit<ValueType> value_;
};

template<typename TList, template<typename> class Unit>
class SingletonRegistry
{
public:
    static Registry<TList, Unit>& instance()
    {
        static Registry<TList, Unit> instance;
        return instance;
    }

private:
    constexpr SingletonRegistry() = default;
};

template<typename T>
struct Holder
{
    using value_type = T;
    T proxy;
};

} // namespace details

using slot_simplify = details::slot_simplify_<details::simplify_default>;
using slot_postprocess = details::slot_postprocess_<>;
using slot_settings_broadcast = details::slot_settings_broadcast_<>;

using SlotTypes = details::Typelist<slot_simplify, slot_postprocess, slot_settings_broadcast>;
} // namespace plugins
using slots = plugins::details::SingletonRegistry<plugins::SlotTypes, plugins::details::Holder>;

} // namespace cura

#endif // PLUGINS_SLOTS_H
