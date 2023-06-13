// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_SLOTS_H
#define PLUGINS_SLOTS_H

#include <exception>
#include <memory>

#include "plugins/converters.h"
#include "plugins/slotproxy.h"
#include "plugins/types.h"
#include "plugins/validator.h"
#include "utils/IntPoint.h"
#include "utils/Simplify.h" // TODO: Remove once the simplify slot has been removed

#include "cura/plugins/slots/postprocess/v0/postprocess.grpc.pb.h"
#include "cura/plugins/slots/simplify/v0/simplify.grpc.pb.h"
#include "cura/plugins/v0/slot_id.pb.h"

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
using slot_simplify_ = SlotProxy<v0::SlotID::SIMPLIFY, "<=1.0.0", slots::simplify::v0::SimplifyService::Stub, Validator, simplify_request, simplify_response, Default>;

/**
 * @brief Alias for the Postprocess slot.
 *
 * This alias represents the Postprocess slot, which is used for post-processing G-code.
 *
 * @tparam Default The default behavior when no plugin is registered.
 */
template<class Default = default_process>
using slot_postprocess_ = SlotProxy<v0::SlotID::POSTPROCESS, "<=1.0.0", slots::postprocess::v0::PostprocessService::Stub, Validator, postprocess_request, postprocess_response, Default>;

template<typename... Types>
struct Typelist
{
};

template<typename TList, template<typename> class Unit>
class Registry;

template<template<typename> class Unit>
class Registry<Typelist<>, Unit>
{
};

template<typename T, typename... Types, template<typename> class Unit>
class Registry<Typelist<T, Types...>, Unit> : public Registry<Typelist<Types...>, Unit>
{
public:
    using ValueType = T;
    using Base = Registry<Typelist<Types...>, Unit>;
    friend Base;

    template<typename Tp>
    constexpr Tp& get()
    {
        return get_type<Tp>().proxy;
    }

    template<typename Tp>
    constexpr auto invoke(auto&&... args)
    {
        return std::invoke(get<Tp>(), std::forward<decltype(args)>(args)...);
    }

    template<typename Tp>
    void connect(auto&& plugin)
    {
        get_type<Tp>().proxy = Tp{ std::forward<Tp>(std::move(plugin)) };
    }

    void broadcast(auto&&... args)
    {
        value_.proxy.broadcast(std::forward<decltype(args)>(args)...);
        Base::value_.proxy.broadcast(std::forward<decltype(args)>(args)...);
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
    T proxy;
};

} // namespace details

using slot_simplify = details::slot_simplify_<details::simplify_default>;
using slot_postprocess = details::slot_postprocess_<>;

using SlotTypes = details::Typelist<slot_simplify, slot_postprocess>;
} // namespace plugins
using slots = plugins::details::SingletonRegistry<plugins::SlotTypes, plugins::details::Holder>;

} // namespace cura

#endif // PLUGINS_SLOTS_H
