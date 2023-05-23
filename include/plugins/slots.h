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

#include "postprocess.grpc.pb.h"
#include "simplify.grpc.pb.h"

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
using simplify_slot = SlotProxy<SlotID::SIMPLIFY, "<=1.0.0", Validator, plugins::v1::SimplifyService::Stub, agrpc::RPC<&plugins::v1::SimplifyService::Stub::PrepareAsyncModify>, simplify_request, simplify_response, Default>;

/**
 * @brief Alias for the Postprocess slot.
 *
 * This alias represents the Postprocess slot, which is used for post-processing G-code.
 *
 * @tparam Default The default behavior when no plugin is registered.
 */
template<class Default = default_process>
using postprocess_slot = SlotProxy<SlotID::POSTPROCESS, "<=1.0.0", Validator, plugins::v1::SimplifyService::Stub, agrpc::RPC<&plugins::v1::PostprocessService::Stub::PrepareAsyncModify>, postprocess_request, postprocess_response, Default>;

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
        get_type<Tp>().proxy = Tp { std::forward<Tp>( std::move(plugin) ) };
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

using simplify_t = details::simplify_slot<details::simplify_default>;
using postprocess_t = details::postprocess_slot<>;

using SlotTypes = details::Typelist<simplify_t, postprocess_t>;
} // namespace plugins
using slots = plugins::details::SingletonRegistry<plugins::SlotTypes, plugins::details::Holder>;

} // namespace cura

#endif // PLUGINS_SLOTS_H
