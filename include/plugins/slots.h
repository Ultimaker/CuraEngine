// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_SLOTS_H
#define PLUGINS_SLOTS_H

#include <exception>
#include <memory>
#include <unordered_map>
#include <variant>

#include <boost/serialization/singleton.hpp>

#include "plugins/converters.h"
#include "plugins/slotproxy.h"
#include "plugins/types.h"
#include "plugins/validator.h"
#include "utils/IntPoint.h"
#include "utils/NoCopy.h"
#include "utils/Simplify.h" // TODO: Remove once the simplify slot has been removed

#include "plugin.grpc.pb.h"
#include "postprocess.grpc.pb.h"
#include "simplify.grpc.pb.h"

namespace cura::plugins
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
using simplify_slot = SlotProxy<SlotID::SIMPLIFY, "<=0.0.1", Validator, proto::Simplify::Stub, agrpc::RPC<&proto::Simplify::Stub::PrepareAsyncSimplify>, simplify_request, simplify_response, Default>;

/**
 * @brief Alias for the Postprocess slot.
 *
 * This alias represents the Postprocess slot, which is used for post-processing G-code.
 *
 * @tparam Default The default behavior when no plugin is registered.
 */
template<class Default = default_process>
using postprocess_slot = SlotProxy<SlotID::POSTPROCESS, ">=1.0.0 <2.0.0 || >3.2.1", Validator, proto::Postprocess::Stub, agrpc::RPC<&proto::Postprocess::Stub::PrepareAsyncPostprocess>, postprocess_request, postprocess_response, Default>;

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

    template<typename Tp>
    Tp& get()
    {
        return get_type<Tp>().value;
    }

    template<typename Tp>
    auto call(auto&&... args)
    {
        auto holder = get_type<Tp>();
        return std::invoke(holder.value, std::forward<decltype(args)>(args)...);
    }

private:
    template<typename Tp>
    Unit<Tp>& get_type()
    {
        return get_helper<Tp>(std::is_same<Tp, ValueType>{});
    }

    template<typename Tp>
    Unit<Tp>& get_helper(std::true_type)
    {
        return value_;
    }

    template<typename Tp>
    Unit<Tp>& get_helper(std::false_type)
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
    SingletonRegistry()
    {
    }
};

template<typename T>
struct Holder
{
    T value;
    //    agrpc::GrpcContext context;
};

} // namespace details

using simplify_t = details::simplify_slot<details::simplify_default>;
using postprocess_t = details::postprocess_slot<>;

using SlotTypes = details::Typelist<simplify_t, postprocess_t>;
using slot_registry = details::SingletonRegistry<SlotTypes, details::Holder>;

} // namespace cura::plugins

#endif // PLUGINS_SLOTS_H
