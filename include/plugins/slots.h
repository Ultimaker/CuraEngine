// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_SLOTS_H
#define PLUGINS_SLOTS_H

#include <memory>
#include <unordered_map>
#include <variant>

#include <boost/serialization/singleton.hpp>

#include "plugins/converters.h"
#include "plugins/slotproxy.h"
#include "plugins/types.h"
#include "plugins/validator.h"
#include "utils/Simplify.h"  // TODO: Remove once the simplify slot has been removed
#include "utils/NoCopy.h"

#include "plugin.grpc.pb.h"
#include "postprocess.grpc.pb.h"
#include "simplify.grpc.pb.h"

namespace cura::plugins
{
namespace details
{
struct default_process
{
    constexpr auto operator()(auto&& arg, auto&&...){ return std::forward<decltype(arg)>(arg); };
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
using simplify_slot = SlotProxy<SlotID::SIMPLIFY,
                                Validator<"<=0.0.1", "qwerty-azerty-temp-hash">,
                                proto::Simplify::Stub,
                                agrpc::RPC<&proto::Simplify::Stub::PrepareAsyncSimplify>,
                                simplify_request,
                                simplify_response,
                                Default>;

/**
 * @brief Alias for the Postprocess slot.
 *
 * This alias represents the Postprocess slot, which is used for post-processing G-code.
 *
 * @tparam Default The default behavior when no plugin is registered.
 */
template<class Default = default_process>
using postprocess_slot = SlotProxy<SlotID::POSTPROCESS,
                                   Validator<">=1.0.0 <2.0.0 || >3.2.1", "qwerty-azerty-temp-hash">,
                                   proto::Postprocess::Stub,
                                   agrpc::RPC<&proto::Postprocess::Stub::PrepareAsyncPostprocess>,
                                   postprocess_request,
                                   postprocess_response,
                                   Default>;

} // namespace details

/**
 * @brief Class for managing plugin slots.
 *
 * The `Slots` class provides functionality to manage plugin slots. It allows registering and retrieving plugins
 * for specific slots.
 *
 * @tparams SlotTypes The different slot types.
 */
template<class... SlotTypes>
class Slots
{
    using slots_t = std::variant<SlotTypes...>; ///< The variant representing available slots.
    std::unordered_map<SlotID, slots_t> slots_{}; ///< The map storing registered slots.

    constexpr Slots() noexcept = default;
public:
    Slots(const Slots&) = delete;
    Slots(Slots&&) = delete;

    /**
     * @brief Returns the instance of the Slots class.
     *
     * @return The instance of the Slots class.
     */
    static Slots& instance() noexcept
    {
        static Slots instance{};
        return instance;
    }

    /**
     * @brief Registers a plugin for the specified slot.
     *
     * @param plugin The plugin to register.
     */
    constexpr void set(auto&& plugin)
    {
        using plugin_t = decltype(plugin);
#ifdef PLUGINS
        slots_.emplace(plugin.slot_id, std::forward<plugin_t>(plugin));
#else
        slots_.emplace(plugin.slot_id, std::forward<plugin_t>(plugin_t{}));  // Allways create a default (not connected) plugin
#endif
    }

    /**
     * @brief Retrieves the plugin for the specified slot.
     *
     * @tparam SlotID The ID of the slot.
     * @return The plugin for the specified slot.
     */
    template<plugins::SlotID SlotID>
    constexpr auto get() const
    {
        return std::get<SlotID>(slots_.at(SlotID));
    }
};

using simplify_t = details::simplify_slot<details::simplify_default>;
using postprocess_t = details::postprocess_slot<>;

// The Template arguments should be ordered in the same ordering as the SlotID enum
using slot_registry = plugins::Slots<simplify_t, postprocess_t>;

} // namespace cura::plugins

#endif // PLUGINS_SLOTS_H
