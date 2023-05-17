// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_SLOTS_H
#define PLUGINS_SLOTS_H

#include <memory>
#include <unordered_map>
#include <variant>

#include "plugins/converters.h"
#include "plugins/slotproxy.h"
#include "plugins/types.h"
#include "plugins/validator.h"

#include "plugin.grpc.pb.h"
#include "postprocess.grpc.pb.h"
#include "simplify.grpc.pb.h"

namespace cura::plugins
{

/**
 * @brief Alias for the Simplify slot.
 *
 * This alias represents the Simplify slot, which is used for simplifying polygons.
 *
 * @tparam Default The default behavior when no plugin is registered.
 */
template<auto Default>
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
template<auto Default>
using postprocess_slot = SlotProxy<SlotID::POSTPROCESS,
                                   Validator<">=1.0.0 <2.0.0 || >3.2.1", "qwerty-azerty-temp-hash">,
                                   proto::Postprocess::Stub,
                                   agrpc::RPC<&proto::Postprocess::Stub::PrepareAsyncPostprocess>,
                                   postprocess_request,
                                   postprocess_response,
                                   Default>;

/**
 * @brief Class for managing plugin slots.
 *
 * The `Slots` class provides functionality to manage plugin slots. It allows registering and retrieving plugins
 * for specific slots.
 *
 * @tparam Simplify The Simplify slot type.
 * @tparam Postprocess The Postprocess slot type.
 */
template<class Simplify, class Postprocess>  // TODO: use variadic template args
class Slots
{
    using slots_t = std::variant<Simplify, Postprocess>; ///< The variant representing available slots.
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
        slots_.emplace(plugin.slot_id, std::forward<decltype(plugin)>(plugin));
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
} // namespace cura::plugins

#endif // PLUGINS_SLOTS_H
