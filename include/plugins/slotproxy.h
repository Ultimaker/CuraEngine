// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_SLOTPROXY_H
#define PLUGINS_SLOTPROXY_H

#include <concepts>
#include <functional>
#include <memory>
#include <optional>

#include <boost/asio/use_awaitable.hpp>
#include <grpcpp/channel.h>

#include "plugins/converters.h"
#include "plugins/pluginproxy.h"
#include "plugins/types.h"
#include "plugins/validator.h"
#include "utils/types/char_range_literal.h"

namespace cura::plugins
{

/**
 * @brief A class template representing a proxy for a plugin slot.
 *
 * The SlotProxy class template acts as a proxy for a plugin slot and provides an interface
 * for communication with plugins assigned to the slot. It delegates plugin requests to the
 * corresponding PluginProxy object and provides a default behavior when no plugin is available.
 *
 * @tparam Slot The plugin slot ID.
 * @tparam Validator The type used for validating the plugin.
 * @tparam Stub The process stub type.
 * @tparam Prepare The prepare type.
 * @tparam Request The gRPC convertible request type.
 * @tparam Response The gRPC convertible response type.
 * @tparam Default The default behavior when no plugin is available.
 */
template<plugins::v0::SlotID SlotID, details::CharRangeLiteral SlotVersionRng, class Stub, class ValidatorTp, class RequestTp, class ResponseTp, class Default>
class SlotProxy
{
    Default default_process{};
    using value_type = PluginProxy<SlotID, SlotVersionRng, Stub, ValidatorTp, RequestTp, ResponseTp>;
    std::optional<value_type> plugin_{ std::nullopt };

public:
    /**
     * @brief Default constructor.
     *
     * Constructs a SlotProxy object without initializing the plugin.
     */
    SlotProxy() noexcept = default;

    /**
     * @brief Constructs a SlotProxy object with a plugin.
     *
     * Constructs a SlotProxy object and initializes the plugin using the provided gRPC channel.
     *
     * @param channel A shared pointer to the gRPC channel for communication with the plugin.
     */
    SlotProxy(std::shared_ptr<grpc::Channel> channel) : plugin_{ std::move(channel) } {};

    /**
     * @brief Executes the plugin operation.
     *
     * This operator allows the SlotProxy object to be invoked as a callable, which delegates the
     * plugin request to the corresponding PluginProxy object if available. If no plugin is available,
     * it invokes the default behavior provided by the `Default` callable object.
     *
     * @tparam Args The argument types for the plugin request.
     * @param args The arguments for the plugin request.
     * @return The result of the plugin request or the default behavior.
     */
    auto operator()(auto&&... args)
    {
        if (plugin_.has_value())
        {
            return std::invoke(plugin_.value(), std::forward<decltype(args)>(args)...);
        }
        return std::invoke(default_process, std::forward<decltype(args)>(args)...);
    }

    template<details::CharRangeLiteral BroadcastChannel>
    void broadcast(auto&&...args)
    {
        if (plugin_.has_value())
        {
            plugin_.value().template broadcast<BroadcastChannel>(std::forward<decltype(args)>(args)...);
        }
    }
};

} // namespace cura::plugins


#endif // PLUGINS_SLOTPROXY_H
