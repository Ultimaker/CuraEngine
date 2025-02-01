// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_SLOTPROXY_H
#define PLUGINS_SLOTPROXY_H

#include <concepts>
#include <functional>
#include <grpcpp/channel.h>
#include <memory>
#include <optional>

#include <boost/asio/use_awaitable.hpp>

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
 * @tparam SlotID The plugin slot ID.
 * @tparam SlotVersion The version of the indicated slot.
 * @tparam Stub The process stub type.
 * @tparam ValidatorTp The type used for validating the plugin.
 * @tparam RequestTp The gRPC convertible request type.
 * @tparam ResponseTp The gRPC convertible response type.
 * @tparam Default The default behavior when no plugin is available.
 */
template<plugins::v0::SlotID SlotID, utils::CharRangeLiteral SlotVersion, class Stub, class ValidatorTp, class RequestTp, class ResponseTp, class Default>
class SlotProxy
{
    Default default_process{};
    using value_type = PluginProxy<SlotID, SlotVersion, Stub, ValidatorTp, RequestTp, ResponseTp>;
    std::vector<value_type> plugins_;

public:
    static constexpr plugins::v0::SlotID slot_id{ SlotID };

    /**
     * @brief Default constructor.
     *
     * Constructs a SlotProxy object without initializing the plugin.
     */
    SlotProxy() noexcept = default;

    /**
     * @brief Adds a plugin to this proxy.
     * @param name The fully-qualified name of the plugin
     * @param version The full verison of the plugin
     * @param channel A shared pointer to the gRPC channel for communication with the plugin.
     */
    void addPlugin(const std::string& name, const std::string& version, std::shared_ptr<grpc::Channel> channel)
    {
        plugins_.emplace_back(name, version, channel);
    }

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
    constexpr auto generate(auto&&... args)
    {
        if (! plugins_.empty())
        {
            return plugins_.front().generate(std::forward<decltype(args)>(args)...);
        }
        return std::invoke(default_process, std::forward<decltype(args)>(args)...);
    }

    constexpr auto modify(auto& original_value, auto&&... args)
    {
        if (! plugins_.empty())
        {
            auto modified_value = original_value;

            for (value_type& plugin : plugins_)
            {
                modified_value = plugin.modify(modified_value, std::forward<decltype(args)>(args)...);
            }

            return modified_value;
        }
        if constexpr (sizeof...(args) == 0)
        {
            return std::invoke(default_process, original_value);
        }
        return std::invoke(default_process, original_value, std::forward<decltype(args)>(args)...);
    }

    template<v0::SlotID S>
    void broadcast(auto&&... args)
    {
        for (value_type& plugin : plugins_)
        {
            plugin.template broadcast<S>(std::forward<decltype(args)>(args)...);
        }
    }
};

} // namespace cura::plugins


#endif // PLUGINS_SLOTPROXY_H
