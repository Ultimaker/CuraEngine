// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_PLUGINPROXY_H
#define PLUGINS_PLUGINPROXY_H

#include "Application.h"
#include "components/broadcast.h"
#include "components/common.h"
#include "components/invoke.h"
#include "cura/plugins/slots/broadcast/v0/broadcast.grpc.pb.h"
#include "cura/plugins/slots/broadcast/v0/broadcast.pb.h"
#include "cura/plugins/slots/handshake/v0/handshake.grpc.pb.h"
#include "cura/plugins/slots/handshake/v0/handshake.pb.h"
#include "cura/plugins/v0/slot_id.pb.h"
#include "plugins/broadcasts.h"
#include "plugins/exception.h"
#include "plugins/metadata.h"
#include "utils/format/thread_id.h"
#include "utils/types/char_range_literal.h"
#include "utils/types/generic.h"

#include <agrpc/asio_grpc.hpp>
#include <agrpc/grpc_context.hpp>
#include <boost/asio/awaitable.hpp>
#include <boost/asio/co_spawn.hpp>
#include <boost/asio/detached.hpp>
#include <boost/asio/use_awaitable.hpp>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <range/v3/utility/semiregular_box.hpp>
#include <spdlog/spdlog.h>

#include <chrono>
#include <string>
#include <thread>

namespace cura::plugins
{

/**
 * @brief A plugin proxy class template.
 *
 * Template arguments are:
 * SlotID - plugin slot ID
 * SlotVersionRng - plugin version range
 * Stub - process stub type
 * ValidatorTp - validator type
 * RequestTp - gRPC convertible request type, or dummy -- if stub is a proper invoke-stub, this is enforced by the specialization of the invoke component
 * ResponseTp - gRPC convertible response type, or dummy -- if stub is a proper invoke-stub, this is enforced by the specialization of the invoke component
 *
 * Class provides methods for validating the plugin, making requests and processing responses.
 */
template<plugins::v0::SlotID SlotID, utils::CharRangeLiteral SlotVersionRng, class Stub, class ValidatorTp, typename RequestTp, typename ResponseTp>
class PluginProxy
{
public:
    // type aliases for easy use
    using value_type = typename ResponseTp::native_value_type;
    using validator_type = ValidatorTp;

    using req_converter_type = RequestTp;
    using rsp_converter_type = ResponseTp;

    using invoke_component_t = PluginProxyInvokeComponent<Stub, req_converter_type, rsp_converter_type>;
    using broadcast_component_t = PluginProxyBroadcastComponent<SlotID>;

    /**
     * @brief Constructs a PluginProxy object.
     *
     * This constructor initializes the PluginProxy object by establishing communication
     * channels with the plugin identified by the given slot ID. It performs plugin validation
     * and checks for communication errors.
     *
     * @param channel A shared pointer to the gRPC channel for communication with the plugin.
     *
     * @throws std::runtime_error if the plugin fails validation or communication errors occur.
     */
    constexpr PluginProxy() = default;

    explicit PluginProxy(std::shared_ptr<grpc::Channel> channel)
        : invoke_component_{ slot_info_, plugin_info_, channel }
        , broadcast_component_{ slot_info_, plugin_info_, channel }
    {
        // Connect to the plugin and exchange a handshake
        agrpc::GrpcContext grpc_context;
        grpc::Status status;
        slots::handshake::v0::HandshakeService::Stub handshake_stub(channel);
        plugin_metadata plugin_info;

        boost::asio::co_spawn(
            grpc_context,
            [this, &grpc_context, &status, &plugin_info, &handshake_stub]() -> boost::asio::awaitable<void>
            {
                using RPC = agrpc::RPC<&slots::handshake::v0::HandshakeService::Stub::PrepareAsyncCall>;
                grpc::ClientContext client_context{};
                prep_client_context(client_context, *slot_info_);

                // Construct request
                handshake_request handshake_req;
                handshake_request::value_type request{ handshake_req(*slot_info_) };

                // Make unary request
                handshake_response::value_type response;
                status = co_await RPC::request(grpc_context, handshake_stub, client_context, request, response, boost::asio::use_awaitable);
                handshake_response handshake_rsp;
                plugin_info = handshake_rsp(response, client_context.peer());
                valid_ = validator_type{ *slot_info_, plugin_info };
                if (valid_)
                {
                    spdlog::info("Using plugin: '{}-{}' running at [{}] for slot {}", plugin_info.plugin_name, plugin_info.plugin_version, plugin_info.peer, slot_info_->slot_id);
                    if (! plugin_info.broadcast_subscriptions.empty())
                    {
                        spdlog::info("Subscribing plugin '{}' to the following broadcasts {}", plugin_info.plugin_name, plugin_info.broadcast_subscriptions);
                    }
                }
            },
            boost::asio::detached);
        grpc_context.run();

        if (! status.ok()) // TODO: handle different kind of status codes
        {
            throw exceptions::RemoteException(*slot_info_, status.error_message());
        }
        if (! plugin_info.plugin_name.empty() && ! plugin_info.slot_version.empty())
        {
            plugin_info_->emplace(plugin_info);
        }
    };

    constexpr PluginProxy(const PluginProxy&) = default;
    constexpr PluginProxy(PluginProxy&&) noexcept = default;
    constexpr PluginProxy& operator=(const PluginProxy& other)
    {
        if (this != &other)
        {
            valid_ = other.valid_;
            invoke_component_ = other.invoke_component_;
            broadcast_component_ = other.broadcast_component_;
            plugin_info_ = other.plugin_info_;
            slot_info_ = other.slot_info_;
        }
        return *this;
    }
    constexpr PluginProxy& operator=(PluginProxy&& other)
    {
        if (this != &other)
        {
            valid_ = std::move(other.valid_);
            invoke_component_ = std::move(other.invoke_component_);
            broadcast_component_ = std::move(other.broadcast_component_);
            plugin_info_ = std::move(other.plugin_info_);
            slot_info_ = std::move(other.slot_info_);
        }
        return *this;
    }
    ~PluginProxy() = default;

    value_type invoke(auto&&... args)
    {
        return invoke_component_.invoke(std::forward<decltype(args)>(args)...);
    }

    template<plugins::v0::SlotID Subscription>
    void broadcast(auto&&... args)
    {
        return broadcast_component_.template broadcast<Subscription>(std::forward<decltype(args)>(args)...);
    }

private:
    validator_type valid_{}; ///< The validator object for plugin validation.

    details::slot_info_ptr slot_info_ ///< Holds information about the plugin slot.
        { std::make_shared<slot_metadata>(slot_metadata{ .slot_id = SlotID, .version_range = SlotVersionRng.value, .engine_uuid = Application::getInstance().instance_uuid }) };
    details::plugin_info_ptr plugin_info_{ std::make_shared<std::optional<plugin_metadata>>(
        std::nullopt) }; ///< Optional object that holds the plugin metadata, set after handshake

    invoke_component_t invoke_component_;
    broadcast_component_t broadcast_component_;
};

} // namespace cura::plugins

#endif // PLUGINS_PLUGINPROXY_H
