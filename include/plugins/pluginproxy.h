// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_PLUGINPROXY_H
#define PLUGINS_PLUGINPROXY_H

#include "Application.h"
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
 * RequestTp - gRPC convertible request type,
 * ResponseTp - gRPC convertible response type.
 *
 * Class provides methods for validating the plugin, making requests and processing responses.
 */
template<plugins::v0::SlotID SlotID, utils::CharRangeLiteral SlotVersionRng, class Stub, class ValidatorTp, utils::grpc_convertable RequestTp, utils::grpc_convertable ResponseTp>
class PluginProxy
{
public:
    // type aliases for easy use
    using value_type = typename ResponseTp::native_value_type;
    using validator_type = ValidatorTp;

    using req_msg_type = typename RequestTp::value_type;
    using rsp_msg_type = typename ResponseTp::value_type;

    using req_converter_type = RequestTp;
    using rsp_converter_type = ResponseTp;

    using modify_stub_t = Stub;
    using broadcast_stub_t = slots::broadcast::v0::BroadcastService::Stub;

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
        : modify_stub_(channel)
        , broadcast_stub_(channel)
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
                prep_client_context(client_context);

                // Construct request
                handshake_request handshake_req;
                handshake_request::value_type request{ handshake_req(slot_info_) };

                // Make unary request
                handshake_response::value_type response;
                status = co_await RPC::request(grpc_context, handshake_stub, client_context, request, response, boost::asio::use_awaitable);
                handshake_response handshake_rsp;
                plugin_info = handshake_rsp(response, client_context.peer());
                valid_ = validator_type{ slot_info_, plugin_info_.value() };
                if (valid_)
                {
                    spdlog::info("Using plugin: '{}-{}' running at [{}] for slot {}", plugin_info.plugin_name, plugin_info.plugin_version, plugin_info.peer, slot_info_.slot_id);
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
            throw exceptions::RemoteException(slot_info_, status.error_message());
        }
        if (! plugin_info.plugin_name.empty() && ! plugin_info.slot_version.empty())
        {
            plugin_info_ = plugin_info;
        }
    };

    constexpr PluginProxy(const PluginProxy&) = default;
    constexpr PluginProxy(PluginProxy&&) noexcept = default;
    constexpr PluginProxy& operator=(const PluginProxy& other)
    {
        if (this != &other)
        {
            valid_ = other.valid_;
            modify_stub_ = other.modify_stub_;
            broadcast_stub_ = other.broadcast_stub_;
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
            modify_stub_ = std::move(other.modify_stub_);
            broadcast_stub_ = std::move(other.broadcast_stub_);
            plugin_info_ = std::move(other.plugin_info_);
            slot_info_ = std::move(other.slot_info_);
        }
        return *this;
    }
    ~PluginProxy() = default;

    /**
     * @brief Executes to plugin Modify operation.
     *
     * As part of this operation, a request is sent to the plugin
     * and the returned response is processed.
     *
     * @tparam Args -  argument types for the plugin request
     * @param args - arguments for the plugin request
     * @return The converted response value from plugin.
     *
     * @throws std::runtime_error if communication with the plugin fails.
     */
    value_type invoke(auto&&... args)
    {
        agrpc::GrpcContext grpc_context;
        value_type ret_value{};
        grpc::Status status;

        boost::asio::co_spawn(
            grpc_context,
            [this, &grpc_context, &status, &ret_value, &args...]()
            {
                return this->modifyCall(grpc_context, status, ret_value, std::forward<decltype(args)>(args)...);
            },
            boost::asio::detached);
        grpc_context.run();

        if (! status.ok()) // TODO: handle different kind of status codes
        {
            if (plugin_info_.has_value())
            {
                throw exceptions::RemoteException(slot_info_, plugin_info_.value(), status.error_message());
            }
            throw exceptions::RemoteException(slot_info_, status.error_message());
        }
        return ret_value;
    }

    template<v0::SlotID S>
    void broadcast(auto&&... args)
    {
        if (! plugin_info_->broadcast_subscriptions.contains(S))
        {
            return;
        }
        agrpc::GrpcContext grpc_context;
        grpc::Status status;

        boost::asio::co_spawn(
            grpc_context,
            [this, &grpc_context, &status, &args...]()
            {
                return this->broadcastCall<S>(grpc_context, status, std::forward<decltype(args)>(args)...);
            },
            boost::asio::detached);
        grpc_context.run();

        if (! status.ok()) // TODO: handle different kind of status codes
        {
            if (plugin_info_.has_value())
            {
                throw exceptions::RemoteException(slot_info_, plugin_info_.value(), status.error_message());
            }
            throw exceptions::RemoteException(slot_info_, status.error_message());
        }
    }

private:
    validator_type valid_{}; ///< The validator object for plugin validation.
    req_converter_type req_{}; ///< The Modify request converter object.
    rsp_converter_type rsp_{}; ///< The Modify response converter object.

    ranges::semiregular_box<modify_stub_t> modify_stub_; ///< The gRPC Modify stub for communication.
    ranges::semiregular_box<broadcast_stub_t> broadcast_stub_; ///< The gRPC Broadcast stub for communication.

    slot_metadata slot_info_{ .slot_id = SlotID,
                              .version_range = SlotVersionRng.value,
                              .engine_uuid = Application::getInstance().instance_uuid }; ///< Holds information about the plugin slot.
    std::optional<plugin_metadata> plugin_info_{ std::nullopt }; ///< Optional object that holds the plugin metadata, set after handshake

    /**
     * @brief Executes the modifyCall operation with the plugin.
     *
     * Sends a request to the plugin and saves the response.
     *
     * @param grpc_context - The gRPC context to use for the call
     * @param status - Status of the gRPC call which gets updated in this method
     * @param ret_value - Reference to the value in which response to be stored
     * @param args - Request arguments
     * @return A boost::asio::awaitable<void> indicating completion of the operation
     */
    boost::asio::awaitable<void> modifyCall(agrpc::GrpcContext& grpc_context, grpc::Status& status, value_type& ret_value, auto&&... args)
    {
        using RPC = agrpc::RPC<&modify_stub_t::PrepareAsyncCall>;
        grpc::ClientContext client_context{};
        prep_client_context(client_context);

        // Construct request
        auto request{ req_(std::forward<decltype(args)>(args)...) };

        // Make unary request
        rsp_msg_type response;
        status = co_await RPC::request(grpc_context, modify_stub_, client_context, request, response, boost::asio::use_awaitable);
        ret_value = rsp_(response);
        co_return;
    }

    template<v0::SlotID S>
    boost::asio::awaitable<void> broadcastCall(agrpc::GrpcContext& grpc_context, grpc::Status& status, auto&&... args)
    {
        grpc::ClientContext client_context{};
        prep_client_context(client_context);

        auto broadcaster{ details::broadcast_factory<broadcast_stub_t, S>() };
        auto request = details::broadcast_message_factory<S>(std::forward<decltype(args)>(args)...);
        auto response = google::protobuf::Empty{};
        status = co_await broadcaster.request(grpc_context, broadcast_stub_, client_context, request, response, boost::asio::use_awaitable);
        co_return;
    }

    /**
     * @brief Prepares client_context for the remote call.
     *
     * Sets timeout for the call and adds metadata to context.
     *
     * @param client_context - Client context to prepare
     * @param timeout - Call timeout duration (optional, default = 500ms)
     */
    void prep_client_context(grpc::ClientContext& client_context, std::chrono::milliseconds timeout = std::chrono::milliseconds(500))
    {
        // Set time-out
        client_context.set_deadline(std::chrono::system_clock::now() + timeout);

        // Metadata
        client_context.AddMetadata("cura-engine-uuid", slot_info_.engine_uuid.data());
        client_context.AddMetadata("cura-thread-id", fmt::format("{}", std::this_thread::get_id()));
    }
};
} // namespace cura::plugins

#endif // PLUGINS_PLUGINPROXY_H
