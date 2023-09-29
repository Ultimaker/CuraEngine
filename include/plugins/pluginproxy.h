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
#include <agrpc/client_rpc.hpp>
#include <agrpc/grpc_context.hpp>
#include <agrpc/use_awaitable.hpp>
#include <boost/asio/awaitable.hpp>
#include <boost/asio/co_spawn.hpp>
#include <boost/asio/detached.hpp>
#include <boost/asio/use_awaitable.hpp>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <range/v3/utility/semiregular_box.hpp>
#include <spdlog/spdlog.h>

#include <chrono>
#if __has_include(<coroutine>)
#include <coroutine>
#elif __has_include(<experimental/coroutine>)
#include <experimental/coroutine>
#define USE_EXPERIMENTAL_COROUTINE
#endif
#include <memory>
#include <optional>
#include <string>
#include <thread>

namespace cura::plugins
{

/**
 * @brief A plugin proxy class template.
 *
 * Template arguments are:
 * SlotID - plugin slot ID
 * SlotVersion - slot version used -- will be the only version available in the engine for that slot, since there is just the latest version of the slot a.t.m.
 * Stub - process stub type
 * ValidatorTp - validator type
 * RequestTp - gRPC convertible request type, or dummy -- if stub is a proper invoke-stub, this is enforced by the specialization of the invoke component
 * ResponseTp - gRPC convertible response type, or dummy -- if stub is a proper invoke-stub, this is enforced by the specialization of the invoke component
 *
 * Class provides methods for validating the plugin, making requests and processing responses.
 */
template<plugins::v0::SlotID SlotID, utils::CharRangeLiteral SlotVersion, class Stub, class ValidatorTp, typename RequestTp, typename ResponseTp>
class PluginProxy
{
    // type aliases for easy use
    using value_type = typename ResponseTp::native_value_type;
    using validator_type = ValidatorTp;
    using req_converter_type = RequestTp;
    using rsp_converter_type = ResponseTp;
    using rsp_msg_type = typename ResponseTp::value_type;
    using invoke_stub_t = Stub;
    using broadcast_stub_t = slots::broadcast::v0::BroadcastService::Stub;

    ranges::semiregular_box<invoke_stub_t> invoke_stub_; ///< The gRPC Invoke stub for communication.
    ranges::semiregular_box<broadcast_stub_t> broadcast_stub_; ///< The gRPC Broadcast stub for communication.
public:
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

    PluginProxy(const std::string& name, const std::string& version, std::shared_ptr<grpc::Channel> channel)
        : invoke_stub_{ channel }
        , broadcast_stub_{ channel }
    {
        // Connect to the plugin and exchange a handshake
        agrpc::GrpcContext grpc_context;
        grpc::Status status;
        slots::handshake::v0::HandshakeService::Stub handshake_stub(channel);
        plugin_metadata plugin_info;

        boost::asio::co_spawn(
            grpc_context,
            [this, &grpc_context, &status, &plugin_info, &handshake_stub, &name, &version]() -> boost::asio::awaitable<void>
            {
                using RPC = agrpc::ClientRPC<&slots::handshake::v0::HandshakeService::Stub::PrepareAsyncCall>;
                grpc::ClientContext client_context{};
                prep_client_context(client_context, slot_info_);

                // Construct request
                handshake_request handshake_req;
                handshake_request::value_type request{ handshake_req(name, version, slot_info_) };

                // Make unary request
                handshake_response::value_type response;
                status = co_await RPC::request(grpc_context, handshake_stub, client_context, request, response, boost::asio::use_awaitable);
                handshake_response handshake_rsp;
                plugin_info = handshake_rsp(response, client_context.peer());
                valid_ = validator_type{ slot_info_, plugin_info };
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
            spdlog::error(status.error_message());
            throw exceptions::RemoteException(slot_info_, status.error_message());
        }
        if (! plugin_info.plugin_name.empty() && ! plugin_info.slot_version_range.empty())
        {
            plugin_info_.emplace(plugin_info);
        }
    };

    constexpr PluginProxy(const PluginProxy&) = default;
    constexpr PluginProxy(PluginProxy&&) noexcept = default;
    constexpr PluginProxy& operator=(const PluginProxy& other)
    {
        if (this != &other)
        {
            invoke_stub_ = other.invoke_stub_;
            broadcast_stub_ = other.broadcast_stub_;
            valid_ = other.valid_;
            plugin_info_ = other.plugin_info_;
            slot_info_ = other.slot_info_;
        }
        return *this;
    }
    constexpr PluginProxy& operator=(PluginProxy&& other) noexcept
    {
        if (this != &other)
        {
            invoke_stub_ = std::move(other.invoke_stub_);
            broadcast_stub_ = std::move(other.broadcast_stub_);
            valid_ = std::move(other.valid_);
            plugin_info_ = std::move(other.plugin_info_);
            slot_info_ = std::move(other.slot_info_);
        }
        return *this;
    }
    ~PluginProxy() = default;

    value_type generate(auto&&... args)
    {
        agrpc::GrpcContext grpc_context;
        value_type ret_value{};
        grpc::Status status;

        boost::asio::co_spawn(
            grpc_context,
            [this, &grpc_context, &status, &ret_value, &args...]()
            {
                return this->generateCall(grpc_context, status, ret_value, std::forward<decltype(args)>(args)...);
            },
            boost::asio::detached);
        grpc_context.run();

        if (! status.ok()) // TODO: handle different kind of status codes
        {
            if (plugin_info_.has_value())
            {
                spdlog::error(
                    "Plugin '{}' running at [{}] for slot {} failed with error: {}",
                    plugin_info_.value().plugin_name,
                    plugin_info_.value().peer,
                    slot_info_.slot_id,
                    status.error_message());
                throw exceptions::RemoteException(slot_info_, plugin_info_.value(), status.error_message());
            }
            spdlog::error("Plugin for slot {} failed with error: {}", slot_info_.slot_id, status.error_message());
            throw exceptions::RemoteException(slot_info_, status.error_message());
        }
        return ret_value;
    }

    value_type modify(auto& original_value, auto&&... args)
    {
        agrpc::GrpcContext grpc_context;
        value_type ret_value{};
        grpc::Status status;

        boost::asio::co_spawn(
            grpc_context,
            [this, &grpc_context, &status, &ret_value, &original_value, &args...]()
            {
                return this->modifyCall(grpc_context, status, ret_value, original_value, std::forward<decltype(args)>(args)...);
            },
            boost::asio::detached);
        grpc_context.run();

        if (! status.ok()) // TODO: handle different kind of status codes
        {
            if (plugin_info_.has_value())
            {
                spdlog::error(
                    "Plugin '{}' running at [{}] for slot {} failed with error: {}",
                    plugin_info_.value().plugin_name,
                    plugin_info_.value().peer,
                    slot_info_.slot_id,
                    status.error_message());
                throw exceptions::RemoteException(slot_info_, plugin_info_.value(), status.error_message());
            }
            spdlog::error("Plugin for slot {} failed with error: {}", slot_info_.slot_id, status.error_message());
            throw exceptions::RemoteException(slot_info_, status.error_message());
        }
        return ret_value;
    }

    template<plugins::v0::SlotID Subscription>
    void broadcast(auto&&... args)
    {
        if (! plugin_info_.value().broadcast_subscriptions.contains(Subscription))
        {
            return;
        }
        agrpc::GrpcContext grpc_context;
        grpc::Status status;

        boost::asio::co_spawn(
            grpc_context,
            [this, &grpc_context, &status, &args...]()
            {
                return this->broadcastCall<Subscription>(grpc_context, status, std::forward<decltype(args)>(args)...);
            },
            boost::asio::detached);
        grpc_context.run();

        if (! status.ok()) // TODO: handle different kind of status codes
        {
            if (plugin_info_.has_value())
            {
                spdlog::error(
                    "Plugin '{}' running at [{}] for slot {} failed with error: {}",
                    plugin_info_.value().plugin_name,
                    plugin_info_.value().peer,
                    slot_info_.slot_id,
                    status.error_message());
                throw exceptions::RemoteException(slot_info_, plugin_info_.value(), status.error_message());
            }
            spdlog::error("Plugin for slot {} failed with error: {}", slot_info_.slot_id, status.error_message());
            throw exceptions::RemoteException(slot_info_, status.error_message());
        }
    }

private:
    inline static void prep_client_context(grpc::ClientContext& client_context, const slot_metadata& slot_info, const std::chrono::milliseconds& timeout = std::chrono::minutes(5))
    {
        // Set time-out
        client_context.set_deadline(std::chrono::system_clock::now() + timeout);

        // Metadata
        client_context.AddMetadata("cura-engine-uuid", slot_info.engine_uuid.data());
        client_context.AddMetadata("cura-thread-id", fmt::format("{}", std::this_thread::get_id()));
    }

    /**
     * @brief Executes the invokeCall operation with the plugin.
     *
     * Sends a request to the plugin and saves the response.
     *
     * @param grpc_context - The gRPC context to use for the call
     * @param status - Status of the gRPC call which gets updated in this method
     * @param ret_value - Reference to the value in which response to be stored
     * @param args - Request arguments
     * @return A boost::asio::awaitable<void> indicating completion of the operation
     */
    boost::asio::awaitable<void> generateCall(agrpc::GrpcContext& grpc_context, grpc::Status& status, value_type& ret_value, auto&&... args)
    {
        using RPC = agrpc::ClientRPC<&invoke_stub_t::PrepareAsyncCall>;
        grpc::ClientContext client_context{};
        prep_client_context(client_context, slot_info_);

        // Construct request
        auto request{ req_(std::forward<decltype(args)>(args)...) };

        // Make unary request
        rsp_msg_type response;
        status = co_await RPC::request(grpc_context, invoke_stub_, client_context, request, response, boost::asio::use_awaitable);
        ret_value = rsp_(response);
        co_return;
    }

    boost::asio::awaitable<void> modifyCall(agrpc::GrpcContext& grpc_context, grpc::Status& status, value_type& ret_value, auto& original_value, auto&&... args)
    {
        using RPC = agrpc::ClientRPC<&invoke_stub_t::PrepareAsyncCall>;
        grpc::ClientContext client_context{};
        prep_client_context(client_context, slot_info_);

        // Construct request
        auto request{ req_(original_value, std::forward<decltype(args)>(args)...) };

        // Make unary request
        rsp_msg_type response;
        status = co_await RPC::request(grpc_context, invoke_stub_, client_context, request, response, boost::asio::use_awaitable);
        ret_value = std::move(rsp_(original_value, response));
        co_return;
    }

    template<plugins::v0::SlotID Subscription>
    boost::asio::awaitable<void> broadcastCall(agrpc::GrpcContext& grpc_context, grpc::Status& status, auto&&... args)
    {
        grpc::ClientContext client_context{};
        prep_client_context(client_context, slot_info_);
        using RPC = agrpc::ClientRPC<&broadcast_stub_t::PrepareAsyncBroadcastSettings>;

        details::broadcast_rpc<Subscription, broadcast_stub_t> requester{};
        auto request = requester(std::forward<decltype(args)>(args)...);

        auto response = google::protobuf::Empty{};
        status = co_await RPC::request(grpc_context, broadcast_stub_, client_context, request, response, boost::asio::use_awaitable);
        co_return;
    }

    validator_type valid_{}; ///< The validator object for plugin validation.
    req_converter_type req_{}; ///< The Invoke request converter object.
    rsp_converter_type rsp_{}; ///< The Invoke response converter object.
    slot_metadata slot_info_{ .slot_id = SlotID,
                              .version = SlotVersion.value,
                              .engine_uuid = Application::getInstance().instance_uuid }; ///< Holds information about the plugin slot.
    std::optional<plugin_metadata> plugin_info_{ std::optional<plugin_metadata>(std::nullopt) }; ///< Optional object that holds the plugin metadata, set after handshake
};

} // namespace cura::plugins

#endif // PLUGINS_PLUGINPROXY_H
