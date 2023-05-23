// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_PLUGINPROXY_H
#define PLUGINS_PLUGINPROXY_H

#include <agrpc/asio_grpc.hpp>
#include <agrpc/grpc_context.hpp>
#include <boost/asio/awaitable.hpp>
#include <boost/asio/co_spawn.hpp>
#include <boost/asio/detached.hpp>
#include <boost/asio/use_awaitable.hpp>
#include <range/v3/utility/semiregular_box.hpp>
#include <chrono>
#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include "plugins/exception.h"

namespace cura::plugins
{

/**
 * @brief A class template representing a proxy for a plugin.
 *
 * The PluginProxy class template facilitates communication with plugins by providing
 * an interface for sending requests and receiving responses. It uses gRPC for communication.
 *
 * @tparam Slot The plugin slot ID.
 * @tparam Validator The type used for validating the plugin.
 * @tparam Stub The process stub type.
 * @tparam Prepare The prepare type.
 * @tparam Request The gRPC convertible request type.
 * @tparam Response The gRPC convertible response type.
 */
template<plugins::SlotID Slot, details::CharRangeLiteral SlotVersionRng, class Validator, class Stub, class Prepare, class Request, grpc_convertable Response>
class PluginProxy
{
public:
    // type aliases for easy use
    using value_type = typename Response::native_value_type;

    using validator_t = Validator;

    using request_process_t = typename Request::value_type;
    using response_process_t = typename Response::value_type;

    using request_converter_t = Request;
    using response_converter_t = Response;

    using stub_t = Stub;

    static inline constexpr plugins::SlotID slot_id{ Slot };

private:
    validator_t valid_{}; ///< The validator object for plugin validation.
    request_converter_t request_converter_{}; ///< The request converter object.
    response_converter_t response_converter_{}; ///< The response converter object.

    grpc::Status status_; ///< The gRPC status object.
    ranges::semiregular_box<stub_t> stub_; ///< The gRPC stub for communication.
    std::string plugin_name_{}; ///< The name of the plugin.
    std::string plugin_version_{}; ///< The version of the plugin.
    std::string plugin_peer_{}; ///< The peer of the plugin.

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

    explicit PluginProxy(std::shared_ptr<grpc::Channel> channel) : stub_(channel)
    {
        // Give the plugin some time to start up and initiate handshake
        agrpc::GrpcContext grpc_context;
        boost::asio::co_spawn(grpc_context, wait_for_plugin_ready(grpc_context, channel), boost::asio::detached);
        grpc_context.run();
    }

    constexpr PluginProxy(const PluginProxy&) = default;
    constexpr PluginProxy(PluginProxy&&)  noexcept = default;
    constexpr PluginProxy& operator=(const PluginProxy& other)
    {
        if (this != &other)
        {
            valid_ = other.valid_;
            status_ = other.status_;
            stub_ = other.stub_;
            plugin_name_ = other.plugin_name_;
            plugin_version_ = other.plugin_version_;
            plugin_peer_ = other.plugin_peer_;
        }
        return *this;
    }

    constexpr PluginProxy& operator=(PluginProxy&& other)
    {
        if (this != &other)
        {
            valid_ = std::move(other.valid_);
            status_ = std::move(other.status_);
            stub_ = std::move(other.stub_); // FIXME: the stub should be moved
            plugin_name_ = std::move(other.plugin_name_);
            plugin_version_ = std::move(other.plugin_version_);
            plugin_peer_ = std::move(other.plugin_peer_);
        }
        return *this;
    }
    ~PluginProxy() = default;

    /**
     * @brief Executes the plugin operation.
     *
     * This operator allows the PluginProxy object to be invoked as a callable, which sends
     * a request to the plugin and waits for the response. The response is converted using
     * the response_converter_ object, and the converted value is returned.
     *
     * @tparam Args The argument types for the plugin request.
     * @param args The arguments for the plugin request.
     * @return The converted response value.
     *
     * @throws std::runtime_error if communication with the plugin fails.
     */
    value_type operator()(auto&&... args)
    {
        agrpc::GrpcContext grpc_context;
        value_type ret_value{};
        boost::asio::co_spawn(
            grpc_context,
            [&]() -> boost::asio::awaitable<void>
            {
                const auto deadline = std::chrono::system_clock::now() + std::chrono::milliseconds(10); // TODO use deadline
                grpc::ClientContext client_context{};
                request_process_t request{ request_converter_(std::forward<decltype(args)>(args)...) };
                response_process_t response{};
                status_ = co_await Prepare::request(grpc_context, stub_, client_context, request, response, boost::asio::use_awaitable);
                ret_value = response_converter_(response);
            },
            boost::asio::detached);
        grpc_context.run();

        if (! status_.ok())
        {
            throw std::runtime_error(fmt::format("Slot {} with plugin {} '{}' at {} had a communication failure: {}", slot_id, plugin_name_, plugin_version_, plugin_name_, status_.error_message()));
        }
        return ret_value;
    }

private:
    boost::asio::awaitable<void> wait_for_plugin_ready(agrpc::GrpcContext& grpc_context, std::shared_ptr<grpc::Channel> channel, const std::chrono::seconds& timeout = std::chrono::seconds(5))
    {
        const auto deadline = std::chrono::system_clock::now() + timeout;

        const auto state = channel->GetState(true);
        bool has_state_changed = co_await agrpc::notify_on_state_change(grpc_context, *channel, state, deadline);
        if (has_state_changed)
        {
            auto plugin_connection_state = channel->GetState(true);
            if (plugin_connection_state != grpc_connectivity_state::GRPC_CHANNEL_READY && plugin_connection_state != grpc_connectivity_state::GRPC_CHANNEL_CONNECTING)
            {
                throw std::runtime_error(fmt::format("Plugin for slot {} is not ready", slot_id));
            }
            //co_await handshake(grpc_context, timeout);
            if (! valid_)
            {
                throw exceptions::ValidatorException(valid_, plugin_name_, plugin_version_, plugin_peer_);
            }
        }
    }

//    boost::asio::awaitable<void> handshake(agrpc::GrpcContext& grpc_context, const std::chrono::seconds& timeout = std::chrono::seconds(5))
//    {
//        const auto deadline = std::chrono::system_clock::now() + timeout; // TODO use deadline
//        using RPC = agrpc::RPC<&stub_t::PrepareAsyncIdentify>;
//        grpc::ClientContext client_context{};
//        plugin_request<SlotVersionRng> plugin_request_conv{};
//        request_plugin_t request{ plugin_request_conv(slot_id) };
//        response_plugin_t response{};
//        auto status = co_await RPC::request(grpc_context, stub_, client_context, request, response, boost::asio::use_awaitable);
//        if (! status.ok())
//        {
//            throw std::runtime_error(fmt::format("Slot {} with plugin {} '{}' at {} had a communication failure: {}", slot_id, plugin_name_, plugin_version_, plugin_name_, status_.error_message()));
//        }
//        spdlog::debug("Plugin responded with: {}", response.DebugString());
//        plugin_response plugin_response_conv{};
//        const auto& [rsp_slot_id, rsp_plugin_name, rsp_slot_version, rsp_plugin_version] = plugin_response_conv(response);
//        plugin_name_ = rsp_plugin_name;
//        plugin_version_ = rsp_plugin_version;
//        plugin_peer_ = client_context.peer();
//        valid_ = Validator{ rsp_slot_version };
//    }
};
} // namespace cura::plugins

#endif // PLUGINS_PLUGINPROXY_H
