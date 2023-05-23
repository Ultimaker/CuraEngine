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
#include <chrono>
#include <fmt/format.h>
#include <range/v3/utility/semiregular_box.hpp>
#include <spdlog/spdlog.h>
#include <string>

#include "plugins/exception.h"
#include "plugins/metadata.h"

#include "cura/plugins/v1/slot_id.pb.h"
#include "utils/concepts/generic.h"

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
template<plugins::v1::SlotID SlotID, details::CharRangeLiteral SlotVersionRng, class Stub, class ValidatorTp, grpc_convertable RequestTp, grpc_convertable ResponseTp>
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

    using stub_t = Stub;

private:
    validator_type valid_{}; ///< The validator object for plugin validation.
    req_converter_type req_{}; ///< The request converter object.
    rsp_converter_type rsp_{}; ///< The response converter object.

    ranges::semiregular_box<stub_t> stub_; ///< The gRPC stub for communication.

    constexpr static slot_metadata slot_info_{ .slot_id = SlotID, .version_range = SlotVersionRng.value };
    std::optional<plugin_metadata> plugin_info_{ std::nullopt }; ///< The plugin info object.

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
    explicit PluginProxy(std::shared_ptr<grpc::Channel> channel) : stub_(channel){};

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
        grpc::Status status;

        boost::asio::co_spawn(
            grpc_context,
            [this, &status, &grpc_context, &ret_value, &args...]() -> boost::asio::awaitable<void>
            {
                using RPC = agrpc::RPC<&stub_t::PrepareAsyncModify>;
                grpc::ClientContext client_context{};

                // Set time-out
                client_context.set_deadline(std::chrono::system_clock::now() + std::chrono::milliseconds(500)); // TODO: don't use magic number and make it realistic

                // Metadata
                client_context.AddMetadata("cura-slot-service-name", fmt::format("{}", slot_info_.slot_id));
                client_context.AddMetadata("cura-slot-version-range", slot_info_.version_range.data());

                // Construct request
                auto request{ req_(std::forward<decltype(args)>(args)...) };

                // Make unary request
                rsp_msg_type response;
                status = co_await RPC::request(grpc_context, stub_, client_context, request, response, boost::asio::use_awaitable);
                ret_value = rsp_(response);

                if (! plugin_info_.has_value())
                {
                    plugin_info_ = plugin_metadata{ client_context };
                    valid_ = validator_type{ slot_info_, plugin_info_.value() };
                }
            },
            boost::asio::detached);
        grpc_context.run();

        if (! status.ok())  // TODO: handle different kind of status codes
        {
            if (plugin_info_.has_value())
            {
                throw exceptions::RemoteException(slot_info_, plugin_info_.value(), status.error_message());
            }
            throw exceptions::RemoteException(slot_info_, status.error_message());
        }

        if (! valid_ )
        {
            if (plugin_info_.has_value())
            {
                throw exceptions::ValidatorException(valid_, slot_info_, plugin_info_.value());
            }
            throw exceptions::ValidatorException(valid_, slot_info_);
        }

        return ret_value;  // TODO: check if ret_value is always filled or if we need a solution like: https://stackoverflow.com/questions/67908591/how-to-convert-boostasioawaitable-to-stdfuture
    }
};
} // namespace cura::plugins

#endif // PLUGINS_PLUGINPROXY_H
