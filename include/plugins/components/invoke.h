// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_INVOKECOMPONENT_H
#define PLUGINS_INVOKECOMPONENT_H

#include "common.h"
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
#include <boost/asio/awaitable.hpp>
#include <boost/asio/co_spawn.hpp>
#include <boost/asio/detached.hpp>
#include <boost/asio/use_awaitable.hpp>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <range/v3/utility/semiregular_box.hpp>
#include <spdlog/spdlog.h>

namespace cura::plugins
{
namespace details
{

template<typename T>
concept plugin_invoker = requires(T value)
{
    requires std::is_member_function_pointer_v<decltype(&T::PrepareAsyncCall)>;
};

template<typename T>
concept not_plugin_invoker = ! plugin_invoker<T>;

} // namespace details

template<class Stub, typename RequestTp, typename ResponseTp>
class PluginProxyInvokeComponent
{
public:
    constexpr PluginProxyInvokeComponent();
    PluginProxyInvokeComponent(details::slot_info_ptr slot_info, details::plugin_info_ptr plugin_info, std::shared_ptr<grpc::Channel> channel);
    auto invoke(auto&&... args);
};

template<details::not_plugin_invoker Stub, typename RequestTp, typename ResponseTp>
class PluginProxyInvokeComponent<Stub, RequestTp, ResponseTp>
{
public:
    constexpr PluginProxyInvokeComponent() = default;

    PluginProxyInvokeComponent(details::slot_info_ptr slot_info, details::plugin_info_ptr plugin_info, std::shared_ptr<grpc::Channel> channel)
    {
    }

    auto invoke(auto&&... args)
    {
        assert(false); // Invoke called on a stub which it isn't meant for (for example, broadcast), should not actually be called.
        return 0;
    }
};

template<details::plugin_invoker Stub, utils::grpc_convertable RequestTp, utils::grpc_convertable ResponseTp>
class PluginProxyInvokeComponent<Stub, RequestTp, ResponseTp>
{
    using value_type = typename ResponseTp::native_value_type;

    using req_converter_type = RequestTp;
    using rsp_converter_type = ResponseTp;
    using rsp_msg_type = typename ResponseTp::value_type;

    using invoke_stub_t = Stub;

public:
    constexpr PluginProxyInvokeComponent() = default;

    PluginProxyInvokeComponent(details::slot_info_ptr slot_info, details::plugin_info_ptr plugin_info, std::shared_ptr<grpc::Channel> channel)
        : slot_info_{ slot_info }
        , plugin_info_{ plugin_info }
        , invoke_stub_{ channel }
    {
    }

    /**
     * @brief Executes to plugin Invoke (modify/generate) operation.
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
    auto invoke(auto&&... args)
    {
        agrpc::GrpcContext grpc_context;
        value_type ret_value{};
        grpc::Status status;

        boost::asio::co_spawn(
            grpc_context,
            [this, &grpc_context, &status, &ret_value, &args...]()
            {
                return this->invokeCall(grpc_context, status, ret_value, std::forward<decltype(args)>(args)...);
            },
            boost::asio::detached);
        grpc_context.run();

        if (! status.ok()) // TODO: handle different kind of status codes
        {
            if (plugin_info_->has_value())
            {
                throw exceptions::RemoteException(*slot_info_, plugin_info_->value(), status.error_message());
            }
            throw exceptions::RemoteException(*slot_info_, status.error_message());
        }
        return ret_value;
    }

private:
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
    boost::asio::awaitable<void> invokeCall(agrpc::GrpcContext& grpc_context, grpc::Status& status, value_type& ret_value, auto&&... args)
    {
        using RPC = agrpc::ClientRPC<&invoke_stub_t::PrepareAsyncCall>;
        grpc::ClientContext client_context{};
        prep_client_context(client_context, *slot_info_);

        // Construct request
        auto request{ req_(std::forward<decltype(args)>(args)...) };

        // Make unary request
        rsp_msg_type response;
        status = co_await RPC::request(grpc_context, invoke_stub_, client_context, request, response, boost::asio::use_awaitable);
        ret_value = rsp_(response);
        co_return;
    }

    req_converter_type req_{}; ///< The Invoke request converter object.
    rsp_converter_type rsp_{}; ///< The Invoke response converter object.

    details::slot_info_ptr slot_info_;
    details::plugin_info_ptr plugin_info_;
    ranges::semiregular_box<invoke_stub_t> invoke_stub_; ///< The gRPC Invoke stub for communication.
};

} // namespace cura::plugins

#endif // PLUGINS_INVOKECOMPONENT_H
