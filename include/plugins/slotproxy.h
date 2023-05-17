// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_INCLUDE_PLUGINS_SLOTPROXY_H
#define CURAENGINE_INCLUDE_PLUGINS_SLOTPROXY_H

#include <exception>
#include <functional>
#include <memory>

#include <agrpc/asio_grpc.hpp>
#include <agrpc/grpc_context.hpp>
#include <boost/asio/awaitable.hpp>
#include <boost/asio/co_spawn.hpp>
#include <boost/asio/detached.hpp>
#include <fmt/format.h>
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <range/v3/utility/semiregular_box.hpp>
#include <spdlog/spdlog.h>

#include "plugins/converters.h"
#include "plugins/types.h"
#include "plugins/validator.h"

#include "plugin.grpc.pb.h"

namespace cura::plugins
{

template<plugins::SlotID Slot, class Validator, class Stub, class Prepare, class Request, class Response>
class SlotProxy
{
public:
    // type aliases for easy use
    using value_type = typename Response::native_value_type;

    using request_plugin_t = typename plugin_request::value_type;
    using response_plugin_t = typename plugin_response::value_type;

    using request_process_t = typename Request::value_type;
    using response_process_t = typename Response::value_type;

    using request_converter_t = Request;
    using response_converter_t = Response;

    using validator_t = Validator;
    using process_stub_t = Stub;

    static inline constexpr plugins::SlotID slot_id{ Slot };

private:
    validator_t valid_{};
    request_converter_t request_converter_{};
    response_converter_t response_converter_{};

    grpc::Status status_;

    proto::Plugin::Stub plugin_stub_;
    process_stub_t process_stub_;

public:
    SlotProxy(std::shared_ptr<grpc::Channel> channel) : plugin_stub_(channel), process_stub_(channel)
    {
        agrpc::GrpcContext grpc_context; // TODO: figure out how the reuse the grpc_context, it is recommended to use 1 per thread. Maybe move this to the lot registry??

        boost::asio::co_spawn(
            grpc_context,
            [&]() -> boost::asio::awaitable<void>
            {
                using RPC = agrpc::RPC<&proto::Plugin::Stub::PrepareAsyncIdentify>;
                grpc::ClientContext client_context{};
                plugin_request plugin_request_conv{};
                request_plugin_t request{ plugin_request_conv(slot_id) };
                response_plugin_t response{};
                status_ = co_await RPC::request(grpc_context, plugin_stub_, client_context, request, response, boost::asio::use_awaitable);
                plugin_response plugin_response_conv{};
                auto [version, _] = plugin_response_conv(response);
                spdlog::debug("Received response from plugin '{}': {}", slot_id, response.DebugString());
                valid_ = Validator{ version };
                spdlog::info("Plugin: {} validated: {}", slot_id, static_cast<bool>(valid_));
            },
            boost::asio::detached);
        grpc_context.run();

        if (! valid_)
        {
            if (! valid_.valid_version())
            {
                throw std::runtime_error(fmt::format("Could not validate plugin '{}' due to an invalid version '{}', expected '{}'!", slot_id, valid_.getVersion(), valid_.version_range));
            }
            throw std::runtime_error(fmt::format("Could not validate plugin '{}' for an unknown reason!", slot_id));
        }

        if (! status_.ok())
        {
            throw std::runtime_error(fmt::format("Communication with plugin '{}' {}", slot_id, status_.error_message()));
        }
    }

    value_type operator()(auto&&... args)
    {
        value_type ret_value{};
        if (valid_)
        {
            agrpc::GrpcContext grpc_context; // TODO: figure out how the reuse the grpc_context, it is recommended to use 1 per thread. Maybe move this to the lot registry??

            boost::asio::co_spawn(
                grpc_context,
                [&]() -> boost::asio::awaitable<void>
                {
                    grpc::ClientContext client_context{};
                    request_process_t request{ request_converter_(std::forward<decltype(args)>(args)...) };
                    response_process_t response{};
                    status_ = co_await Prepare::request(grpc_context, process_stub_, client_context, request, response, boost::asio::use_awaitable);
                    spdlog::debug("Received response from plugin '{}': {}", slot_id, response.DebugString());
                    ret_value = response_converter_(response);
                },
                boost::asio::detached);
            grpc_context.run();
        }

        if (! status_.ok())
        {
            throw std::runtime_error(fmt::format("Communication with plugin '{}' failed, due: {}", slot_id, status_.error_message()));
        }
        return ret_value; // FIXME: handle plugin not connected
    }
};

} // namespace cura::plugins


#endif // CURAENGINE_INCLUDE_PLUGINS_SLOTPROXY_H
