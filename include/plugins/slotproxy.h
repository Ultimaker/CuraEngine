// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_INCLUDE_PLUGINS_SLOTPROXY_H
#define CURAENGINE_INCLUDE_PLUGINS_SLOTPROXY_H

#include <agrpc/grpc_context.hpp>
#include <boost/asio/awaitable.hpp>
#include <functional>
#include <memory>

#include <agrpc/asio_grpc.hpp>
#include <boost/asio/co_spawn.hpp>
#include <boost/asio/detached.hpp>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/channel.h>
#include <range/v3/utility/semiregular_box.hpp>
#include <spdlog/spdlog.h>

#include "plugin.grpc.pb.h"
#include "plugin.pb.h"
#include "plugins/types.h"
#include "plugins/validator.h"

namespace cura::plugins
{

template<plugins::SlotID Slot, class Validator, class Stub, converters::ReceiveCallable Receiver, converters::SendCallable Sender>
class SlotProxy
{
public:
    // type aliases for easy use
    using receive_t = Receiver;
    using send_t = Sender;
    using validator_t = Validator;
    using stub_t = Stub;

    static inline constexpr plugins::SlotID slot_id{ Slot };

private:
    proto::Plugin::Stub plugin_stub_;
    stub_t process_stub_;
    grpc::Status status_;

public:
    SlotProxy(std::shared_ptr<grpc::Channel> channel) : plugin_stub_(channel), process_stub_(channel)
    {
        agrpc::GrpcContext grpc_context;  // TODO: figure out how the reuse the grpc_context, it is recommended to use 1 per thread. Maybe move this to the lot registry??

        boost::asio::co_spawn(
            grpc_context,
            [&]() -> boost::asio::awaitable<void>
            {
                using RPC = agrpc::RPC<&proto::Plugin::Stub::PrepareAsyncIdentify>;
                grpc::ClientContext client_context{};
                proto::PluginRequest request{};
                request.set_id(slot_id);
                proto::PluginResponse response{};
                status_ = co_await RPC::request(grpc_context, plugin_stub_, client_context, request, response, boost::asio::use_awaitable);
                spdlog::info("Received response from plugin: {}", response.DebugString());
            },
            boost::asio::detached);
        grpc_context.run();
    }

    auto operator()(auto&&... args)
    {
        if (true) // validator && socket_->getState() == Arcus::SocketState::Connected)
        {
            //            socket_->sendMessage(converter_(std::forward<decltype(args)>(args)...));
            // TODO: Block until message is received
            // TODO: Convert return message to actual return value
            return 1; // FIXME: This is not correct
        }
        return 1; // FIXME: handle plugin not connected
    }
};

} // namespace cura::plugins


#endif // CURAENGINE_INCLUDE_PLUGINS_SLOTPROXY_H
