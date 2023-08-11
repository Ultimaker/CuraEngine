// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_BROADCASTCOMPONENT_H
#define PLUGINS_BROADCASTCOMPONENT_H

#include "cura/plugins/v0/slot_id.pb.h"
#include "plugins/broadcasts.h"
#include "plugins/components/common.h"
#include "plugins/exception.h"
#include "plugins/metadata.h"
#include "utils/format/thread_id.h"
#include "utils/types/char_range_literal.h"
#include "utils/types/generic.h"

#include <agrpc/asio_grpc.hpp>
#include <agrpc/client_rpc.hpp>
#include <agrpc/grpc_context.hpp>
#include <boost/asio.hpp>
#include <boost/asio/awaitable.hpp>
#include <boost/asio/co_spawn.hpp>
#include <boost/asio/detached.hpp>
#include <boost/asio/use_awaitable.hpp>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <range/v3/utility/semiregular_box.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <type_traits>

namespace cura::plugins
{

template<plugins::v0::SlotID S> // NOTE: Leave slot here (templated) for the case where we have to specialize broadcast-channels by slot.
class PluginProxyBroadcastComponent
{
public:
    constexpr PluginProxyBroadcastComponent() = default;

    PluginProxyBroadcastComponent(details::slot_info_ptr slot_info, details::plugin_info_ptr plugin_info, std::shared_ptr<grpc::Channel> channel)
        : slot_info_{ slot_info }
        , plugin_info_{ plugin_info }
        , broadcast_stub_{ channel }
    {
    }

    template<plugins::v0::SlotID Subscription>
    void broadcast(auto&&... args)
    {
        if (! plugin_info_->value().broadcast_subscriptions.contains(Subscription))
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
            if (plugin_info_->has_value())
            {
                throw exceptions::RemoteException(*slot_info_, plugin_info_->value(), status.error_message());
            }
            throw exceptions::RemoteException(*slot_info_, status.error_message());
        }
    }

private:
    template<plugins::v0::SlotID Subscription>
    boost::asio::awaitable<void> broadcastCall(agrpc::GrpcContext& grpc_context, grpc::Status& status, auto&&... args)
    {
        grpc::ClientContext client_context{};
        prep_client_context(client_context, *slot_info_);

        using request_type = details::broadcast_rpc<Subscription>;
        request_type request{};

        auto response = google::protobuf::Empty{};
        status = co_await request_type::ClientRPC::request(
            grpc_context,
            broadcast_stub_,
            client_context,
            request(std::forward<decltype(args)>(args)...),
            response,
            boost::asio::use_awaitable);
        co_return;
    }

    details::slot_info_ptr slot_info_;
    details::plugin_info_ptr plugin_info_;
    ranges::semiregular_box<details::broadcast_stub<>::stub_type> broadcast_stub_; ///< The gRPC Broadcast stub for communication.
};

} // namespace cura::plugins

#endif // PLUGINS_BROADCASTCOMPONENT_H
