// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "communication/grpc/GrpcTransport.h"

#include <chrono>
#include <exception>
#include <fstream>
#include <utility>

#include <boost/asio/co_spawn.hpp>
#include <boost/asio/detached.hpp>
#include <boost/asio/experimental/awaitable_operators.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/use_awaitable.hpp>
#include <boost/asio/use_future.hpp>
#include <spdlog/spdlog.h>

#include "settings/types/LayerIndex.h"


using namespace std::chrono_literals;

namespace cura
{

GrpcTransport::GrpcTransport(const std::string& endpoint, const grpc::ChannelArguments& arguments)
    : channel_(grpc::CreateCustomChannel(endpoint, grpc::InsecureChannelCredentials(), arguments))
    , stub_(proto::CuraEngineCommunication::NewStub(channel_))
    , send_queue_(grpc_context_.get_executor(), send_channel_buffer_size)
    , receive_queue_(grpc_context_.get_executor(), receive_channel_buffer_size)
{
}

bool GrpcTransport::connect()
{
    bool connected = channel_->WaitForConnected(std::chrono::system_clock::now() + 10s);
    if (connected)
    {
        grpc_thread_ = std::thread(
            [this]
            {
                boost::asio::co_spawn(grpc_context_, runStream(), boost::asio::detached);
                grpc_context_.run();
            });
    }

    return connected;
}

bool GrpcTransport::sendMessage(google::protobuf::Message&& message)
{
    proto::CuraEngineMessage envelope;

    if (auto* progress = dynamic_cast<proto::Progress*>(&message))
    {
        *envelope.mutable_progress() = std::move(*progress);
    }
    else if (auto* layer = dynamic_cast<proto::Layer*>(&message))
    {
        *envelope.mutable_layer() = std::move(*layer);
    }
    else if (auto* layer = dynamic_cast<proto::LayerOptimized*>(&message))
    {
        *envelope.mutable_layer_optimized() = std::move(*layer);
    }
    else if (auto* gcode = dynamic_cast<proto::GCodeLayer*>(&message))
    {
        *envelope.mutable_gcode_layer() = std::move(*gcode);
    }
    else if (auto* uuid = dynamic_cast<proto::SliceUUID*>(&message))
    {
        *envelope.mutable_slice_uuid() = std::move(*uuid);
    }
    else if (auto* estimates = dynamic_cast<proto::PrintTimeMaterialEstimates*>(&message))
    {
        *envelope.mutable_print_time_material_estimates() = std::move(*estimates);
    }
    else if (auto* extruder = dynamic_cast<proto::InitialExtruder*>(&message))
    {
        *envelope.mutable_initial_extruder() = std::move(*extruder);
    }
    else if (auto* finished = dynamic_cast<proto::SlicingFinished*>(&message))
    {
        *envelope.mutable_slicing_finished() = std::move(*finished);
    }
    else
    {
        spdlog::error("Unsupported Cura gRPC output message type: {}", message.GetTypeName());
        return false;
    }

    // Push the envelope to the queue and wait until it has been actually added (but not yet written)
    send_queue_.async_send(boost::system::error_code{}, std::move(envelope), boost::asio::use_future).get();

    return true;
}

std::optional<proto::CuraClientMessage> GrpcTransport::takeNextMessage()
{
    return receive_queue_.async_receive(boost::asio::use_future).get();
}

boost::asio::awaitable<void> GrpcTransport::handleIncomingMessages(RPC& rpc)
{
    while (running_)
    {
        proto::CuraClientMessage message;
        if (! co_await rpc.read(message, boost::asio::use_awaitable))
        {
            if (running_)
            {
                spdlog::error("Error when reading incoming gRPC message");
            }
            co_return;
        }

        co_await receive_queue_.async_send(boost::system::error_code{}, std::move(message), boost::asio::use_awaitable);
    }
}

boost::asio::awaitable<void> GrpcTransport::handleSendingMessages(RPC& rpc)
{
    while (running_)
    {
        std::optional<proto::CuraEngineMessage> message = co_await send_queue_.async_receive(boost::asio::use_awaitable);
        if (! message.has_value())
        {
            // Empty message means we should stop
            co_return;
        }

        if (! co_await rpc.write(*message, boost::asio::use_awaitable))
        {
            spdlog::error("Error when sending gRPC message");
            co_return;
        }
    }
}

boost::asio::awaitable<void> GrpcTransport::runStream()
{
    using boost::asio::use_awaitable;
    using namespace boost::asio::experimental::awaitable_operators;

    RPC rpc{ grpc_context_ };
    {
        std::lock_guard<std::mutex> lock(active_context_mutex_);
        client_context_ = &rpc.context();
    }

    if (! co_await rpc.start(*stub_, use_awaitable))
    {
        spdlog::error("Error when starting gRPC streaming");
        co_return;
    }

    // Spawns the read and write coroutines independently and wait until they are both completed
    co_await (handleIncomingMessages(rpc) && handleSendingMessages(rpc));

    const grpc::Status status = co_await rpc.finish(use_awaitable);
    if (! status.ok() && status.error_code() != grpc::StatusCode::CANCELLED)
    {
        spdlog::warn("Cura gRPC stream ended unexpectedly: {}", status.error_message());
    }
}

void GrpcTransport::stop()
{
    // Notify the send loop to gently stop by adding an empty message
    send_queue_.async_send(boost::system::error_code{}, std::nullopt, boost::asio::detached);

    // Notify the read loop to gently stop by setting running to false and cancelling the client context
    running_ = false;
    {
        std::lock_guard<std::mutex> lock(active_context_mutex_);
        if (client_context_)
        {
            client_context_->TryCancel();
        }
    }

    // Wait until send and read loops have completed
    if (grpc_thread_.joinable())
    {
        grpc_thread_.join();
    }
}


} // namespace cura
