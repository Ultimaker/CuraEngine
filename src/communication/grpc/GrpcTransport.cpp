// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "communication/grpc/GrpcTransport.h"

#include <chrono>
#include <exception>
#include <fstream>
#include <utility>

#include <boost/asio/as_tuple.hpp>
#include <boost/asio/co_spawn.hpp>
#include <boost/asio/detached.hpp>
#include <boost/asio/experimental/awaitable_operators.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/use_awaitable.hpp>
#include <boost/asio/use_future.hpp>
#include <spdlog/spdlog.h>

#include "types/LayerIndex.h"


using namespace std::chrono_literals;
namespace proto = cura::proto;

namespace curaengine::communication::grpc
{

GrpcTransport::GrpcTransport(const std::string& endpoint, const ::grpc::ChannelArguments& arguments)
    : channel_(::grpc::CreateCustomChannel(endpoint, ::grpc::InsecureChannelCredentials(), arguments))
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
    const auto [send_error] = send_queue_.async_send(boost::system::error_code{}, std::move(envelope), boost::asio::as_tuple(boost::asio::use_future)).get();

    return ! send_error;
}

std::optional<proto::CuraClientMessage> GrpcTransport::takeNextMessage()
{
    auto [receive_error, received_message] = receive_queue_.async_receive(boost::asio::as_tuple(boost::asio::use_future)).get();
    if (receive_error)
    {
        return std::nullopt;
    }

    return std::move(received_message);
}

boost::asio::awaitable<void> GrpcTransport::handleIncomingMessages(RPC& rpc)
{
    while (true)
    {
        proto::CuraClientMessage incoming_message;
        if (! co_await rpc.read(incoming_message, boost::asio::use_awaitable))
        {
            if (receive_queue_.is_open()) // Means the communication channel was not intentionally closed
            {
                spdlog::error("Error when reading incoming gRPC message");
            }
            shutdown();
            co_return;
        }

        const auto [queue_error] = co_await receive_queue_.async_send(boost::system::error_code{}, std::move(incoming_message), boost::asio::as_tuple(boost::asio::use_awaitable));
        if (queue_error)
        {
            // The queue has been closed, which means we should stop
            co_return;
        }
    }
}

boost::asio::awaitable<void> GrpcTransport::handleSendingMessages(RPC& rpc)
{
    while (true)
    {
        auto [queue_error, outgoing_message] = co_await send_queue_.async_receive(boost::asio::as_tuple(boost::asio::use_awaitable));
        if (queue_error)
        {
            // The queue has been closed, which means we should stop
            co_return;
        }

        if (! co_await rpc.write(outgoing_message, boost::asio::use_awaitable))
        {
            if (send_queue_.is_open()) // Means the communication channel was not intentionally closed
            {
                spdlog::error("Error when sending gRPC message");
            }
            shutdown();
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
        shutdown();
        co_return;
    }

    // Spawns the read and write coroutines independently and wait until they are both completed
    co_await (handleIncomingMessages(rpc) && handleSendingMessages(rpc));

    // Make sure everything is stopped and unblocked, whatever the reason why the coroutines have ended
    shutdown();

    const ::grpc::Status status = co_await rpc.finish(use_awaitable);
    if (! status.ok() && status.error_code() != ::grpc::StatusCode::CANCELLED)
    {
        spdlog::warn("Cura gRPC stream ended unexpectedly: {}", status.error_message());
    }
}

void GrpcTransport::shutdown()
{
    // Close the queues first, which wakes up any waiting coroutine and marks the stop as intentional
    send_queue_.close();
    receive_queue_.close();

    // Then cancel the pending read/write operations. The context is forgotten afterwards, since the RPC object that owns it is about to be destroyed
    {
        std::lock_guard<std::mutex> lock(active_context_mutex_);
        if (client_context_)
        {
            client_context_->TryCancel();
            client_context_ = nullptr;
        }
    }
}

void GrpcTransport::stop()
{
    shutdown();

    // Wait until send and read loops have completed
    if (grpc_thread_.joinable())
    {
        grpc_thread_.join();
    }
}


} // namespace curaengine::communication::grpc
