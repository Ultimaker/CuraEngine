// Copyright (c) 2018 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <atomic>
#include <condition_variable>
#include <google/protobuf/message.h>
#include <grpcpp/grpcpp.h>
#include <mutex>
#include <optional>
#include <thread>

#include <agrpc/asio_grpc.hpp>
#include <boost/asio/awaitable.hpp>
#include <boost/asio/experimental/concurrent_channel.hpp>
#include <boost/asio/steady_timer.hpp>

#include "Cura.grpc.pb.h"

namespace cura
{

class GrpcTransport
{
    friend class GrpcCommunicationTest;

public:
    GrpcTransport(const std::string& endpoint, const grpc::ChannelArguments& arguments);

    bool connect();
    bool sendMessage(google::protobuf::Message&& message);
    std::optional<proto::CuraClientMessage> takeNextMessage();
    void stop();

private:
    using RPC = agrpc::ClientRPC<&proto::CuraEngineCommunication::Stub::PrepareAsyncCommunicate>;

    // Channels are used as asynchronous producer/consumer queue. If their buffer happen to be full, they will block until the message can be finally added,
    // so there is no data loss anyway, but the buffer size should be fine-tuned to be large enough to allow a smooth messages stream without overwhelming the memory.
    static constexpr size_t send_channel_buffer_size = 8; // Sent messages can take a large amount of memory, and sending them is very fast, so we keep the buffer size very low.
    static constexpr size_t receive_channel_buffer_size = 2; // Received messages can be huge (contain a whole 3D model), so keep the buffer even smaller.

    boost::asio::awaitable<void> runStream();
    boost::asio::awaitable<void> handleIncomingMessages(RPC& rpc);
    boost::asio::awaitable<void> handleSendingMessages(RPC& rpc);

    std::shared_ptr<grpc::Channel> channel_;
    std::unique_ptr<proto::CuraEngineCommunication::Stub> stub_;
    agrpc::GrpcContext grpc_context_;
    std::thread grpc_thread_;
    std::mutex active_context_mutex_;
    grpc::ClientContext* client_context_{ nullptr };
    std::atomic<bool> running_{ true };
    boost::asio::experimental::concurrent_channel<void(boost::system::error_code, std::optional<proto::CuraEngineMessage>)> send_channel_;
    boost::asio::experimental::concurrent_channel<void(boost::system::error_code, std::optional<proto::CuraClientMessage>)> receive_channel_;
};

} // namespace cura
