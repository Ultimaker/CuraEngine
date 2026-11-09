// Copyright (c) 2026 UltiMaker
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

/*!
 * Class that handles the asynchronous sending and receiving of protobuf messages through a gRPC communication channel
 */
class GrpcTransport
{
public:
    /*!
     * Constructor, which immediately tries to connect to the given endpoint
     * @param endpoint The endpoint to connect to, which should be something like "127.0.0.1:49674"
     * @param arguments Extra arguments to be used for the channel
     */
    GrpcTransport(const std::string& endpoint, const grpc::ChannelArguments& arguments);

    /*!
     * Makes sures the communication channel is properly connected and start the messages receiving and sending thread
     * @return True if the communication channel is properly connected
     */
    bool connect();

    /*!
     * Send a message to the remote server
     * @param message The message to be sent
     * @return True if the message could be sent, false if it is somehow invalid
     * @note The message is not sent immediately but pushed to the queue for later sending
     */
    bool sendMessage(google::protobuf::Message&& message);

    /*!
     * Blocks until a message has been received and returns it
     * @return The received message
     */
    std::optional<proto::CuraClientMessage> takeNextMessage();

    /*! Gently stop the communication with the remote gRPC server */
    void stop();

private:
    using RPC = agrpc::ClientRPC<&proto::CuraEngineCommunication::Stub::PrepareAsyncCommunicate>;

    // Channels are used as asynchronous producer/consumer queue. If their buffer happen to be full, they will block until the message can be finally added,
    // so there is no data loss anyway, but the buffer size should be fine-tuned to be large enough to allow a smooth messages stream without overwhelming the memory.
    static constexpr size_t send_channel_buffer_size = 8; // Sent messages can take a large amount of memory, and sending them is very fast, so we keep the buffer size very low.
    static constexpr size_t receive_channel_buffer_size = 2; // Received messages can be huge (contain a whole 3D model), so keep the buffer even smaller.

    /*! Main coroutine that handles the sending and receiving of messages */
    boost::asio::awaitable<void> runStream();

    /*! Secondary coroutine that handles the receiving of messages when there are some available */
    boost::asio::awaitable<void> handleIncomingMessages(RPC& rpc);

    /*! Secondary coroutine that handles the sending of messages when there are some available */
    boost::asio::awaitable<void> handleSendingMessages(RPC& rpc);

    std::shared_ptr<grpc::Channel> channel_; //!< The gPRC communication channel
    std::unique_ptr<proto::CuraEngineCommunication::Stub> stub_; //!< The object that connects the channel with the protobuf service
    agrpc::GrpcContext grpc_context_; //!< The asio-gRPC event pump that runs the data transmission and waking up of coroutines
    std::thread grpc_thread_; //!< The thread in which the gRPC communication is processed
    std::mutex active_context_mutex_; //!< Mutex around the client_context_ variable
    grpc::ClientContext* client_context_{ nullptr }; //!< The context that contains the current connection to the server
    std::atomic<bool> running_{ true }; //!< Indicates that the gRPC client is running, or supposed to
    //!< The asynchronous producer/consumer queue containing the messages to be sent
    boost::asio::experimental::concurrent_channel<void(boost::system::error_code, std::optional<proto::CuraEngineMessage>)> send_queue_;
    //!< The asynchronous producer/consumer queue containing the received messages
    boost::asio::experimental::concurrent_channel<void(boost::system::error_code, std::optional<proto::CuraClientMessage>)> receive_queue_;
};

} // namespace cura
