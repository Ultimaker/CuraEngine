// Copyright (c) 2018 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef GRPCCOMMUNICATIONPRIVATE_H
#define GRPCCOMMUNICATIONPRIVATE_H

#include <atomic>
#include <condition_variable>
#include <deque>
#include <google/protobuf/message.h>
#include <grpcpp/grpcpp.h>
#include <mutex>
#include <optional>
#include <sstream> //For ostringstream.
#include <thread>

#include <agrpc/asio_grpc.hpp>
#include <boost/asio/awaitable.hpp>
#include <boost/asio/steady_timer.hpp>

#include "Cura.grpc.pb.h"
#include "GrpcCommunication.h" //We're adding a subclass to this.
#include "SliceDataStruct.h"
#include "settings/types/LayerIndex.h"

namespace cura
{

class MeshGroup;
class Scene;
class Mesh;

class GrpcCommunication::Private
{
    friend class GrpcCommunicationTest;

    using RPC = agrpc::ClientRPC<&proto::CuraEngineCommunication::Stub::PrepareAsyncCommunicate>;

public:
    Private();

    /*
     * Get the optimised layer data for a specific layer.
     * \param layer_nr The layer number to get the optimised layer data for.
     * \return The optimised layer data for that layer.
     */
    std::shared_ptr<proto::LayerOptimized> getOptimizedLayerById(LayerIndex::value_type layer_nr);

    /*
     * Reads the global settings from a Protobuf message.
     *
     * The global settings are stored in the current scene.
     * \param global_settings_message The global settings message.
     */
    void readGlobalSettingsMessage(const proto::SettingList& global_settings_message, const std::shared_ptr<Slice>& slice);

    std::shared_ptr<Slice> readSliceMessage(const proto::Slice& slice_message);

    void readExtruderSettingsMessage(const google::protobuf::RepeatedPtrField<proto::Extruder>& extruder_messages, const std::shared_ptr<Slice>& slice);

    /*
     * \brief Reads a Protobuf message describing a mesh group.
     *
     * This gets the vertex data from the message as well as the settings.
     */
    void readMeshGroupMessage(const proto::MeshGroup& mesh_group_message, Scene& scene);

    void readObjectMessage(const proto::Object& object_message, Slice& slice, MeshGroup& mesh_group);

    std::shared_ptr<grpc::Channel> channel;
    std::unique_ptr<proto::CuraEngineCommunication::Stub> stub;
    agrpc::GrpcContext grpc_context;
    std::thread grpc_thread;
    std::mutex active_context_mutex;
    std::atomic<grpc::ClientContext*> active_context{ nullptr };
    std::atomic<bool> connected{ false };
    std::atomic<bool> stream_failed{ false };
    std::atomic<bool> read_finished{ false };
    std::mutex message_mutex;
    std::condition_variable message_condition;
    std::deque<proto::CuraClientMessage> incoming_messages;
    std::atomic<bool> exit_requested{ false };
    std::mutex send_queue_mutex;
    std::condition_variable send_queue_space;
    std::deque<proto::CuraEngineMessage> send_queue;
    std::atomic<size_t> send_queue_bytes{ 0 };
    size_t object_count; //!< Number of objects that need to be sliced.
    std::string temp_gcode_file; //!< Temporary buffer for the g-code.
    std::ostringstream gcode_output_stream; //!< The stream to write g-code to.

    SliceDataStruct<cura::proto::Layer> sliced_layers;
    SliceDataStruct<cura::proto::LayerOptimized> optimized_layers;

    int last_sent_progress; //!< Last sent progress promille (1/1000th). Used to not send duplicate messages with the same promille.

    const size_t millisecUntilNextTry; // Delay between stream readiness checks.

    bool sendMessage(const google::protobuf::Message& message);
    bool flushMessages();
    bool waitForSliceOrExit();
    std::optional<proto::CuraClientMessage> takeNextMessage();
    boost::asio::awaitable<void> runStream();
    boost::asio::awaitable<void> readLoop(RPC& rpc);
    boost::asio::awaitable<void> writeLoop(RPC& rpc);
    void stop();

private:
    static void loadTextureData(const std::string& texture_str, Mesh& mesh);
};

} // namespace cura

#endif // GRPCCOMMUNICATIONPRIVATE_H
