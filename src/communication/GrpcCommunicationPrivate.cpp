// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "communication/GrpcCommunicationPrivate.h"

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

#include "ExtruderTrain.h"
#include "Slice.h"
#include "settings/types/LayerIndex.h"
#include "utils/Matrix4x3D.h" //To convert vertices to integer-points.
#include "utils/MeshUtils.h"
#include "utils/Point3F.h" //To accept vertices (which are provided in floating point).

using namespace std::chrono_literals;

namespace cura
{

GrpcCommunication::Private::Private(const std::string& endpoint, const grpc::ChannelArguments& arguments)
    : channel(grpc::CreateCustomChannel(endpoint, grpc::InsecureChannelCredentials(), arguments))
    , stub(proto::CuraEngineCommunication::NewStub(channel))
    , last_sent_progress(-1)
    , send_channel(grpc_context.get_executor(), 4096)
    , receive_channel(grpc_context.get_executor(), 16)
{
    grpc_thread = std::thread(
        [this]
        {
            boost::asio::co_spawn(grpc_context, runStream(), boost::asio::detached);
            grpc_context.run();
        });

    connected = channel->WaitForConnected(std::chrono::system_clock::now() + 10s);
}

bool GrpcCommunication::Private::sendMessage(const google::protobuf::Message& message)
{
    proto::CuraEngineMessage envelope;
    if (const auto* progress = dynamic_cast<const proto::Progress*>(&message))
    {
        *envelope.mutable_progress() = *progress;
        // spdlog::info(">>>>>>>>>> Sending progress");
    }
    else if (const auto* layer = dynamic_cast<const proto::Layer*>(&message))
    {
        *envelope.mutable_layer() = *layer;
        // spdlog::info(">>>>>>>>>> Sending layer");
    }
    else if (const auto* layer = dynamic_cast<const proto::LayerOptimized*>(&message))
    {
        *envelope.mutable_layer_optimized() = *layer;
        // spdlog::info(">>>>>>>>>> Sending layer_optimized");
    }
    else if (const auto* gcode = dynamic_cast<const proto::GCodeLayer*>(&message))
    {
        *envelope.mutable_gcode_layer() = *gcode;
        // spdlog::info(">>>>>>>>>> Sending gcode");
    }
    else if (const auto* uuid = dynamic_cast<const proto::SliceUUID*>(&message))
    {
        *envelope.mutable_slice_uuid() = *uuid;
        // spdlog::info(">>>>>>>>>> Sending slice UUID");
    }
    else if (const auto* estimates = dynamic_cast<const proto::PrintTimeMaterialEstimates*>(&message))
    {
        *envelope.mutable_print_time_material_estimates() = *estimates;
        // spdlog::info(">>>>>>>>>> Sending estimates");
    }
    else if (const auto* extruder = dynamic_cast<const proto::InitialExtruder*>(&message))
    {
        *envelope.mutable_initial_extruder() = *extruder;
        // spdlog::info(">>>>>>>>>> Sending initial extruder");
    }
    else if (const auto* finished = dynamic_cast<const proto::SlicingFinished*>(&message))
    {
        *envelope.mutable_slicing_finished() = *finished;
        // spdlog::info(">>>>>>>>>> Sending finished");
    }
    else
    {
        spdlog::error("Unsupported Cura gRPC output message type: {}", message.GetTypeName());
        return false;
    }

    send_channel.async_send(boost::system::error_code{}, envelope, boost::asio::use_future).get();

    // const size_t message_bytes = envelope.ByteSizeLong();
    // constexpr size_t max_queued_bytes = 500 * 1024 * 1024;
    // if (message_bytes > max_queued_bytes)
    // {
    //     return false;
    // }
    // std::unique_lock<std::mutex> lock(send_queue_mutex);
    // send_queue_space.wait(
    //     lock,
    //     [this, message_bytes, max_queued_bytes]
    //     {
    //         return stream_failed || ! connected || message_bytes <= max_queued_bytes - send_queue_bytes;
    //     });
    // if (! connected || stream_failed)
    // {
    //     return false;
    // }
    // send_queue_bytes += message_bytes;
    // send_queue.push_back(std::move(envelope));
    return true;
}

// bool GrpcCommunication::Private::flushMessages()
// {
//     std::unique_lock<std::mutex> lock(send_queue_mutex);
//     send_queue_space.wait(
//         lock,
//         [this]
//         {
//             return stream_failed || ! connected || (send_queue.empty() && send_queue_bytes == 0);
//         });
//     return connected && ! stream_failed;
// }

std::optional<proto::CuraClientMessage> GrpcCommunication::Private::takeNextMessage()
{
    std::optional<proto::CuraClientMessage> message = receive_channel.async_receive(boost::asio::use_future).get();

    // if (error_code.failed())
    // {
    // return std::nullopt;
    // }

    // std::unique_lock<std::mutex> lock(message_mutex);
    // message_condition.wait(
    //     lock,
    //     [this]
    //     {
    //         return ! incoming_messages.empty() || stream_failed || ! connected;
    //     });
    // if (incoming_messages.empty() || stream_failed)
    // {
    //     return std::nullopt;
    // }


    // auto result = std::make_optional(std::move(incoming_messages.front()));

    if (message->has_start_slice())
    {
        spdlog::info("<<<<<<<<<< Received start slice");
    }
    else if (message->has_exit_engine())
    {
        spdlog::info("<<<<<<<<<< Received engine exit");
    }
    else if (message->has_mesh_group())
    {
        spdlog::info("<<<<<<<<<< Received mesh group");
    }
    else if (message->has_object())
    {
        spdlog::info("<<<<<<<<<< Received object");
    }
    else if (message->has_slice())
    {
        spdlog::info("<<<<<<<<<< Received slice");
    }

    // incoming_messages.pop_front();
    return message;
}

bool GrpcCommunication::Private::waitForSliceOrExit()
{
    std::unique_lock<std::mutex> lock(message_mutex);
    message_condition.wait(
        lock,
        [this]
        {
            return ! incoming_messages.empty() || exit_requested || stream_failed;
        });
    return ! incoming_messages.empty() && ! stream_failed;
}

boost::asio::awaitable<void> GrpcCommunication::Private::readLoop(RPC& rpc)
{
    using boost::asio::use_awaitable;
    try
    {
        proto::CuraClientMessage message;
        while (! stream_failed && co_await rpc.read(message, use_awaitable))
        {
            // if (message.has_exit_engine())
            // {
            //     std::lock_guard<std::mutex> lock(message_mutex);
            //     exit_requested = true;
            //     incoming_messages.clear();
            //     message_condition.notify_all();
            // }
            // else
            {
                co_await receive_channel.async_send(boost::system::error_code{}, message, use_awaitable);
                // std::lock_guard<std::mutex> lock(message_mutex);
                // incoming_messages.push_back(std::move(message));
                // message_condition.notify_all();
            }
        }
    }
    catch (const std::exception& error)
    {
        spdlog::warn("Cura gRPC stream read failed: {}", error.what());
    }
    catch (...)
    {
        spdlog::warn("Cura gRPC stream read failed with an unknown error.");
    }

    std::unique_lock<std::mutex> slice_lock(message_mutex);
    stream_failed = true;
    read_finished = true;
    connected = false;
    message_condition.notify_all();
    slice_lock.unlock();
    rpc.cancel();
    send_queue_space.notify_all();
}

boost::asio::awaitable<void> GrpcCommunication::Private::writeLoop(RPC& rpc)
{
    using boost::asio::use_awaitable;
    // auto executor = co_await boost::asio::this_coro::executor;
    // boost::asio::steady_timer polling_timer(executor);
    while (! stream_failed)
    {
        std::optional<proto::CuraEngineMessage> message;
        try
        {
            message = co_await send_channel.async_receive(use_awaitable);
        }
        catch (boost::asio::experimental::error::channel_errors error)
        {
            spdlog::error("BADDDD channel");
            co_return;
        }
        // proto::CuraEngineMessage& message = std::get<1>(message_to_send);
        // send_channel.async_receive(
        //     [&message](proto::CuraEngineMessage& received_message)
        //     {
        //         message = std::move(received_message);
        //     });
        // if (ec == boost::asio::error::eof)
        // {
        //     break;
        // }

        // {
        //     std::lock_guard<std::mutex> lock(send_queue_mutex);
        //     if (! send_queue.empty())
        //     {
        //         send_queue_bytes -= send_queue.front().ByteSizeLong();
        //         message = std::move(send_queue.front());
        //         send_queue.pop_front();
        //     }
        // }

        if (! message.has_value())
        {
            // No more messages, quit
            co_return;
        }

#if 1
        if (message->has_progress())
        {
            spdlog::info(">>>>>>>>>> Actually Sending progress");
        }
        else if (message->has_layer())
        {
            spdlog::info(">>>>>>>>>> Actually Sending layer");
        }
        else if (message->has_layer_optimized())
        {
            spdlog::info(">>>>>>>>>> Actually Sending layer_optimized");
        }
        else if (message->has_gcode_layer())
        {
            spdlog::info(">>>>>>>>>> Actually Sending gcode");
        }
        else if (message->has_slice_uuid())
        {
            spdlog::info(">>>>>>>>>> Actually Sending slice UUID");
        }
        else if (message->has_print_time_material_estimates())
        {
            spdlog::info(">>>>>>>>>> Actually Sending estimates");
        }
        else if (message->has_initial_extruder())
        {
            spdlog::info(">>>>>>>>>> Actually Sending initial extruder");
        }
        else if (message->has_slicing_finished())
        {
            spdlog::info(">>>>>>>>>> Actually Sending finished");
        }
#endif

        const size_t message_bytes = message->ByteSizeLong();
        {
            std::lock_guard<std::mutex> lock(send_queue_mutex);
            send_queue_bytes += message_bytes;
        }
        bool write_succeeded = false;
        try
        {
            write_succeeded = co_await rpc.write(*message, use_awaitable);
        }
        catch (...)
        {
            stream_failed = true;
            spdlog::warn("Cura gRPC write threw while sending an output envelope.");
        }
        {
            std::lock_guard<std::mutex> lock(send_queue_mutex);
            send_queue_bytes -= message_bytes;
        }
        message_condition.notify_all();
        send_queue_space.notify_all();
        if (! write_succeeded || stream_failed)
        {
            stream_failed = true;
            rpc.cancel();
            co_return;
        }

        // polling_timer.expires_after(std::chrono::milliseconds(2));
        // co_await polling_timer.async_wait(use_awaitable);
    }
}

boost::asio::awaitable<void> GrpcCommunication::Private::runStream()
{
    using boost::asio::use_awaitable;
    using namespace boost::asio::experimental::awaitable_operators;

    RPC rpc{ grpc_context };
    {
        std::lock_guard<std::mutex> lock(active_context_mutex);
        active_context = &rpc.context();
    }

    if (! co_await rpc.start(*stub, use_awaitable))
    {
        stream_failed = true;
        read_finished = true;
        {
            std::lock_guard<std::mutex> lock(active_context_mutex);
            active_context = nullptr;
        }
        message_condition.notify_all();
        send_queue_space.notify_all();
        co_return;
    }

    connected = true;
    read_finished = false;
    exit_requested = false;
    message_condition.notify_all();
    co_await (readLoop(rpc) && writeLoop(rpc));
    // try
    // {
    //     co_await writeLoop(rpc);
    // }
    // catch (const std::exception& error)
    // {
    //     spdlog::warn("Cura gRPC stream write failed: {}", error.what());
    //     stream_failed = true;
    // }
    // catch (...)
    // {
    //     spdlog::warn("Cura gRPC stream write failed with an unknown error.");
    //     stream_failed = true;
    // }

    rpc.cancel();
    boost::asio::steady_timer reader_shutdown_timer(co_await boost::asio::this_coro::executor);
    while (! read_finished)
    {
        reader_shutdown_timer.expires_after(std::chrono::milliseconds(2));
        co_await reader_shutdown_timer.async_wait(use_awaitable);
    }

    const grpc::Status status = co_await rpc.finish(use_awaitable);
    connected = false;
    send_queue_space.notify_all();
    {
        std::lock_guard<std::mutex> lock(active_context_mutex);
        active_context = nullptr;
    }
    message_condition.notify_all();
    send_queue_space.notify_all();

    if (! status.ok())
    {
        spdlog::warn("Cura gRPC stream ended: {}", status.error_message());
    }
}

void GrpcCommunication::Private::stop()
{
    send_channel.try_send(boost::system::error_code{}, std::nullopt);

    stream_failed = true;
    connected = false;
    {
        std::lock_guard<std::mutex> lock(active_context_mutex);
        if (grpc::ClientContext* context = active_context.load())
        {
            context->TryCancel();
        }
    }

    message_condition.notify_all();
    if (grpc_thread.joinable())
    {
        grpc_thread.join();
    }
}

std::shared_ptr<proto::LayerOptimized> GrpcCommunication::Private::getOptimizedLayerById(LayerIndex::value_type layer_nr)
{
    layer_nr += optimized_layers.current_layer_offset;
    std::unordered_map<int, std::shared_ptr<proto::LayerOptimized>>::iterator find_result = optimized_layers.slice_data.find(layer_nr);

    if (find_result != optimized_layers.slice_data.end()) // Load layer from the cache.
    {
        return find_result->second;
    }
    else // Not in the cache yet. Create an empty layer.
    {
        std::shared_ptr<proto::LayerOptimized> layer = std::make_shared<proto::LayerOptimized>();
        layer->set_id(layer_nr);
        optimized_layers.current_layer_count++;
        optimized_layers.slice_data[layer_nr] = layer;
        return layer;
    }
}

void GrpcCommunication::Private::readGlobalSettingsMessage(const proto::SettingList& global_settings_message, const std::shared_ptr<Slice>& slice)
{
    for (const cura::proto::Setting& setting_message : global_settings_message.settings())
    {
        slice->scene.settings.add(setting_message.name(), setting_message.value());
    }
}

void GrpcCommunication::Private::readExtruderSettingsMessage(const google::protobuf::RepeatedPtrField<proto::Extruder>& extruder_messages, const std::shared_ptr<Slice>& slice)
{
    // Make sure we have enough extruders added currently.
    const size_t extruder_count = slice->scene.settings.get<size_t>("machine_extruder_count");
    for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        slice->scene.extruders.emplace_back(extruder_nr, &slice->scene.settings);
    }

    // Parse the extruder number and the settings from the messages.
    for (const cura::proto::Extruder& extruder_message : extruder_messages)
    {
        const int32_t extruder_nr = extruder_message.id(); // Cast from proto::int to int32_t!
        if (extruder_nr < 0 || extruder_nr >= static_cast<int32_t>(extruder_count))
        {
            spdlog::warn("Received extruder index that is out of range: {}", extruder_nr);
            continue;
        }
        ExtruderTrain& extruder
            = slice->scene
                  .extruders[extruder_nr]; // Extruder messages may arrive out of order, so don't iteratively get the next extruder but take the extruder_nr from this message.
        for (const cura::proto::Setting& setting_message : extruder_message.settings().settings())
        {
            extruder.settings_.add(setting_message.name(), setting_message.value());
        }
    }
}

void GrpcCommunication::Private::readMeshGroupMessage(const proto::MeshGroup& mesh_group_message, Scene& scene)
{
    auto mesh_group = std::make_shared<MeshGroup>();

    // Load the settings in the mesh group.
    for (const cura::proto::Setting& setting : mesh_group_message.settings())
    {
        mesh_group->settings.add(setting.name(), setting.value());
    }

    scene.appendMeshGroup(mesh_group);
}

void GrpcCommunication::Private::readObjectMessage(const proto::Object& object_message, Slice& slice, MeshGroup& mesh_group)
{
    Matrix4x3D matrix;
    constexpr size_t bytes_per_face = sizeof(Point3F) * 3; // 3 vectors per face.
    constexpr size_t bytes_per_uv = sizeof(Point2F) * 3; // 3 vectors per face.
    const size_t face_count = object_message.vertices().size() / bytes_per_face;

    if (face_count <= 0)
    {
        spdlog::warn("Got an empty mesh. Ignoring it!");
        return;
    }

    mesh_group.meshes.emplace_back();
    Mesh& mesh = mesh_group.meshes.back();

    // Load the settings for the mesh.
    for (const cura::proto::Setting& setting : object_message.settings())
    {
        mesh.settings_.add(setting.name(), setting.value());
    }

    // Set the parent setting to the correct extruder.
    const auto extruder_nr = mesh.settings_.get<size_t>("extruder_nr");
    ExtruderTrain& extruder = slice.scene.extruders[extruder_nr];
    mesh.settings_.setParent(&extruder.settings_);

    for (size_t face = 0; face < face_count; face++)
    {
        const std::string data = object_message.vertices().substr(face * bytes_per_face, bytes_per_face);
        const Point3F* float_vertices = reinterpret_cast<const Point3F*>(data.data());
        const std::string uv_coordinates_str = object_message.uv_coordinates().empty() ? "" : object_message.uv_coordinates().substr(face * bytes_per_uv, bytes_per_uv);
        const Point2F* uv_coordinates_data = uv_coordinates_str.empty() ? nullptr : reinterpret_cast<const Point2F*>(uv_coordinates_str.data());

        Point3LL verts[3];
        verts[0] = matrix.apply(float_vertices[0].toPoint3d());
        verts[1] = matrix.apply(float_vertices[1].toPoint3d());
        verts[2] = matrix.apply(float_vertices[2].toPoint3d());

        std::optional<Point2F> uv_coordinates[3];
        if (uv_coordinates_data)
        {
            uv_coordinates[0] = uv_coordinates_data[0];
            uv_coordinates[1] = uv_coordinates_data[1];
            uv_coordinates[2] = uv_coordinates_data[2];
        }

        mesh.addFace(verts[0], verts[1], verts[2], uv_coordinates[0], uv_coordinates[1], uv_coordinates[2]);
    }

    loadTextureData(object_message.texture(), mesh);

    mesh.mesh_name_ = object_message.name();
    mesh.finish();

    object_count++;
}

void GrpcCommunication::Private::loadTextureData(const std::string& texture_str, Mesh& mesh)
{
    MeshUtils::loadTextureFromString(texture_str, mesh);
}

} // namespace cura
