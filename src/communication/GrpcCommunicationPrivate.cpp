// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "communication/GrpcCommunicationPrivate.h"

#include <chrono>
#include <exception>
#include <fstream>
#include <png.h>
#include <rapidjson/document.h>
#include <rapidjson/error/en.h>
#include <rapidjson/memorystream.h>
#include <utility>

#include <boost/asio/co_spawn.hpp>
#include <boost/asio/detached.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/use_awaitable.hpp>
#include <spdlog/spdlog.h>

#include "ExtruderTrain.h"
#include "Slice.h"
#include "settings/types/LayerIndex.h"
#include "utils/Matrix4x3D.h" //To convert vertices to integer-points.
#include "utils/MeshUtils.h"
#include "utils/Point3F.h" //To accept vertices (which are provided in floating point).

namespace cura
{

GrpcCommunication::Private::Private()
    : object_count(0)
    , last_sent_progress(-1)
    , millisecUntilNextTry(100)
{
}

bool GrpcCommunication::Private::sendMessage(const google::protobuf::Message& message)
{
    proto::CuraEngineMessage envelope;
    if (const auto* progress = dynamic_cast<const proto::Progress*>(&message))
        *envelope.mutable_progress() = *progress;
    else if (const auto* layer = dynamic_cast<const proto::Layer*>(&message))
        *envelope.mutable_layer() = *layer;
    else if (const auto* layer = dynamic_cast<const proto::LayerOptimized*>(&message))
        *envelope.mutable_layer_optimized() = *layer;
    else if (const auto* gcode = dynamic_cast<const proto::GCodeLayer*>(&message))
        *envelope.mutable_gcode_layer() = *gcode;
    else if (const auto* uuid = dynamic_cast<const proto::SliceUUID*>(&message))
        *envelope.mutable_slice_uuid() = *uuid;
    else if (const auto* estimates = dynamic_cast<const proto::PrintTimeMaterialEstimates*>(&message))
        *envelope.mutable_print_time_material_estimates() = *estimates;
    else if (const auto* extruder = dynamic_cast<const proto::InitialExtruder*>(&message))
        *envelope.mutable_initial_extruder() = *extruder;
    else if (const auto* finished = dynamic_cast<const proto::SlicingFinished*>(&message))
        *envelope.mutable_slicing_finished() = *finished;
    else
    {
        spdlog::error("Unsupported Cura gRPC output message type: {}", message.GetTypeName());
        return false;
    }

    const size_t message_bytes = envelope.ByteSizeLong();
    constexpr size_t max_queued_bytes = 500 * 1024 * 1024;
    if (message_bytes > max_queued_bytes)
        return false;
    while (true)
    {
        if (! connected || stream_failed)
            return false;
        size_t queued_bytes = send_queue_bytes.load();
        if (queued_bytes > max_queued_bytes - message_bytes)
        {
            std::unique_lock<std::mutex> lock(send_queue_mutex);
            send_queue_space.wait(
                lock,
                [this, message_bytes]
                {
                    return stream_failed || ! connected || send_queue_bytes.load() <= max_queued_bytes - message_bytes;
                });
            continue;
        }
        if (send_queue_bytes.compare_exchange_weak(queued_bytes, queued_bytes + message_bytes))
        {
            std::lock_guard<std::mutex> lock(send_queue_mutex);
            send_queue.push_back(std::move(envelope));
            return true;
        }
    }
}

bool GrpcCommunication::Private::flushMessages()
{
    std::unique_lock<std::mutex> lock(send_queue_mutex);
    send_queue_space.wait(
        lock,
        [this]
        {
            return stream_failed || ! connected || send_queue_bytes.load() == 0;
        });
    return connected && ! stream_failed;
}

std::optional<proto::CuraClientMessage> GrpcCommunication::Private::takeNextMessage()
{
    std::unique_lock<std::mutex> lock(message_mutex);
    message_condition.wait(
        lock,
        [this]
        {
            return ! incoming_messages.empty() || stream_failed || ! connected;
        });
    if (incoming_messages.empty() || stream_failed)
        return std::nullopt;

    auto result = std::make_optional(std::move(incoming_messages.front()));
    incoming_messages.pop_front();
    return result;
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
            if (message.has_exit_engine())
            {
                std::lock_guard<std::mutex> lock(message_mutex);
                exit_requested = true;
                incoming_messages.clear();
                message_condition.notify_all();
            }
            else
            {
                std::lock_guard<std::mutex> lock(message_mutex);
                incoming_messages.push_back(std::move(message));
                message_condition.notify_all();
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
    auto executor = co_await boost::asio::this_coro::executor;
    boost::asio::steady_timer polling_timer(executor);
    while (! stream_failed)
    {
        std::optional<proto::CuraEngineMessage> message;
        {
            std::lock_guard<std::mutex> lock(send_queue_mutex);
            if (! send_queue.empty())
            {
                message = std::move(send_queue.front());
                send_queue.pop_front();
            }
        }

        if (message.has_value())
        {
            if (co_await rpc.write(*message, use_awaitable))
            {
                send_queue_bytes.fetch_sub(message->ByteSizeLong());
                send_queue_space.notify_all();
            }
            else
            {
                stream_failed = true;
                rpc.cancel();
                co_return;
            }
            continue;
        }

        polling_timer.expires_after(std::chrono::milliseconds(2));
        co_await polling_timer.async_wait(use_awaitable);
    }
}

boost::asio::awaitable<void> GrpcCommunication::Private::runStream()
{
    using boost::asio::use_awaitable;
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
    exit_requested = false;
    message_condition.notify_all();
    boost::asio::co_spawn(grpc_context, readLoop(rpc), boost::asio::detached);
    try
    {
        co_await writeLoop(rpc);
    }
    catch (const std::exception& error)
    {
        spdlog::warn("Cura gRPC stream write failed: {}", error.what());
        stream_failed = true;
    }
    catch (...)
    {
        spdlog::warn("Cura gRPC stream write failed with an unknown error.");
        stream_failed = true;
    }
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
        spdlog::warn("Cura gRPC stream ended: {}", status.error_message());
}

void GrpcCommunication::Private::stop()
{
    stream_failed = true;
    connected = false;
    {
        std::lock_guard<std::mutex> lock(active_context_mutex);
        if (grpc::ClientContext* context = active_context.load())
            context->TryCancel();
    }
    message_condition.notify_all();
    if (grpc_thread.joinable())
        grpc_thread.join();
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
}

void GrpcCommunication::Private::loadTextureData(const std::string& texture_str, Mesh& mesh)
{
    MeshUtils::loadTextureFromString(texture_str, mesh);
}

} // namespace cura
