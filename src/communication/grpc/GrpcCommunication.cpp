// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "communication/grpc/GrpcCommunication.h"

#ifdef SENTRY_URL
#ifdef _WIN32
#if ! defined(NOMINMAX)
#define NOMINMAX
#endif
#if ! defined(WIN32_LEAN_AND_MEAN)
#define WIN32_LEAN_AND_MEAN
#endif
#endif
#include <sentry.h>
#endif

#include <chrono>
#include <google/protobuf/message.h>
#include <unordered_map> //To map settings to their extruder numbers for limit_to_extruder.

#include <fmt/format.h>
#include <spdlog/details/os.h>
#include <spdlog/spdlog.h>

#include "Application.h" //To get and set the current slice command.
#include "Cura.pb.h"
#include "ExtruderTrain.h"
#include "FffProcessor.h" //To start a slice.
#include "PrintFeature.h"
#include "Slice.h" //To process slices.
#include "communication/grpc/GrpcTransport.h"
#include "communication/grpc/PathCompiler.h"
#include "communication/grpc/SliceDataStruct.h" //To store sliced layer data.
#include "plugins/slots.h"
#include "settings/types/LayerIndex.h" //To point to layers.
#include "settings/types/Velocity.h" //To send to layer view how fast stuff is printing.
#include "utils/MeshUtils.h"
#include "utils/Point3F.h"
#include "utils/channel.h"

namespace cura
{

GrpcCommunication::GrpcCommunication()
{
}

GrpcCommunication::~GrpcCommunication()
{
    spdlog::info("Closing connection.");
    grpc_transport_->stop();
}

bool GrpcCommunication::connect(const std::string& endpoint)
{
    spdlog::info("Connecting to Cura gRPC service at {}", endpoint);

    grpc::ChannelArguments arguments;
    arguments.SetMaxReceiveMessageSize(500 * 1024 * 1024);
    arguments.SetMaxSendMessageSize(500 * 1024 * 1024);
    grpc_transport_ = std::make_unique<GrpcTransport>(endpoint, arguments);

    if (! grpc_transport_->connect())
    {
        spdlog::error("Could not establish Cura gRPC stream at {}", endpoint);
        grpc_transport_->stop();
        return false;
    }

    path_compiler = std::make_unique<PathCompiler>();
    return true;
}

void GrpcCommunication::sendGCodePart(const std::string& gcode_part)
{
    const std::string message_str = slots::instance().modify<plugins::v0::SlotID::POSTPROCESS_MODIFY>(gcode_part);
    if (message_str.empty())
    {
        return;
    }

    proto::GCodeLayer message;
    message.set_data(message_str);
    grpc_transport_->sendMessage(std::move(message));
}

std::shared_ptr<Slice> GrpcCommunication::makeSlice(const proto::Slice& slice_message)
{
    auto slice = std::make_shared<Slice>();

#ifdef SENTRY_URL
    sentry_value_t user = sentry_value_new_object();
    sentry_value_set_by_key(user, "id", sentry_value_new_string(slice_message.sentry_id().c_str()));
    if (slice_message.has_user_name())
    {
        spdlog::debug("Setting Sentry user to {}", slice_message.user_name());
        sentry_value_set_by_key(user, "username", sentry_value_new_string(slice_message.user_name().c_str()));
    }
    sentry_set_user(user);
    sentry_set_tag("cura.version", slice_message.cura_version().c_str());
    if (slice_message.has_project_name())
    {
        sentry_set_tag("cura.project_name", slice_message.project_name().c_str());
    }
#endif

#ifdef ENABLE_PLUGINS
    for (const auto& plugin : slice_message.engine_plugins())
    {
        const auto slot_id = static_cast<plugins::v0::SlotID>(plugin.id());
        slots::instance().connect(slot_id, plugin.plugin_name(), plugin.plugin_version(), utils::createChannel({ plugin.address(), plugin.port() }));
#ifdef SENTRY_URL
        sentry_set_tag(fmt::format("plugin_{}.version", plugin.plugin_name()).c_str(), plugin.plugin_version().c_str());
#endif
    }
#endif // ENABLE_PLUGINS

    readGlobalSettingsMessage(slice_message.global_settings(), slice->scene.settings);
    readExtruderSettingsMessage(slice_message.extruders(), slice->scene);

    const size_t extruder_count = slice->scene.extruders.size();

    // For each setting, register what extruder it should be obtained from (if this is limited to an extruder).
    for (const proto::SettingExtruder& setting_extruder : slice_message.limit_to_extruder())
    {
        const int32_t extruder_nr = setting_extruder.extruder(); // Cast from proto::int to int32_t!
        if (extruder_nr < 0 || extruder_nr > static_cast<int32_t>(extruder_count))
        {
            // If it's -1 it should be ignored as per the spec. Let's also ignore it if it's beyond range.
            continue;
        }
        ExtruderTrain& extruder = slice->scene.extruders[setting_extruder.extruder()];
        slice->scene.limit_to_extruder.emplace(setting_extruder.name(), &extruder);
    }

    return slice;
}

bool GrpcCommunication::hasSlice()
{
    current_slice_.reset();

    std::optional<proto::CuraClientMessage> message;
    for (message = grpc_transport_->takeNextMessage(); message.has_value() && ! message->has_start_slice() && ! message->has_exit_engine();
         message = grpc_transport_->takeNextMessage())
    {
        if (message->has_slice())
        {
            current_slice_ = makeSlice(message->slice());
        }
        else if (message->has_mesh_group())
        {
            if (! current_slice_)
            {
                spdlog::error("Received mesh group message without a current slice.");
                return false;
            }

            readMeshGroupMessage(message->mesh_group(), current_slice_->scene);
        }
        else if (message->has_object())
        {
            if (! current_slice_)
            {
                spdlog::error("Received object message without a current slice.");
                return false;
            }

            if (current_slice_->scene.mesh_groups.empty())
            {
                spdlog::error("Received object message without a mesh group.");
                return false;
            }

            readObjectMessage(message->object(), current_slice_->scene.extruders, *current_slice_->scene.mesh_groups.back());
        }
    }

    return message.has_value() && message->has_start_slice() && current_slice_;
}

void GrpcCommunication::sendCurrentPosition(const Point3LL& position)
{
    path_compiler->setCurrentPosition(position);
}

void GrpcCommunication::sendSliceUUID(const std::string& slice_uuid) const
{
    proto::SliceUUID message;
    message.set_slice_uuid(slice_uuid);
    grpc_transport_->sendMessage(std::move(message));
}

void GrpcCommunication::sendFinishedSlicing() const
{
    grpc_transport_->sendMessage(proto::SlicingFinished());
    spdlog::debug("Sent slicing finished message.");
}

void GrpcCommunication::sendLayerComplete(const LayerIndex::value_type& layer_nr, const coord_t& z, const coord_t& thickness)
{
    proto::LayerOptimized* layer = path_compiler->getOptimizedLayerById(layer_nr);
    layer->set_height(z);
    layer->set_thickness(thickness);
}

void GrpcCommunication::sendLineTo(
    const PrintFeatureType& type,
    const Point3LL& to,
    const coord_t& line_width,
    const coord_t& line_thickness,
    const Velocity& velocity,
    const PrintSegmentAttributes& print_attributes)
{
    path_compiler->sendLineTo(type, to, line_width, line_thickness, velocity, print_attributes);
}

void GrpcCommunication::sendOptimizedLayerData()
{
    path_compiler->flushPathSegments(); // Make sure the last path segment has been flushed from the compiler.

    SliceDataStruct<proto::LayerOptimized>& data = path_compiler->getOptimizedLayers();
    data.sliced_objects++;
    data.current_layer_offset = data.current_layer_count;
    if (data.sliced_objects < Application::getInstance().current_slice_->scene.mesh_groups.size()) // Nothing to send.
    {
        return;
    }
    spdlog::info("Sending {} layers.", data.current_layer_count);

    for (auto& entry : data.slice_data) // Note: This is in no particular order!
    {
        spdlog::debug("Sending layer data for layer {} of {}.", entry.first, data.slice_data.size());
        grpc_transport_->sendMessage(std::move(entry.second)); // Send the actual layers.
    }

    data.sliced_objects = 0;
    data.current_layer_count = 0;
    data.current_layer_offset = 0;
    data.slice_data.clear();
}

void GrpcCommunication::sendPrintInformation(const std::vector<Duration>& time_estimates, const PrintInformation& print_information) const
{
    spdlog::debug("Sending print information.");

    proto::PrintTimeMaterialEstimates message;

    message.set_time_infill(time_estimates[static_cast<unsigned char>(PrintFeatureType::Infill)]);
    message.set_time_inset_0(time_estimates[static_cast<unsigned char>(PrintFeatureType::OuterWall)]);
    message.set_time_inset_x(time_estimates[static_cast<unsigned char>(PrintFeatureType::InnerWall)]);
    message.set_time_none(time_estimates[static_cast<unsigned char>(PrintFeatureType::NoneType)]);
    message.set_time_retract(time_estimates[static_cast<unsigned char>(PrintFeatureType::StationaryRetractUnretract)]);
    message.set_time_skin(time_estimates[static_cast<unsigned char>(PrintFeatureType::Skin)]);
    message.set_time_skirt(time_estimates[static_cast<unsigned char>(PrintFeatureType::SkirtBrim)]);
    message.set_time_support(time_estimates[static_cast<unsigned char>(PrintFeatureType::Support)]);
    message.set_time_support_infill(time_estimates[static_cast<unsigned char>(PrintFeatureType::SupportInfill)]);
    message.set_time_support_interface(time_estimates[static_cast<unsigned char>(PrintFeatureType::SupportInterface)]);
    message.set_time_prime_tower(time_estimates[static_cast<unsigned char>(PrintFeatureType::PrimeTower)]);
    message.set_time_travel(
        time_estimates[static_cast<unsigned char>(PrintFeatureType::MoveUnretracted)] + time_estimates[static_cast<unsigned char>(PrintFeatureType::MoveRetracted)]
        + time_estimates[static_cast<unsigned char>(PrintFeatureType::MoveWhileRetracting)] + time_estimates[static_cast<unsigned char>(PrintFeatureType::MoveWhileUnretracting)]);

    for (size_t extruder_nr = 0; extruder_nr < Application::getInstance().current_slice_->scene.extruders.size(); extruder_nr++)
    {
        proto::MaterialEstimates* material_message = message.add_materialestimates();
        material_message->set_id(extruder_nr);

        const std::optional<ExtruderPrintInformation>& extruder_info = print_information.extruders_info[extruder_nr];
        if (extruder_info.has_value())
        {
            material_message->set_material_amount(extruder_info->filament_amount);
            material_message->set_material_length(extruder_info->filament_length);
            material_message->set_material_weight(extruder_info->filament_weight);
            material_message->set_material_cost(extruder_info->filament_cost);
            material_message->set_material_name(extruder_info->material_name);
        }
        else
        {
            material_message->set_material_amount(std::numeric_limits<float>::signaling_NaN());
            // NOTE: Set to signalling-NaN to specify that no calculations should be done on the result, and if attempted, should fail fast instead of propagate.
            //       Instead, it's meant to repressent the state of a completely unused extruder or tool, since a value of 0.0 could be a rounding error.
            //       (Also future-proof: '0.0' might conceivably signal a state in which a tool is used that doesn't extrude material, 'NaN' still means unused.)
            material_message->set_material_length(0.0);
            material_message->set_material_weight(0.0);
            material_message->set_material_cost(0.0);
            material_message->set_material_name("");
        }
    }

    grpc_transport_->sendMessage(std::move(message));

    proto::InitialExtruder initial_extruder_message;
    initial_extruder_message.set_extruder_nr(print_information.initial_extruder_nr.value_or(0));
    grpc_transport_->sendMessage(std::move(initial_extruder_message));

    spdlog::debug("Done sending print information.");
}

void GrpcCommunication::sendProgress(double progress) const
{
    const int rounded_amount = 1000 * progress;
    if (last_sent_progress == rounded_amount) // No need to send another tiny update step.
    {
        return;
    }

    const size_t mesh_groups_count = Application::getInstance().current_slice_->scene.mesh_groups.size();
    proto::Progress message;
    double progress_all_objects = progress / mesh_groups_count;
    progress_all_objects += path_compiler->getOptimizedLayers().sliced_objects * (1.0 / mesh_groups_count);
    message.set_amount(progress_all_objects);
    grpc_transport_->sendMessage(std::move(message));

    const_cast<GrpcCommunication*>(this)->last_sent_progress = rounded_amount;
}

void GrpcCommunication::setLayerForSend(const LayerIndex::value_type& layer_nr)
{
    path_compiler->setLayer(layer_nr);
}

void GrpcCommunication::setExtruderForSend(const ExtruderTrain& extruder)
{
    path_compiler->setExtruder(extruder);
}

void GrpcCommunication::sliceNext()
{
    if (! current_slice_)
    {
        spdlog::error("Cannot slice, no current slice available.");
        return;
    }

    if (current_slice_->scene.mesh_groups.empty())
    {
        spdlog::error("Cannot slice, scene has no mesh groups.");
        return;
    }

    slots::instance().broadcast<plugins::v0::SlotID::SETTINGS_BROADCAST>(current_slice_->scene);
    Application::getInstance().current_slice_ = current_slice_;

    spdlog::info("Start actual slicing.");

    current_slice_->compute();
    sendFinishedSlicing();
    current_slice_.reset();
}


void GrpcCommunication::readGlobalSettingsMessage(const proto::SettingList& global_settings_message, Settings& settings)
{
    for (const proto::Setting& setting_message : global_settings_message.settings())
    {
        settings.add(setting_message.name(), setting_message.value());
    }
}

void GrpcCommunication::readExtruderSettingsMessage(const google::protobuf::RepeatedPtrField<proto::Extruder>& extruder_messages, Scene& scene)
{
    // Make sure we have enough extruders added currently.
    const size_t extruder_count = scene.settings.get<size_t>("machine_extruder_count");
    for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        scene.extruders.emplace_back(extruder_nr, &scene.settings);
    }

    // Parse the extruder number and the settings from the messages.
    for (const proto::Extruder& extruder_message : extruder_messages)
    {
        const int32_t extruder_nr = extruder_message.id(); // Cast from proto::int to int32_t!
        if (extruder_nr < 0 || extruder_nr >= static_cast<int32_t>(extruder_count))
        {
            spdlog::warn("Received extruder index that is out of range: {}", extruder_nr);
            continue;
        }
        ExtruderTrain& extruder
            = scene.extruders[extruder_nr]; // Extruder messages may arrive out of order, so don't iteratively get the next extruder but take the extruder_nr from this message.
        for (const proto::Setting& setting_message : extruder_message.settings().settings())
        {
            extruder.settings_.add(setting_message.name(), setting_message.value());
        }
    }
}

void GrpcCommunication::readMeshGroupMessage(const proto::MeshGroup& mesh_group_message, Scene& scene)
{
    auto mesh_group = std::make_shared<MeshGroup>();

    // Load the settings in the mesh group.
    for (const proto::Setting& setting : mesh_group_message.settings())
    {
        mesh_group->settings.add(setting.name(), setting.value());
    }

    scene.appendMeshGroup(mesh_group);
}

void GrpcCommunication::readObjectMessage(const proto::Object& object_message, const std::vector<ExtruderTrain>& extruders, MeshGroup& mesh_group)
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

    Mesh& mesh = mesh_group.meshes.emplace_back();

    // Load the settings for the mesh.
    for (const proto::Setting& setting : object_message.settings())
    {
        mesh.settings_.add(setting.name(), setting.value());
    }

    // Set the parent setting to the correct extruder.
    const auto extruder_nr = mesh.settings_.get<size_t>("extruder_nr");
    const ExtruderTrain& extruder = extruders[extruder_nr];
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

    MeshUtils::loadTextureFromString(object_message.texture(), mesh);

    mesh.mesh_name_ = object_message.name();
    mesh.finish();
}

} // namespace cura
