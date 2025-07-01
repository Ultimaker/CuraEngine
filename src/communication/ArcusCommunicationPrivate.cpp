// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifdef ARCUS

#include "communication/ArcusCommunicationPrivate.h"

#include <fstream>
#include <png.h>
#include <rapidjson/document.h>
#include <rapidjson/error/en.h>
#include <rapidjson/memorystream.h>

#include <spdlog/spdlog.h>

#include "Application.h"
#include "ExtruderTrain.h"
#include "Slice.h"
#include "settings/types/LayerIndex.h"
#include "utils/Matrix4x3D.h" //To convert vertices to integer-points.
#include "utils/Point3F.h" //To accept vertices (which are provided in floating point).

namespace cura
{

ArcusCommunication::Private::Private()
    : socket(nullptr)
    , object_count(0)
    , last_sent_progress(-1)
    , slice_count(0)
    , millisecUntilNextTry(100)
{
}

std::shared_ptr<proto::LayerOptimized> ArcusCommunication::Private::getOptimizedLayerById(LayerIndex::value_type layer_nr)
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

void ArcusCommunication::Private::readGlobalSettingsMessage(const proto::SettingList& global_settings_message)
{
    auto slice = Application::getInstance().current_slice_;
    for (const cura::proto::Setting& setting_message : global_settings_message.settings())
    {
        slice->scene.settings.add(setting_message.name(), setting_message.value());
    }
}

void ArcusCommunication::Private::readExtruderSettingsMessage(const google::protobuf::RepeatedPtrField<proto::Extruder>& extruder_messages)
{
    // Make sure we have enough extruders added currently.
    auto slice = Application::getInstance().current_slice_;
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

void ArcusCommunication::Private::readMeshGroupMessage(const proto::ObjectList& mesh_group_message)
{
    if (mesh_group_message.objects_size() <= 0)
    {
        return; // Don't slice empty mesh groups.
    }

    Scene& scene = Application::getInstance().current_slice_->scene;
    MeshGroup& mesh_group = scene.mesh_groups.at(object_count);

    // Load the settings in the mesh group.
    for (const cura::proto::Setting& setting : mesh_group_message.settings())
    {
        mesh_group.settings.add(setting.name(), setting.value());
    }

    Matrix4x3D matrix;
    for (const cura::proto::Object& object : mesh_group_message.objects())
    {
        constexpr size_t bytes_per_face = sizeof(Point3F) * 3; // 3 vectors per face.
        constexpr size_t bytes_per_uv = sizeof(Point2F) * 3; // 3 vectors per face.
        const size_t face_count = object.vertices().size() / bytes_per_face;

        if (face_count <= 0)
        {
            spdlog::warn("Got an empty mesh. Ignoring it!");
            continue;
        }

        mesh_group.meshes.emplace_back();
        Mesh& mesh = mesh_group.meshes.back();

        // Load the settings for the mesh.
        for (const cura::proto::Setting& setting : object.settings())
        {
            mesh.settings_.add(setting.name(), setting.value());
        }
        ExtruderTrain& extruder = mesh.settings_.get<ExtruderTrain&>("extruder_nr"); // Set the parent setting to the correct extruder.
        mesh.settings_.setParent(&extruder.settings_);

        for (size_t face = 0; face < face_count; face++)
        {
            const std::string data = object.vertices().substr(face * bytes_per_face, bytes_per_face);
            const Point3F* float_vertices = reinterpret_cast<const Point3F*>(data.data());
            const std::string uv_coordinates_str = object.uv_coordinates().empty() ? "" : object.uv_coordinates().substr(face * bytes_per_uv, bytes_per_uv);
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

        loadTextureData(object.texture(), mesh);

        mesh.mesh_name_ = object.name();
        mesh.finish();
    }
    object_count++;
    mesh_group.finalize();
}

void ArcusCommunication::Private::loadTextureData(const std::string& texture_str, Mesh& mesh)
{
    if (texture_str.empty())
    {
        return;
    }

    auto texture_data = reinterpret_cast<const unsigned char*>(texture_str.data());
    const size_t texture_size = texture_str.size();
    png_image raw_texture = {};
    raw_texture.version = PNG_IMAGE_VERSION;
    if (! png_image_begin_read_from_memory(&raw_texture, texture_data, texture_size))
    {
        spdlog::error("Error when beginning reading mesh texture: {}", raw_texture.message);
        return;
    }

    std::vector<uint8_t> buffer(PNG_IMAGE_SIZE(raw_texture));
    if (! png_image_finish_read(&raw_texture, nullptr, buffer.data(), 0, nullptr) || buffer.empty())
    {
        spdlog::error("Error when finishing reading mesh texture: {}", raw_texture.message);
        return;
    }

    // Make sure pointer will be destroyed when leaving
    std::unique_ptr<png_struct, void (*)(png_structp)> png_ptr(
        png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr),
        [](png_structp png_ptr_destroy)
        {
            png_destroy_read_struct(&png_ptr_destroy, nullptr, nullptr);
        });
    if (! png_ptr)
    {
        return;
    }

    // Make sure pointer will be destroyed when leaving
    std::unique_ptr<png_info, void (*)(png_infop)> info_ptr(
        png_create_info_struct(png_ptr.get()),
        [](png_infop info_ptr_destroy)
        {
            png_destroy_read_struct(nullptr, &info_ptr_destroy, nullptr);
        });
    if (! info_ptr)
    {
        return;
    }

    if (setjmp(png_jmpbuf(png_ptr.get())) != 0)
    {
        return;
    }

    struct PngReadContext
    {
        const unsigned char* data;
        size_t size;
        size_t offset;
    } read_context{ texture_data, texture_size, 0 };

    png_set_read_fn(
        png_ptr.get(),
        &read_context,
        [](const png_structp read_png_ptr, const png_bytep out_bytes, const png_size_t byte_count_to_read)
        {
            auto* context = static_cast<PngReadContext*>(png_get_io_ptr(read_png_ptr));
            if (context->offset + byte_count_to_read > context->size)
            {
                png_error(read_png_ptr, "Read beyond end of buffer");
            }
            memcpy(out_bytes, context->data + context->offset, byte_count_to_read);
            context->offset += byte_count_to_read;
        });
    png_read_info(png_ptr.get(), info_ptr.get());

    png_textp text_ptr;
    int num_text;
    if (png_get_text(png_ptr.get(), info_ptr.get(), &text_ptr, &num_text) <= 0)
    {
        return;
    }

    auto texture_data_mapping = std::make_shared<TextureDataMapping>();
    for (int i = 0; i < num_text; ++i)
    {
        if (std::string(text_ptr[i].key) == "Description")
        {
            rapidjson::MemoryStream json_memory_stream(text_ptr[i].text, text_ptr[i].text_length);

            rapidjson::Document json_document;
            json_document.ParseStream(json_memory_stream);
            if (json_document.HasParseError())
            {
                spdlog::error("Error parsing texture data mapping (offset {}): {}", json_document.GetErrorOffset(), GetParseError_En(json_document.GetParseError()));
                return;
            }

            for (auto it = json_document.MemberBegin(); it != json_document.MemberEnd(); ++it)
            {
                std::string feature_name = it->name.GetString();

                const rapidjson::Value& array = it->value;
                if (array.IsArray() && array.Size() == 2)
                {
                    (*texture_data_mapping)[feature_name] = TextureBitField{ array[0].GetUint(), array[1].GetUint() };
                }
            }

            break;
        }
    }

    if (! texture_data_mapping->empty())
    {
        mesh.texture_ = std::make_shared<Image>(
            raw_texture.width,
            raw_texture.height,
            PNG_IMAGE_SAMPLE_COMPONENT_SIZE(raw_texture.format) * PNG_IMAGE_SAMPLE_CHANNELS(raw_texture.format),
            std::move(buffer));
        mesh.texture_data_mapping_ = texture_data_mapping;
    }
}

} // namespace cura

#endif // ARCUS
