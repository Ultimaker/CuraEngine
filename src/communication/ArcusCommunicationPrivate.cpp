//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifdef ARCUS

#include "ArcusCommunicationPrivate.h"
#include "../Application.h"
#include "../ExtruderTrain.h"
#include "../Slice.h"
#include "../settings/types/LayerIndex.h"
#include "../utils/floatpoint.h"
#include "../utils/logoutput.h"

namespace cura
{

ArcusCommunication::Private::Private()
    : socket(nullptr)
    , object_count(0)
    , last_sent_progress(-1)
    , slice_count(0)
    , millisecUntilNextTry(100)
{}

std::shared_ptr<proto::LayerOptimized> ArcusCommunication::Private::getOptimizedLayerById(LayerIndex layer_nr)
{
    layer_nr += optimized_layers.current_layer_offset;
    std::unordered_map<int, std::shared_ptr<proto::LayerOptimized>>::iterator find_result = optimized_layers.slice_data.find(layer_nr);

    if (find_result != optimized_layers.slice_data.end()) //Load layer from the cache.
    {
        return find_result->second;
    }
    else //Not in the cache yet. Create an empty layer.
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
    Slice* slice = Application::getInstance().current_slice;
    for (const cura::proto::Setting& setting_message : global_settings_message.settings())
    {
        slice->scene.settings.add(setting_message.name(), setting_message.value());
    }
}

void ArcusCommunication::Private::readExtruderSettingsMessage(const google::protobuf::RepeatedPtrField<proto::Extruder>& extruder_messages)
{
    //Make sure we have enough extruders added currently.
    Slice* slice = Application::getInstance().current_slice;
    const size_t extruder_count = slice->scene.settings.get<size_t>("machine_extruder_count");
    for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        slice->scene.extruders.emplace_back(extruder_nr, &slice->scene.settings);
    }

    //Parse the extruder number and the settings from the messages.
    for (const cura::proto::Extruder& extruder_message : extruder_messages)
    {
        const int32_t extruder_nr = extruder_message.id(); //Cast from proto::int to int32_t!
        if (extruder_nr < 0 || extruder_nr >= static_cast<int32_t>(extruder_count))
        {
            logWarning("Received extruder index that is out of range: %i\n", extruder_nr);
            continue;
        }
        ExtruderTrain& extruder = slice->scene.extruders[extruder_nr]; //Extruder messages may arrive out of order, so don't iteratively get the next extruder but take the extruder_nr from this message.
        for (const cura::proto::Setting& setting_message : extruder_message.settings().settings())
        {
            extruder.settings.add(setting_message.name(), setting_message.value());
        }
    }
}

void ArcusCommunication::Private::readMeshGroupMessage(const proto::ObjectList& mesh_group_message)
{
    if (mesh_group_message.objects_size() <= 0)
    {
        return; //Don't slice empty mesh groups.
    }

    Scene& scene = Application::getInstance().current_slice->scene;
    MeshGroup& mesh_group = scene.mesh_groups.at(object_count);

    //Load the settings in the mesh group.
    for (const cura::proto::Setting& setting : mesh_group_message.settings())
    {
        mesh_group.settings.add(setting.name(), setting.value());
    }

    FMatrix3x3 matrix;
    for (const cura::proto::Object& object : mesh_group_message.objects())
    {
        const size_t bytes_per_face = sizeof(FPoint3) * 3; //3 vectors per face.
        const size_t face_count = object.vertices().size() / bytes_per_face;

        if (face_count <= 0)
        {
            logWarning("Got an empty mesh. Ignoring it!");
            continue;
        }

        mesh_group.meshes.emplace_back();
        Mesh& mesh = mesh_group.meshes.back();

        //Load the settings for the mesh.
        for (const cura::proto::Setting& setting : object.settings())
        {
            mesh.settings.add(setting.name(), setting.value());
        }
        ExtruderTrain& extruder = mesh.settings.get<ExtruderTrain&>("extruder_nr"); //Set the parent setting to the correct extruder.
        mesh.settings.setParent(&extruder.settings);

        for (size_t face = 0; face < face_count; face++)
        {
            const std::string data = object.vertices().substr(face * bytes_per_face, bytes_per_face);
            const FPoint3* float_vertices = reinterpret_cast<const FPoint3*>(data.data());

            Point3 verts[3];
            verts[0] = matrix.apply(float_vertices[0]);
            verts[1] = matrix.apply(float_vertices[1]);
            verts[2] = matrix.apply(float_vertices[2]);
            mesh.addFace(verts[0], verts[1], verts[2]);
        }

        mesh.mesh_name = object.name();
        mesh.finish();
    }
    object_count++;
    mesh_group.finalize();
}

} //namespace cura

#endif //ARCUS