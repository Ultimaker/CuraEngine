#include "serialization.h"
#include "../sliceDataStorage.h"
#include "../slicer.h"
#include "../Application.h"
#include "../Slice.h"
#include "../Scene.h"
#include "../MeshGroup.h"
#include "../settings/EnumSettings.h"
#include "../raft.h"

namespace
{

size_t getRaftLayerCount()
{
    size_t raft_layers = 0;
    const cura::Settings& mesh_group_settings = cura::Application::getInstance().current_slice->scene.current_mesh_group->settings;
    if (mesh_group_settings.get<cura::EPlatformAdhesion>("adhesion_type") == cura::EPlatformAdhesion::RAFT)
    {
        ++raft_layers;
    }
    return raft_layers;
}

void serializePolygons(geometry::proto::Layer* layer, const cura::Polygons& polygons, const std::string& mesh_name, geometry::proto::Path_Type type)
{
    // Some parts could have empty layers. So, add at least one path to the part profile even it's empty.
    if (polygons.empty() && type == geometry::proto::Path_Type::Path_Type_PART)
    {
        auto path = layer->add_paths();
        path->set_meshname(mesh_name);
        path->set_type(type);
    }

    for (const auto& polygon : polygons)
    {
        auto path = layer->add_paths();
        path->set_meshname(mesh_name);
        path->set_type(type);
        path->set_area(ClipperLib::Area(polygon));
        for (const auto& point : polygon)
        {
            path->add_points(point.X);
            path->add_points(point.Y);
        }
    }
}

void serializeRaft(geometry::proto::Slices& slices, const cura::SliceDataStorage& storage)
{
    const cura::coord_t raft_thickness = cura::Raft::getTotalThickness();
    slices.add_zvalues(raft_thickness);
    serializePolygons(slices.add_layers(), storage.raftOutline, "", geometry::proto::Path_Type_RAFT);
}

}

namespace cura
{

namespace serialization_utils
{

geometry::proto::Slices getSlices(const SliceDataStorage& storage)
{
    geometry::proto::Slices slices;
    const size_t layers_count = storage.print_layer_count;
    slices.mutable_layers()->Reserve(layers_count);
    slices.mutable_zvalues()->Resize(layers_count, 0);

    for (size_t i = 0; i < layers_count; ++i)
    {
        auto layer = slices.add_layers();
        for (const auto& mesh : Application::getInstance().current_slice->scene.current_mesh_group->meshes)
        {
            if (i < mesh.layers.size())
            {
                const SlicerLayer& slicer_layer = mesh.layers[i];
                serializePolygons(layer, slicer_layer.polygons, mesh.mesh_name, geometry::proto::Path_Type_PART);
                slices.set_zvalues(static_cast<int>(i), slicer_layer.z);
            }
        }
    }
    return slices;
}

geometry::proto::Slices getSupportSlices(const SliceDataStorage& storage)
{
    geometry::proto::Slices slices;

    const size_t layers_count = storage.print_layer_count + getRaftLayerCount();

    slices.mutable_layers()->Reserve(layers_count);
    slices.mutable_zvalues()->Reserve(layers_count);

    const cura::Settings& mesh_group_settings = cura::Application::getInstance().current_slice->scene.current_mesh_group->settings;
    if (mesh_group_settings.get<cura::EPlatformAdhesion>("adhesion_type") == cura::EPlatformAdhesion::RAFT)
    {
        serializeRaft(slices, storage);
    }

    for (size_t i = 0; i < storage.print_layer_count; ++i)
    {
        auto layer = slices.add_layers();
        for (auto& mesh : Application::getInstance().current_slice->scene.current_mesh_group->meshes)
        {
            serializePolygons(layer, mesh.support[i], mesh.mesh_name, geometry::proto::Path_Type_SUPPORT);
            serializePolygons(layer, mesh.support_roof[i], mesh.mesh_name, geometry::proto::Path_Type_SUPPORT_ROOF);
            serializePolygons(layer, mesh.support_bottom[i], mesh.mesh_name, geometry::proto::Path_Type_SUPPORT_BOTTOM);
        }
        slices.add_zvalues(storage.meshes.at(0).layers[i].printZ);
    }
    return slices;
}

}//namespace serialization_utils

}//cura
