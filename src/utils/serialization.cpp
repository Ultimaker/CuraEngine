#include "serialization.h"
#include "../sliceDataStorage.h"
#include "../slicer.h"
#include "../Application.h"
#include "../Slice.h"
#include "../Scene.h"
#include "../MeshGroup.h"
#include "../settings/Settings.h"
#include "../settings/EnumSettings.h"
#include "../raft.h"
#include "clipper/clipper.hpp"
#include "polygon.h"

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

ClipperLib::Path transformPath(const geometry::proto::Path& path)
{
    assert(!(path.points_size() % 2));
    ClipperLib::Path parsed_path;
    parsed_path.reserve(path.points_size() / 2);

    auto it = path.points().begin();
    auto end = path.points().end();
    for(;it != end; it += 2)
        parsed_path.emplace_back(*it, *(it + 1));

    return parsed_path;
}

}

namespace cura
{

namespace serialization_utils
{

bool parseSlicesFromStream(std::istream& stream, MeshGroup& meshgroup, std::vector<Mesh>& meshes, Settings& object_parent_settings, bool support_slices/* = false*/)
{
    geometry::proto::Slices slice;
    if (!slice.ParseFromIstream(&stream))
        return false;

    size_t layers_count = static_cast<size_t>(slice.layers().size());
    const size_t raft_layers = support_slices ? getRaftLayerCount() : 0;
    if (layers_count <= raft_layers)
        return false;

    // Skip raft layers, those will be generated afterwards
    layers_count -= raft_layers;

    for (size_t layer_nr = 0; layer_nr < layers_count; ++layer_nr)
    {
        const geometry::proto::Layer& layer = slice.layers(layer_nr + raft_layers);
        for (auto& path : layer.paths())
        {
            const std::string& mesh_name = path.meshname();
            geometry::proto::Path_Type type = path.type();
            ClipperLib::Path parsed_path = transformPath(path);

            if (!support_slices)
            {
                auto mesh_it = std::find_if(meshes.begin(), meshes.end(), [&mesh_name](const Mesh& mesh) {
                    return mesh.mesh_name == mesh_name;
                });

                if (mesh_it == meshes.end())
                {
                    Mesh mesh(object_parent_settings);
                    mesh.mesh_name = mesh_name;
                    mesh.layers.resize(layers_count);
                    meshes.push_back(std::move(mesh));
                    mesh_it = --meshes.end();
                }

                mesh_it->layers[layer_nr].z = slice.zvalues(layer_nr);
                mesh_it->addPath(parsed_path, type, layer_nr);
            }
            else
            {
                for (auto& mesh : meshgroup.meshes)
                {
                    if (mesh.mesh_name == mesh_name)
                    {
                        mesh.addPath(parsed_path, type, layer_nr);
                    }
                }
            }
        }
    }

    return true;
}

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
            serializePolygons(layer, mesh.support_roofs[i], mesh.mesh_name, geometry::proto::Path_Type_SUPPORT_ROOF);
            serializePolygons(layer, mesh.support_bottoms[i], mesh.mesh_name, geometry::proto::Path_Type_SUPPORT_BOTTOM);
        }
        slices.add_zvalues(storage.meshes.at(0).layers[i].printZ);
    }
    return slices;
}

}//namespace serialization_utils

}//cura
