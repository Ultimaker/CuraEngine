/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#include "VolumeStore.hpp"

#include "GuiSocket.hpp"
#include "Time.hpp"

namespace mason {

void VolumeStoreLayer::addPolygons(const Polygons &polygons)
{
    m_polygons.add(polygons);
}

const Polygons &
VolumeStoreLayer::getPolygons() const
{
    return m_polygons;
}

VolumeStore::VolumeStore()
{
}

void VolumeStore::addMeshGroup(const MeshGroup *mesh_group)
{
    size_t num_meshes = mesh_group->meshes.size();
    for (size_t mesh_idx=0U; mesh_idx!=num_meshes; ++mesh_idx) {
        addMesh(&mesh_group->meshes[mesh_idx]);
    }

    sendToGui();
}

void VolumeStore::addMesh(const Mesh *mesh)
{
    // TODO: Make slicing thickness configurable.
    static const int layer_thickness = 250;
    static const int initial_slice_z = layer_thickness/2;
    Point3 mesh_max = fromCuraPoint3(mesh->max());
    size_t layer_count = mesh_max.z / layer_thickness + 1;
    bool keep_open_polygons = mesh->getSettingBoolean("meshfix_keep_open_polygons");
    bool extensive_stitching = mesh->getSettingBoolean("meshfix_extensive_stitching");
    TimeKeeper part_timer;
    Slicer slicer(mesh, initial_slice_z, layer_thickness, layer_count,
                  keep_open_polygons, extensive_stitching);
    std::cout << "slicing time " << part_timer.restart() << std::endl;

    if (m_layers.size() < layer_count) {
        m_layers.resize(layer_count);
    }
    if (m_layer_zs.size() < layer_count+1) {
        size_t old_size = m_layer_zs.size();
        size_t new_size = layer_count+1;
        m_layer_zs.resize(new_size);
        for (size_t layer_idx=old_size; layer_idx!=new_size; ++layer_idx) {
            m_layer_zs[layer_idx] = layer_thickness * layer_idx;
        }
    }

    for (size_t layer_idx=0; layer_idx!=layer_count; ++layer_idx) {
        m_layers[layer_idx].addPolygons(slicer.layers[layer_idx].polygons);
    }
}

void VolumeStore::sendToGui() const
{
    static const coord_t line_width = mmToInt(0.1);

    GuiSocket *socket = NULL;
    if (GuiSocket::isInstantiated()) {
        socket = GuiSocket::getInstance();
    } else {
        return;
    }
    
    size_t num_layers = m_layers.size();
    for (size_t layer_idx=0U; layer_idx!=num_layers; ++layer_idx) {
        const VolumeStoreLayer &layer = m_layers[layer_idx];
        coord_t z_low = m_layer_zs[layer_idx];
        coord_t z_high = m_layer_zs[layer_idx+1];
        coord_t height = z_high - z_low;

        socket->sendLayerInfo(layer_idx, z_high, height);

        socket->sendPolygons(PrintFeatureType::OuterWall, layer_idx, layer.getPolygons(), line_width);
    }
}

size_t VolumeStore::getNumLayers() const
{
    return m_layers.size();
}

const VolumeStoreLayer &VolumeStore::getLayer(size_t layer_idx) const
{
    assert(layer_idx < m_layers.size());
    
    return m_layers[layer_idx];
}

VolumeStore::LayerBounds VolumeStore::getLayerBounds(size_t layer_idx) const
{
    assert(layer_idx+1 < m_layer_zs.size());

    LayerBounds bounds{m_layer_zs[layer_idx], m_layer_zs[layer_idx+1]};
    return bounds;
}

size_t VolumeStore::getLayerIdx(coord_t z) const
{
    std::vector<coord_t>::const_iterator iter = std::lower_bound(m_layer_zs.begin(),m_layer_zs.end(),z);
    assert(iter != m_layer_zs.end());

    return iter - m_layer_zs.begin();
}

}
