/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#ifndef INCLUDED_MASON_VOLUME_STORE_HPP
#define INCLUDED_MASON_VOLUME_STORE_HPP

#include "../mesh.h"
#include "../MeshGroup.h"
#include "../utils/polygon.h"

namespace cura {
namespace mason {

class VolumeStoreLayer {
public:
    void addPolygons(const Polygons &polygons);

    const Polygons &getPolygons() const;

private:
    Polygons m_polygons;
};

/** \brief Volumetric representation of print problem.
 */
class VolumeStore {
public:
    struct LayerBounds {
        int64_t z_min;
        int64_t z_max;
    };
    
    VolumeStore();

    void addMeshGroup(const MeshGroup *mesh_group);

    size_t getNumLayers() const;
    const VolumeStoreLayer &getLayer(size_t layer_idx) const;
    LayerBounds getLayerBounds(size_t layer_idx) const;

private:
    void addMesh(const Mesh *mesh);

    std::vector<VolumeStoreLayer> m_layers;
    // m_layers[i] covers [m_layer_zs[i], m_layer_zs[i+1])
    std::vector<int64_t> m_layer_zs;
};

}
}

#endif
