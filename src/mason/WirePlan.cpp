/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#include "WirePlan.hpp"

namespace cura {
namespace mason {

void WirePlan::addWire(const Wire &wire)
{
    int_coord_t z = std::max(wire.pt0.z, wire.pt1.z);

    int_coord_t layer_height = MM2INT(0.25);
    size_t layer_idx = (z-1) / layer_height;
    size_t num_layers = m_layers.size();
    if (m_layers.size() <= layer_idx) {
        size_t new_num_Layers = layer_idx + 1;
        m_layers.resize(new_num_Layers);
        for (size_t new_layer_idx = num_layers; new_layer_idx!=new_num_Layers; ++new_layer_idx) {
            m_layers[new_layer_idx].z = layer_height * (new_layer_idx+1);
        }
    }

    m_layers[layer_idx].wires.push_back(wire);
}

size_t WirePlan::numLayers() const
{
    return m_layers.size();
}

const WireLayer &WirePlan::getLayer(size_t layer_idx) const
{
    return m_layers[layer_idx];
}

}
}
