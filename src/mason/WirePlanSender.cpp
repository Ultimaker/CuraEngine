/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#include "WirePlanSender.hpp"

#include <map>

namespace mason {

void WirePlanSender::process(BuildPlan *build_plan)
{
    // We execute even if socket isn't connected for debugging purposes.
    GuiSocket *socket = NULL;
    
    if (GuiSocket::isInstantiated()) {
        socket = GuiSocket::getInstance();
    }

    const WirePlan &wire_plan = build_plan->wire_plan;

    m_gui_layer_idx = 0;
    size_t num_layers = wire_plan.numLayers();
    for (size_t layer_idx=0U; layer_idx!=num_layers; ++layer_idx) {
        sendLayer(socket, wire_plan.getLayer(layer_idx));
    }
}

void WirePlanSender::sendLayer(GuiSocket *socket, const WireLayer &wire_layer)
{
    size_t num_wires_layer = wire_layer.wires.size();

    // map from (height,width) -> list of indexes into wire_layer.wires
    using WireParams = std::pair<coord_t, coord_t>;
    using CompatWiresMap = std::map<WireParams, std::vector<size_t> >;
    CompatWiresMap compat_wires_map;
    for (size_t wire_idx=0U; wire_idx!=num_wires_layer; ++wire_idx) {
        const Wire &wire = wire_layer.wires[wire_idx];
        WireParams params(wire.height, wire.width);
        compat_wires_map[params].push_back(wire_idx);
    }

    for (CompatWiresMap::const_iterator compat_iter=compat_wires_map.begin();
         compat_iter!=compat_wires_map.end();
         ++compat_iter) {
        const WireParams &params = compat_iter->first;
        const std::vector<size_t> &wire_idxs = compat_iter->second;
        coord_t height = params.first;
        coord_t width = params.second;

        if (socket) {
            socket->sendLayerInfo(m_gui_layer_idx, wire_layer.z, height);
        }

        Polygons wire_polys;
        for (size_t wire_idx : wire_idxs) {
            const Wire &wire = wire_layer.wires[wire_idx];
            
            PolygonRef wire_poly = wire_polys.newPoly();
            wire_poly.emplace_back(wire.pt0.x, wire.pt0.y);
            wire_poly.emplace_back(wire.pt1.x, wire.pt1.y);
        }

        if (socket) {
            socket->sendPolygons(PrintFeatureType::OuterWall, m_gui_layer_idx, wire_polys, width);
        }

        ++m_gui_layer_idx;
    }
}

}
