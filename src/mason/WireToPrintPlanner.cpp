/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#include "WireToPrintPlanner.hpp"

namespace mason {

WireToPrintPlanner::~WireToPrintPlanner()
{
}

void WireToPrintPlanner::process(BuildPlan *build_plan)
{
    std::shared_ptr<PrintPlanElement> elem;

    size_t num_layers = build_plan->wire_plan.numLayers();
    for (size_t layer_idx=0U; layer_idx!=num_layers; ++layer_idx) {
        const WireLayer &layer = build_plan->wire_plan.getLayer(layer_idx);

        size_t num_wires = layer.wires.size();
        for (size_t wire_idx=0U; wire_idx!=num_wires; ++wire_idx) {
            const Wire &wire = layer.wires[wire_idx];
            
            elem.reset(new HeadMove(wire.pt0,30.0));
            build_plan->print_plan.addElement(elem);
            elem.reset(new ExtrudedSegment(wire.pt0,wire.pt1,30.0,intToMm(wire.width)*intToMm(wire.height)));
            build_plan->print_plan.addElement(elem);
        }
    }
}

}
