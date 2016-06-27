/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#include "BuildPlanner.hpp"

namespace mason {

void BuildPlanner::createWirePlan()
{
    coord_t top_z = mmToInt(0.25);
    coord_t bot_z = mmToInt(0.0);
    coord_t mid_z = (top_z + bot_z)/2;
    coord_t height = top_z - bot_z;

    size_t layer_idx = m_build_plan->target->getLayerIdx(mid_z);
    const VolumeStoreLayer &layer = m_build_plan->target->getLayer(layer_idx);
    const Polygons &polygons = layer.getPolygons();

    Wire wire;
    size_t num_polygons = polygons.size();
    for (size_t polygon_idx=0U; polygon_idx!=num_polygons; ++polygon_idx) {
        const PolygonRef polygon = polygons[polygon_idx];
        size_t num_points = polygon.size();
        assert(num_points > 0U);
        const Point *prev_point = &polygon[0];
        for (size_t point_idx=1U; point_idx<num_points; ++point_idx) {
            const Point *point = &polygon[point_idx];

            Point3 start_pt(prev_point->X,prev_point->Y,top_z);
            Point3 end_pt  (point     ->X,point     ->Y,top_z);
            wire.pt0 = start_pt;
            wire.pt1 = end_pt;
            wire.width = mmToInt(0.5);
            wire.height = height;
            m_build_plan->wire_plan.addWire(wire);

            prev_point = point;
        }
    }
}

void BuildPlanner::createPrintPlan()
{
    std::shared_ptr<PrintPlanElement> elem;

    size_t num_layers = m_build_plan->wire_plan.numLayers();
    for (size_t layer_idx=0U; layer_idx!=num_layers; ++layer_idx) {
        const WireLayer &layer = m_build_plan->wire_plan.getLayer(layer_idx);

        size_t num_wires = layer.wires.size();
        for (size_t wire_idx=0U; wire_idx!=num_wires; ++wire_idx) {
            const Wire &wire = layer.wires[wire_idx];
            
            elem.reset(new HeadMove(wire.pt0,30.0));
            m_build_plan->print_plan.addElement(elem);
            elem.reset(new ExtrudedSegment(wire.pt0,wire.pt1,30.0,intToMm(wire.width)*intToMm(wire.height)));
            m_build_plan->print_plan.addElement(elem);
        }
    }
}

void BuildPlanner::process(BuildPlan *build_plan)
{
    m_build_plan = build_plan;
    
    createWirePlan();
    createPrintPlan();    
}

}

