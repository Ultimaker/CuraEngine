/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#include "SkirtPlanner.hpp"

#include "Time.hpp"

namespace mason {

SkirtPlanner::SkirtPlanner() :
    m_name("SkirtPlanner")
{
}

SkirtPlanner::~SkirtPlanner()
{
}

const std::string &SkirtPlanner::getName() const
{
    return m_name;
}
    
void SkirtPlanner::process(BuildPlan *build_plan)
{
    Polygons skirt_polys;

    TimeKeeper part_timer;
    TimeKeeper layer_timer;
    size_t num_layers = build_plan->target->getNumLayers();
    for (size_t layer_idx=0U; layer_idx!=num_layers; ++layer_idx) {
        const VolumeStoreLayer &layer = build_plan->target->getLayer(layer_idx);
        const Polygons &layer_polys = layer.getPolygons();

        skirt_polys = skirt_polys.unionPolygons(layer_polys);
        //std::cout << "layer " << layer_idx << " took " << layer_timer.restart() << std::endl;
        // FIXME: I don't like their simplify, can break topology.  Fix or replace.
        //   It is in for now as it makes the processing significantly faster.
        skirt_polys.simplify();
        //size_t total_points = 0U;
        //for (size_t poly_idx=0U; poly_idx!=skirt_polys.size(); ++poly_idx) {
        //    total_points += skirt_polys[poly_idx].size();
        //}
        //std::cout << "layer " << layer_idx << " total size " << total_points << std::endl;
    }
    std::cout << "skirt: union total time " << part_timer.restart() << std::endl;
    
    writePolygonsToBuildPlan(skirt_polys, build_plan);
    std::cout << "write to build plan time " << part_timer.restart() << std::endl;
}

void SkirtPlanner::writePolygonsToBuildPlan(const Polygons &polygons, BuildPlan *build_plan)
{    
    coord_t top_z = mmToInt(0.25);
    coord_t bot_z = mmToInt(0.0);
    coord_t height = top_z - bot_z;

    Wire wire;
    size_t num_polygons = polygons.size();
    for (size_t polygon_idx=0U; polygon_idx!=num_polygons; ++polygon_idx) {
        const PolygonRef polygon = polygons[polygon_idx];
        size_t num_points = polygon.size();
        assert(num_points > 0U);
        const Point *prev_point = &polygon[num_points-1];
        for (size_t point_idx=0U; point_idx!=num_points; ++point_idx) {
            const Point *point = &polygon[point_idx];

            Point3 start_pt(prev_point->X,prev_point->Y,top_z);
            Point3 end_pt  (point     ->X,point     ->Y,top_z);
            wire.pt0 = start_pt;
            wire.pt1 = end_pt;
            wire.width = mmToInt(0.5);
            wire.height = height;
            build_plan->wire_plan.addWire(wire);

            prev_point = point;
        }
    }
}

}
