/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#include "SkirtPlanner.hpp"

namespace mason {

SkirtPlanner::~SkirtPlanner()
{
}

void SkirtPlanner::process(BuildPlan *build_plan)
{
    coord_t top_z = mmToInt(0.25);
    coord_t bot_z = mmToInt(0.0);
    coord_t mid_z = (top_z + bot_z)/2;

    size_t layer_idx = build_plan->target->getLayerIdx(mid_z);
    const VolumeStoreLayer &layer = build_plan->target->getLayer(layer_idx);
    const Polygons &polygons = layer.getPolygons();

    writePolygonsToBuildPlan(polygons, build_plan);
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
