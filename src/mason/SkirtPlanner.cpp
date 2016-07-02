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
    static const coord_t bloat_offset = mmToInt(100.0);
    static const coord_t bloat_offset2 = -mmToInt(97.0);
    
    Polygons skirt_polys;

    TimeKeeper part_timer;
    TimeKeeper layer_timer;
    size_t num_layers = build_plan->target->getNumLayers();
    size_t max_avoid_layer = std::min(num_layers,(size_t)2U);
    for (size_t layer_idx=0U; layer_idx!=max_avoid_layer; ++layer_idx) {
        const VolumeStoreLayer &layer = build_plan->target->getLayer(layer_idx);
        const Polygons &layer_polys = layer.getPolygons();

        skirt_polys = skirt_polys.unionPolygons(layer_polys);
    }
    std::cout << "skirt: union total time " << part_timer.restart() << std::endl;
    
    coord_t top_z = mmToInt(0.25);
    coord_t bot_z = mmToInt(0.0);
    coord_t height = top_z - bot_z;

    skirt_polys = skirt_polys.offset(bloat_offset, ClipperLib::jtRound).offset(bloat_offset2, ClipperLib::jtRound);
    size_t total_points = 0U;
    for (size_t poly_idx=0U; poly_idx!=skirt_polys.size(); ++poly_idx) {
        total_points += skirt_polys[poly_idx].size();
    }
    std::cout << "skirt total size " << total_points << std::endl;
    writePolygonsToBuildPlan(skirt_polys, top_z, height, build_plan);
    std::cout << "write to build plan time " << part_timer.restart() << std::endl;
}

void SkirtPlanner::writePolygonsToBuildPlan(const Polygons &polygons,
                                            coord_t top_z, coord_t height,
                                            BuildPlan *build_plan)
{    
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
