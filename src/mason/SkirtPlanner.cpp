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
    coord_t bloat_offset =
        build_plan->settings->getSettingInMicrons("skirt_gap");

    coord_t skirt_top_z =
        build_plan->settings->getSettingInMicrons("layer_height_0");
    coord_t skirt_bot_z = mmToInt(0.0);
    coord_t skirt_height = skirt_top_z - skirt_bot_z;

    coord_t norm_layer_height =
        build_plan->settings->getSettingInMicrons("layer_height");
    coord_t avoid_top_z = skirt_top_z + norm_layer_height;
    
    Polygons skirt_polys;
    //TimeKeeper part_timer;

    // Union bottom layers to find areas to avoid
    size_t num_layers = build_plan->target->getNumLayers();
    for (size_t layer_idx=0U; layer_idx!=num_layers; ++layer_idx) {
        const VolumeStoreLayer &layer = build_plan->target->getLayer(layer_idx);
        const Polygons &layer_polys = layer.getPolygons();
        VolumeStore::LayerBounds layer_bounds =
            build_plan->target->getLayerBounds(layer_idx);

        if (layer_bounds.z_min >= avoid_top_z) {
            break;
        }

        skirt_polys = skirt_polys.unionPolygons(layer_polys);
    }
    //std::cout << "skirt: union total time " << part_timer.restart() << std::endl;

    // Take convex hull to find perimeter outside all obstacles
    Polygon hull = skirt_polys.convexHullMason();
    skirt_polys.clear();
    skirt_polys.add(std::move(hull));
    //std::cout << "skirt: hull time " << part_timer.restart() << std::endl;

    // Add some space between obstacles and skirt
    skirt_polys = skirt_polys.offset(bloat_offset, ClipperLib::jtRound);
    //std::cout << "skirt: offset time " << part_timer.restart() << std::endl;

    // Calculate / print size for debugging
    //size_t total_points = 0U;
    //for (size_t poly_idx=0U; poly_idx!=skirt_polys.size(); ++poly_idx) {
    //    total_points += skirt_polys[poly_idx].size();
    //}
    //std::cout << "skirt total size " << total_points << std::endl;
    //std::cout << "skirt: total size time " << part_timer.restart() << std::endl;

    // Write result into wire plan
    writePolygonsToBuildPlan(skirt_polys, skirt_top_z, skirt_height, build_plan);
    //std::cout << "write to build plan time " << part_timer.restart() << std::endl;
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
