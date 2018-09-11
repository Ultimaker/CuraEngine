//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <algorithm>
#include <limits>

#include "Application.h" //To get settings.
#include "ExtruderTrain.h"
#include "gcodeExport.h"
#include "infill.h"
#include "LayerPlan.h"
#include "PrimeTower.h"
#include "PrintFeature.h"
#include "raft.h"
#include "sliceDataStorage.h"

#define CIRCLE_RESOLUTION 32 //The number of vertices in each circle.

namespace cura 
{

PrimeTower::PrimeTower()
: wipe_from_middle(false)
{
    const Scene& scene = Application::getInstance().current_slice->scene;
    enabled = scene.current_mesh_group->settings.get<bool>("prime_tower_enable")
           && scene.current_mesh_group->settings.get<coord_t>("prime_tower_min_volume") > 10
           && scene.current_mesh_group->settings.get<coord_t>("prime_tower_size") > 10;

    extruder_count = scene.extruders.size();
    extruder_order.resize(extruder_count);
    for (unsigned int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        extruder_order[extruder_nr] = extruder_nr; //Start with default order, then sort.
    }
    //Sort from high adhesion to low adhesion.
    const Scene* scene_pointer = &scene; //Communicate to lambda via pointer to prevent copy.
    std::sort(extruder_order.begin(), extruder_order.end(), [scene_pointer](const unsigned int& extruder_nr_a, const unsigned int& extruder_nr_b) -> bool
    {
        const Ratio adhesion_a = scene_pointer->extruders[extruder_nr_a].settings.get<Ratio>("material_adhesion_tendency");
        const Ratio adhesion_b = scene_pointer->extruders[extruder_nr_b].settings.get<Ratio>("material_adhesion_tendency");
        return adhesion_a < adhesion_b;
    });
}

void PrimeTower::generateGroundpoly()
{
    if (!enabled)
    {
        return;
    }

    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const coord_t tower_size = mesh_group_settings.get<coord_t>("prime_tower_size");
    const bool circular_prime_tower = mesh_group_settings.get<bool>("prime_tower_circular");

    PolygonRef p = outer_poly.newPoly();
    int tower_distance = 0; 
    const coord_t x = mesh_group_settings.get<coord_t>("prime_tower_position_x");
    const coord_t y = mesh_group_settings.get<coord_t>("prime_tower_position_y");
    if (circular_prime_tower)
    {
        const coord_t tower_radius = tower_size / 2;
        for (unsigned int i = 0; i < CIRCLE_RESOLUTION; i++)
        {
            const double angle = (double) i / CIRCLE_RESOLUTION * 2 * M_PI; //In radians.
            p.add(Point(x - tower_radius + tower_distance + cos(angle) * tower_radius,
                        y + tower_radius + tower_distance + sin(angle) * tower_radius));
        }
    }
    else
    {
        p.add(Point(x + tower_distance, y + tower_distance));
        p.add(Point(x + tower_distance, y + tower_distance + tower_size));
        p.add(Point(x + tower_distance - tower_size, y + tower_distance + tower_size));
        p.add(Point(x + tower_distance - tower_size, y + tower_distance));
    }
    middle = Point(x - tower_size / 2, y + tower_size / 2);

    post_wipe_point = Point(x + tower_distance - tower_size / 2, y + tower_distance + tower_size / 2);
}

void PrimeTower::generatePaths()
{
    if (enabled)
    {
        generatePaths_denseInfill();
    }
}

void PrimeTower::generatePaths_denseInfill()
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const coord_t layer_height = mesh_group_settings.get<coord_t>("layer_height");
    pattern_per_extruder.resize(extruder_count);

    coord_t cumulative_inset = 0; //Each tower shape is going to be printed inside the other. This is the inset we're doing for each extruder.
    const Scene& scene = Application::getInstance().current_slice->scene;
    for (size_t extruder_nr : extruder_order)
    {
        const coord_t line_width = scene.extruders[extruder_nr].settings.get<coord_t>("prime_tower_line_width");
        const coord_t required_volume = scene.extruders[extruder_nr].settings.get<double>("prime_tower_min_volume") * 1000000000; //To cubic microns.
        const Ratio flow = scene.extruders[extruder_nr].settings.get<Ratio>("prime_tower_flow");
        coord_t current_volume = 0;
        ExtrusionMoves& pattern = pattern_per_extruder[extruder_nr];

        //Create the walls of the prime tower.
        unsigned int wall_nr = 0;
        for (; current_volume < required_volume; wall_nr++)
        {
            //Create a new polygon with an offset from the outer polygon.
            Polygons polygons = outer_poly.offset(-cumulative_inset - wall_nr * line_width - line_width / 2);
            pattern.polygons.add(polygons);
            current_volume += polygons.polygonLength() * line_width * layer_height * flow;
            if (polygons.empty()) //Don't continue. We won't ever reach the required volume because it doesn't fit.
            {
                break;
            }
        }
        cumulative_inset += wall_nr * line_width;

        //Generate the pattern for the first layer.
        coord_t line_width_layer0 = line_width;
        if (mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") != EPlatformAdhesion::RAFT)
        {
            line_width_layer0 *= scene.extruders[extruder_nr].settings.get<Ratio>("initial_layer_line_width_factor");
        }
        pattern_per_extruder_layer0.emplace_back();
        ExtrusionMoves& pattern_layer0 = pattern_per_extruder_layer0.back();
        pattern_layer0.polygons = outer_poly.offset(-line_width_layer0 / 2);
        const coord_t outline_offset = -line_width_layer0;
        const coord_t line_distance = line_width_layer0;
        constexpr double fill_angle = 45;
        constexpr bool zig_zaggify_infill = false;
        constexpr coord_t extra_infill_shift = 0;
        constexpr coord_t infill_overlap = 60; // so that it can't be zero; EDIT: wtf?
        constexpr coord_t z = 0; // (TODO) because the prime tower stores the paths for each extruder for once instead of generating each layer, we don't know the z position
        constexpr bool connect_polygons = true;
        constexpr int infill_multiplier = 1;
        Infill infill_comp(EFillMethod::CONCENTRIC, zig_zaggify_infill, connect_polygons, outer_poly, outline_offset, line_width_layer0, line_distance, infill_overlap, infill_multiplier, fill_angle, z, extra_infill_shift);
        infill_comp.generate(pattern_layer0.polygons, pattern_layer0.lines);
    }
}


void PrimeTower::addToGcode(const SliceDataStorage& storage, LayerPlan& gcode_layer, const int prev_extruder, const int new_extruder) const
{
    if (!enabled)
    {
        return;
    }
    if (gcode_layer.getPrimeTowerIsPlanned(new_extruder))
    { // don't print the prime tower if it has been printed already with this extruder.
        return;
    }

    if (gcode_layer.getLayerNr() > storage.max_print_height_second_to_last_extruder + 1)
    {
        return;
    }

    bool post_wipe = Application::getInstance().current_slice->scene.extruders[prev_extruder].settings.get<bool>("prime_tower_wipe_enabled");

    // Do not wipe on the first layer, we will generate non-hollow prime tower there for better bed adhesion.
    const int layer_nr = gcode_layer.getLayerNr();
    if (prev_extruder == new_extruder || layer_nr == 0)
    {
        post_wipe = false;
    }

    addToGcode_denseInfill(gcode_layer, new_extruder);

    // post-wipe:
    if (post_wipe)
    {
        //Make sure we wipe the old extruder on the prime tower.
        const Settings& previous_settings = Application::getInstance().current_slice->scene.extruders[prev_extruder].settings;
        const Point previous_nozzle_offset = Point(previous_settings.get<coord_t>("machine_nozzle_offset_x"), previous_settings.get<coord_t>("machine_nozzle_offset_y"));
        const Settings& new_settings = Application::getInstance().current_slice->scene.extruders[new_extruder].settings;
        const Point new_nozzle_offset = Point(new_settings.get<coord_t>("machine_nozzle_offset_x"), new_settings.get<coord_t>("machine_nozzle_offset_y"));
        gcode_layer.addTravel(post_wipe_point - previous_nozzle_offset + new_nozzle_offset);
    }

    gcode_layer.setPrimeTowerIsPlanned(new_extruder);
}

void PrimeTower::addToGcode_denseInfill(LayerPlan& gcode_layer, const size_t extruder_nr) const
{
    const ExtrusionMoves& pattern = (gcode_layer.getLayerNr() == -static_cast<LayerIndex>(Raft::getFillerLayerCount()))
        ? pattern_per_extruder_layer0[extruder_nr]
        : pattern_per_extruder[extruder_nr];

    const GCodePathConfig& config = gcode_layer.configs_storage.prime_tower_config_per_extruder[extruder_nr];

    gcode_layer.addPolygonsByOptimizer(pattern.polygons, config);
    gcode_layer.addLinesByOptimizer(pattern.lines, config, SpaceFillType::Lines);
}

Point PrimeTower::getLocationBeforePrimeTower(const SliceDataStorage& storage) const
{
    Point ret(0, 0);
    int absolute_starting_points = 0;
    for (size_t extruder_nr = 0; extruder_nr < Application::getInstance().current_slice->scene.extruders.size(); extruder_nr++)
    {
        ExtruderTrain& train = Application::getInstance().current_slice->scene.extruders[0];
        if (train.settings.get<bool>("machine_extruder_start_pos_abs"))
        {
            ret += Point(train.settings.get<coord_t>("machine_extruder_start_pos_x"), train.settings.get<coord_t>("machine_extruder_start_pos_y"));
            absolute_starting_points++;
        }
    }
    if (absolute_starting_points > 0)
    { // take the average over all absolute starting positions
        ret /= absolute_starting_points;
    }
    else
    { // use the middle of the bed
        ret = storage.machine_size.flatten().getMiddle();
    }
    return ret;
}

void PrimeTower::subtractFromSupport(SliceDataStorage& storage)
{
    const Polygons outside_polygon = outer_poly.getOutsidePolygons();
    AABB outside_polygon_boundary_box(outside_polygon);
    for(size_t layer = 0; layer <= (size_t)storage.max_print_height_second_to_last_extruder + 1 && layer < storage.support.supportLayers.size(); layer++)
    {
        SupportLayer& support_layer = storage.support.supportLayers[layer];
        // take the differences of the support infill parts and the prime tower area
        support_layer.excludeAreasFromSupportInfillAreas(outside_polygon, outside_polygon_boundary_box);
    }
}


}//namespace cura
