//Copyright (c) 2022 Ultimaker B.V.
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
#include "Scene.h"
#include "Slice.h"
#include "sliceDataStorage.h"

#define CIRCLE_RESOLUTION 32 //The number of vertices in each circle.


namespace cura 
{

PrimeTower::PrimeTower()
: wipe_from_middle(false)
{
    const Scene& scene = Application::getInstance().current_slice->scene;

    {
        EPlatformAdhesion adhesion_type = scene.current_mesh_group->settings.get<EPlatformAdhesion>("adhesion_type");

        //When we have multiple extruders sharing the same heater/nozzle, we expect that all the extruders have been
        //'primed' by the print-start gcode script, but we don't know which one has been left at the tip of the nozzle
        //and whether it needs 'purging' (before extruding a pure material) or not, so we need to prime (actually purge)
        //each extruder before it is used for the model. This can done by the (per-extruder) brim lines or (per-extruder)
        //skirt lines when they are used, but we need to do that inside the first prime-tower layer when they are not
        //used (sacrifying for this purpose the usual single-extruder first layer, that would be better for prime-tower
        //adhesion).

        multiple_extruders_on_first_layer = scene.current_mesh_group->settings.get<bool>("machine_extruders_share_nozzle") && ((adhesion_type != EPlatformAdhesion::SKIRT) && (adhesion_type != EPlatformAdhesion::BRIM));
    }

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
    std::stable_sort(extruder_order.begin(), extruder_order.end(), [scene_pointer](const unsigned int& extruder_nr_a, const unsigned int& extruder_nr_b) -> bool
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

    const coord_t x = mesh_group_settings.get<coord_t>("prime_tower_position_x");
    const coord_t y = mesh_group_settings.get<coord_t>("prime_tower_position_y");
    const coord_t tower_radius = tower_size / 2;
    outer_poly.add(PolygonUtils::makeCircle(Point(x - tower_radius, y + tower_radius), tower_radius, TAU / CIRCLE_RESOLUTION));
    middle = Point(x - tower_size / 2, y + tower_size / 2);

    post_wipe_point = Point(x - tower_size / 2, y + tower_size / 2);
}

void PrimeTower::generatePaths(const SliceDataStorage& storage)
{
    enabled &= storage.max_print_height_second_to_last_extruder >= 0; //Maybe it turns out that we don't need a prime tower after all because there are no layer switches.
    if (enabled)
    {
        generatePaths_denseInfill();
        generateStartLocations();
    }
}

void PrimeTower::generatePaths_denseInfill()
{
    const Scene& scene = Application::getInstance().current_slice->scene;
    const Settings& mesh_group_settings = scene.current_mesh_group->settings;
    const coord_t layer_height = mesh_group_settings.get<coord_t>("layer_height");
    pattern_per_extruder.resize(extruder_count);
    pattern_per_extruder_layer0.resize(extruder_count);

    coord_t cumulative_inset = 0; //Each tower shape is going to be printed inside the other. This is the inset we're doing for each extruder.
    for (size_t extruder_nr : extruder_order)
    {
        const coord_t line_width = scene.extruders[extruder_nr].settings.get<coord_t>("prime_tower_line_width");
        const coord_t required_volume = MM3_2INT(scene.extruders[extruder_nr].settings.get<double>("prime_tower_min_volume"));
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

        if (multiple_extruders_on_first_layer)
        {
            //With a raft there is no difference for the first layer (of the prime tower)
            pattern_per_extruder_layer0 = pattern_per_extruder;
        }
        else
        {
            //Generate the pattern for the first layer.
            coord_t line_width_layer0 = line_width * scene.extruders[extruder_nr].settings.get<Ratio>("initial_layer_line_width_factor");
            ExtrusionMoves& pattern_layer0 = pattern_per_extruder_layer0[extruder_nr];

            // Generate a concentric infill pattern in the form insets for the prime tower's first layer instead of using
            // the infill pattern because the infill pattern tries to connect polygons in different insets which causes the
            // first layer of the prime tower to not stick well.
            Polygons inset = outer_poly.offset(-line_width_layer0 / 2);
            while (!inset.empty())
            {
                pattern_layer0.polygons.add(inset);
                inset = inset.offset(-line_width_layer0);
            }
        }
    }
}

void PrimeTower::generateStartLocations()
{
    // Evenly spread out a number of dots along the prime tower's outline. This is done for the complete outline,
    // so use the same start and end segments for this.
    PolygonsPointIndex segment_start = PolygonsPointIndex(&outer_poly, 0, 0);
    PolygonsPointIndex segment_end = segment_start;

    PolygonUtils::spreadDots(segment_start, segment_end, number_of_prime_tower_start_locations, prime_tower_start_locations);
}

void PrimeTower::addToGcode(const SliceDataStorage& storage, LayerPlan& gcode_layer, const size_t prev_extruder, const size_t new_extruder) const
{
    if (!enabled)
    {
        return;
    }
    if (gcode_layer.getPrimeTowerIsPlanned(new_extruder))
    { // don't print the prime tower if it has been printed already with this extruder.
        return;
    }

    const LayerIndex layer_nr = gcode_layer.getLayerNr();
    if (layer_nr > storage.max_print_height_second_to_last_extruder + 1)
    {
        return;
    }

    bool post_wipe = Application::getInstance().current_slice->scene.extruders[prev_extruder].settings.get<bool>("prime_tower_wipe_enabled");

    // Do not wipe on the first layer, we will generate non-hollow prime tower there for better bed adhesion.
    if (prev_extruder == new_extruder || layer_nr == 0)
    {
        post_wipe = false;
    }

    // Go to the start location if it's not the first layer
    if (layer_nr != 0)
    {
        gotoStartLocation(gcode_layer, new_extruder);
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

void PrimeTower::gotoStartLocation(LayerPlan& gcode_layer, const int extruder_nr) const
{
    int current_start_location_idx = ((((extruder_nr + 1) * gcode_layer.getLayerNr()) % number_of_prime_tower_start_locations)
            + number_of_prime_tower_start_locations) % number_of_prime_tower_start_locations;

    const ClosestPolygonPoint wipe_location = prime_tower_start_locations[current_start_location_idx];

    const ExtruderTrain& train = Application::getInstance().current_slice->scene.extruders[extruder_nr];
    const coord_t inward_dist = train.settings.get<coord_t>("machine_nozzle_size") * 3 / 2 ;
    const coord_t start_dist = train.settings.get<coord_t>("machine_nozzle_size") * 2;
    const Point prime_end = PolygonUtils::moveInsideDiagonally(wipe_location, inward_dist);
    const Point outward_dir = wipe_location.location - prime_end;
    const Point prime_start = wipe_location.location + normal(outward_dir, start_dist);

    gcode_layer.addTravel(prime_start);
}

}//namespace cura
