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
#include "utils/logoutput.h"

#define CIRCLE_RESOLUTION 32 //The number of vertices in each circle.
#define ARC_RESOLUTION 4 //The number of segments in each arc of a wheel


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

    enabled = scene.current_mesh_group->settings.get<PrimeTowerMethod>("prime_tower_mode") != PrimeTowerMethod::NONE
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
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const coord_t tower_size = mesh_group_settings.get<coord_t>("prime_tower_size");
    
    const Settings& brim_extruder_settings = mesh_group_settings.get<ExtruderTrain&>("skirt_brim_extruder_nr").settings;
    const bool has_raft = (mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::RAFT);
    const bool has_prime_brim = mesh_group_settings.get<bool>("prime_tower_brim_enable");
    const coord_t offset = (has_raft || ! has_prime_brim) ? 0 :
        brim_extruder_settings.get<size_t>("brim_line_count") *
        brim_extruder_settings.get<coord_t>("skirt_brim_line_width") *
        brim_extruder_settings.get<Ratio>("initial_layer_line_width_factor");

    const coord_t x = mesh_group_settings.get<coord_t>("prime_tower_position_x") - offset;
    const coord_t y = mesh_group_settings.get<coord_t>("prime_tower_position_y") - offset;
    const coord_t tower_radius = tower_size / 2;
    outer_poly.add(PolygonUtils::makeCircle(Point(x - tower_radius, y + tower_radius), tower_radius, TAU / CIRCLE_RESOLUTION));
    middle = Point(x - tower_size / 2, y + tower_size / 2);

    post_wipe_point = Point(x - tower_size / 2, y + tower_size / 2);

    outer_poly_first_layer = outer_poly.offset(offset);
}

void PrimeTower::generatePaths(const SliceDataStorage& storage)
{
    enabled &= storage.max_print_height_second_to_last_extruder >= 0; //Maybe it turns out that we don't need a prime tower after all because there are no layer switches.
    if (enabled)
    {
        generateGroundpoly();

        std::vector<coord_t> cumulative_insets;
        generatePaths_denseInfill(cumulative_insets);

        generateStartLocations();

        generatePaths_sparseInfill(cumulative_insets);
    }
}

void PrimeTower::generatePaths_denseInfill(std::vector<coord_t> &cumulative_insets)
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

            //Don't continue. We won't ever reach the required volume because it doesn't fit.
            assert(!polygons.empty() && "Prime tower is not large enough to generate the required volume");
        }
        cumulative_inset += wall_nr * line_width;
        cumulative_insets.push_back(cumulative_inset);

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

void PrimeTower::generatePaths_sparseInfill(const std::vector<coord_t> &cumulative_insets)
{
    const Scene& scene = Application::getInstance().current_slice->scene;
    const Settings& mesh_group_settings = scene.current_mesh_group->settings;
    const PrimeTowerMethod method = mesh_group_settings.get<PrimeTowerMethod>("prime_tower_mode");

    if(method == PrimeTowerMethod::OPTIMIZED || method == PrimeTowerMethod::OPTIMIZED_CONSISTENT)
    {
        const size_t nb_extruders = scene.extruders.size();

        // Pre-compute radiuses of each extruder ring
        std::vector<coord_t> rings_radii;
        const coord_t tower_size = mesh_group_settings.get<coord_t>("prime_tower_size");
        const coord_t tower_radius = tower_size / 2;

        rings_radii.push_back(tower_radius);
        for(const coord_t &cumulative_inset : cumulative_insets)
        {
            rings_radii.push_back(tower_radius - cumulative_inset);
        }

        // Generate all possible extruders combinations, e.g. if there are 4 extruders, we have combinations
        // 0 / 0-1 / 0-1-2 / 0-1-2-3 / 1 / 1-2 / 1-2-3 / 2 / 2-3 / 3
        // A combination is represented by a bitmask
        std::vector<size_t> extruders_combinations;

        for(size_t first_extruder = 0 ; first_extruder < nb_extruders ; ++first_extruder)
        {
            for(size_t last_extruder = first_extruder ; last_extruder < nb_extruders ; ++last_extruder)
            {
                size_t extruders_combination = 0;
                for(size_t extruder_nr = first_extruder ; extruder_nr <= last_extruder ; ++extruder_nr)
                {
                    extruders_combination |= (1 << extruder_nr);
                }

                sparse_pattern_per_extruders[extruders_combination] = generatePath_sparseInfill(first_extruder, last_extruder, rings_radii);
            }
        }
    }
}

PrimeTower::ExtrusionMoves PrimeTower::generatePath_sparseInfill(const size_t first_extruder, const size_t last_extruder, const std::vector<coord_t> &rings_radii)
{
    const Scene& scene = Application::getInstance().current_slice->scene;
    const Settings& mesh_group_settings = scene.current_mesh_group->settings;
    const coord_t max_bridging_distance = mesh_group_settings.get<coord_t>("prime_tower_max_briding_distance");
    const coord_t outer_radius = rings_radii[first_extruder];
    const coord_t inner_radius = rings_radii[last_extruder + 1];
    const coord_t radius_delta = outer_radius - inner_radius;

    // Split ring according to max bridging distance
    const size_t nb_rings = std::ceil(static_cast<float>(radius_delta) / max_bridging_distance);
    const coord_t actual_radius_step = radius_delta / nb_rings;

    ExtrusionMoves pattern;
    for(size_t i = 0 ; i < nb_rings ; ++i)
    {
        const coord_t ring_inner_radius = inner_radius + i * actual_radius_step;
        const coord_t ring_outer_radius = inner_radius + (i + 1) * actual_radius_step;

        const size_t semi_nb_spokes = std::ceil((M_PI * ring_outer_radius) / max_bridging_distance);

        pattern.polygons.add(PolygonUtils::makeWheel(middle,
                                                     ring_inner_radius,
                                                     ring_outer_radius,
                                                     semi_nb_spokes,
                                                     ARC_RESOLUTION));
    }

    return pattern;
}

void PrimeTower::generateStartLocations()
{
    // Evenly spread out a number of dots along the prime tower's outline. This is done for the complete outline,
    // so use the same start and end segments for this.
    PolygonsPointIndex segment_start = PolygonsPointIndex(&outer_poly, 0, 0);
    PolygonsPointIndex segment_end = segment_start;

    PolygonUtils::spreadDots(segment_start, segment_end, number_of_prime_tower_start_locations, prime_tower_start_locations);
}

void PrimeTower::addToGcode(const SliceDataStorage& storage, LayerPlan& gcode_layer, const std::vector<bool> &required_extruder_prime, const size_t prev_extruder, const size_t new_extruder) const
{
    if (!enabled)
    {
        return;
    }
    #warning remove this
    //log("add to gcode %d %d %d\n", static_cast<int>(gcode_layer.getLayerNr()), prev_extruder, new_extruder);
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

    PrimeTowerMethod method = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<PrimeTowerMethod>("prime_tower_mode");
    std::vector<size_t> primed_extruders;

    switch(method)
    {
        case PrimeTowerMethod::NONE:
            // This should actually not happen
            break;

        case PrimeTowerMethod::DEFAULT:
            addToGcode_denseInfill(gcode_layer, new_extruder);
            primed_extruders.push_back(new_extruder);
            break;

        case PrimeTowerMethod::OPTIMIZED:
            if(required_extruder_prime[new_extruder])
            {
                // Extruder really needs to be prime
                addToGcode_denseInfill(gcode_layer, new_extruder);
                primed_extruders.push_back(new_extruder);
            }

            // Whatever happens before and after, use the current extruder to prime all the non-required extruders now
            addToGcode_optimizedInfill(gcode_layer, required_extruder_prime, new_extruder, primed_extruders);
            break;
    }

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

    for (const size_t &primed_extruder : primed_extruders)
    {
        gcode_layer.setPrimeTowerIsPlanned(primed_extruder);
    }
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

void PrimeTower::addToGcode_optimizedInfill(LayerPlan& gcode_layer, const std::vector<bool> &required_extruder_prime, const size_t current_extruder, std::vector<size_t> &primed_extruders) const
{
    std::vector<size_t> extruders_to_prime;

    // First, gather all extruders to be primed : we are going to process them all now, even if
    // the rings are not besides each other
    for(size_t extruder_nr = 0 ; extruder_nr < required_extruder_prime.size() ; ++extruder_nr)
    {
        if (!required_extruder_prime[extruder_nr] && !gcode_layer.getPrimeTowerIsPlanned(extruder_nr))
        {
            extruders_to_prime.push_back(extruder_nr);
            primed_extruders.push_back(extruder_nr);
        }
    }

    // Now, group extruders which are besides each other
    std::vector<std::vector<size_t>> extruders_to_prime_grouped;
    for(const size_t &extruder_to_prime : extruders_to_prime)
    {
        if(extruders_to_prime_grouped.empty())
        {
            // First extruder : create new group
            extruders_to_prime_grouped.push_back({extruder_to_prime});
        }
        else
        {
            std::vector<size_t> &last_group = extruders_to_prime_grouped.back();
            if(last_group.back() == extruder_to_prime - 1)
            {
                // New extruders which belongs to same group
                last_group.push_back(extruder_to_prime);
            }
            else
            {
                // New extruders which belongs to new group
                extruders_to_prime_grouped.push_back({extruder_to_prime});
            }
        }
    }

    #warning remove this
    #if 0
    log("GROUPS\n");
    for(auto &group : extruders_to_prime_grouped)
    {
        log("NEW GROUP\n");
        for(auto &extruder : group)
        {
            log("E%d\n", extruder);
        }
    }
    log("GROUPS END\n");
    #endif

    #warning What should we do in case of extruders with different lines widths ?
    // And finally, append patterns for each group
    const GCodePathConfig& config = gcode_layer.configs_storage.prime_tower_config_per_extruder[current_extruder];
    log("line width %d %d\n", config.getLineWidth(), current_extruder);
    for(const std::vector<size_t> &group : extruders_to_prime_grouped)
    {
        size_t mask = 0;
        for(const size_t &extruder : group)
        {
            mask |= (1 << extruder);
        }

        auto iterator = sparse_pattern_per_extruders.find(mask);
        if(iterator != sparse_pattern_per_extruders.end())
        {

            gcode_layer.addPolygonsByOptimizer(iterator->second.polygons, config);
        }
        else
        {
            logWarning("Sparse pattern not found for group %d, skipping\n", mask);
        }
    }
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
