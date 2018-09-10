//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "PrimeTower.h"

#include <algorithm>
#include <limits>

#include "ExtruderTrain.h"
#include "sliceDataStorage.h"
#include "gcodeExport.h"
#include "LayerPlan.h"
#include "infill.h"
#include "PrintFeature.h"
#include "raft.h"

#define CIRCLE_RESOLUTION 32 //The number of vertices in each circle.


namespace cura 
{

PrimeTower::PrimeTower(const SliceDataStorage& storage)
: wipe_from_middle(false)
{
    enabled = storage.getSettingBoolean("prime_tower_enable")
           && storage.getSettingInMicrons("prime_tower_min_volume") > 10
           && storage.getSettingInMicrons("prime_tower_size") > 10;

    extruder_count = storage.meshgroup->getExtruderCount();
    extruder_order.resize(extruder_count);
    for (unsigned int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        extruder_order[extruder_nr] = extruder_nr; //Start with default order, then sort.
    }
    //Sort from high adhesion to low adhesion.
    const SliceDataStorage* storage_ptr = &storage; //Communicate to lambda via pointer to prevent copy.
    std::sort(extruder_order.begin(), extruder_order.end(), [storage_ptr](const unsigned int& extruder_nr_a, const unsigned int& extruder_nr_b) -> bool
    {
        const double adhesion_a = storage_ptr->meshgroup->getExtruderTrain(extruder_nr_a)->getSettingAsRatio("material_adhesion_tendency");
        const double adhesion_b = storage_ptr->meshgroup->getExtruderTrain(extruder_nr_b)->getSettingAsRatio("material_adhesion_tendency");
        return adhesion_a < adhesion_b;
    });
}

void PrimeTower::generateGroundpoly(const SliceDataStorage& storage)
{
    if (!enabled)
    {
        return;
    }

    coord_t tower_size = storage.getSettingInMicrons("prime_tower_size");
    bool circular_prime_tower = storage.getSettingBoolean("prime_tower_circular");

    PolygonRef p = outer_poly.newPoly();
    int tower_distance = 0; 
    int x = storage.getSettingInMicrons("prime_tower_position_x"); // storage.model_max.x
    int y = storage.getSettingInMicrons("prime_tower_position_y"); // storage.model_max.y
    if (circular_prime_tower)
    {
        double_t tower_radius = tower_size / 2;
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

void PrimeTower::generatePaths(const SliceDataStorage& storage)
{
    enabled &= storage.max_print_height_second_to_last_extruder >= 0; //Maybe it turns out that we don't need a prime tower after all because there are no layer switches.
    if (enabled)
    {
        generatePaths_denseInfill(storage);
        generateStartLocations();
    }
}

void PrimeTower::generatePaths_denseInfill(const SliceDataStorage& storage)
{
    const coord_t layer_height = storage.getSettingInMicrons("layer_height");
    pattern_per_extruder.resize(extruder_count);

    coord_t cumulative_inset = 0; //Each tower shape is going to be printed inside the other. This is the inset we're doing for each extruder.
    for (unsigned int extruder : extruder_order)
    {
        const coord_t line_width = storage.meshgroup->getExtruderTrain(extruder)->getSettingInMicrons("prime_tower_line_width");
        const coord_t required_volume = storage.meshgroup->getExtruderTrain(extruder)->getSettingInCubicMillimeters("prime_tower_min_volume") * 1000000000; //To cubic microns.
        const double flow = storage.meshgroup->getExtruderTrain(extruder)->getSettingAsRatio("prime_tower_flow");
        coord_t current_volume = 0;
        ExtrusionMoves& pattern = pattern_per_extruder[extruder];

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
        if (storage.getSettingAsPlatformAdhesion("adhesion_type") != EPlatformAdhesion::RAFT)
        {
            line_width_layer0 *= storage.meshgroup->getExtruderTrain(extruder)->getSettingAsRatio("initial_layer_line_width_factor");
        }
        pattern_per_extruder_layer0.emplace_back();

        ExtrusionMoves& pattern_layer0 = pattern_per_extruder_layer0.back();

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


void PrimeTower::generateStartLocations()
{
    // Evenly spread out a number of dots along the prime tower's outline. This is done for the complete outline,
    // so use the same start and end segments for this.
    PolygonsPointIndex segment_start = PolygonsPointIndex(&outer_poly, 0, 0);
    PolygonsPointIndex segment_end = segment_start;

    PolygonUtils::spreadDots(segment_start, segment_end, number_of_prime_tower_start_locations, prime_tower_start_locations);
}


void PrimeTower::addToGcode(const SliceDataStorage& storage, LayerPlan& gcode_layer, const GCodeExport& gcode, const int prev_extruder, const int new_extruder) const
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

    bool post_wipe = storage.meshgroup->getExtruderTrain(prev_extruder)->getSettingBoolean("prime_tower_wipe_enabled");

    // Do not wipe on the first layer, we will generate non-hollow prime tower there for better bed adhesion.
    const int layer_nr = gcode_layer.getLayerNr();
    if (prev_extruder == new_extruder || layer_nr == 0)
    {
        post_wipe = false;
    }

    // Go to the start location if it's not the first layer
    if (layer_nr != 0)
    {
        gotoStartLocation(storage, gcode_layer, new_extruder);
    }

    addToGcode_denseInfill(storage, gcode_layer, new_extruder);

    // post-wipe:
    if (post_wipe)
    { //Make sure we wipe the old extruder on the prime tower.
        gcode_layer.addTravel(post_wipe_point - gcode.getExtruderOffset(prev_extruder) + gcode.getExtruderOffset(new_extruder));
    }

    gcode_layer.setPrimeTowerIsPlanned(new_extruder);
}

void PrimeTower::addToGcode_denseInfill(const SliceDataStorage& storage, LayerPlan& gcode_layer, const int extruder_nr) const
{
    const ExtrusionMoves& pattern = (gcode_layer.getLayerNr() == -Raft::getFillerLayerCount(storage))
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
    for (unsigned int extruder_nr = 0; extruder_nr < storage.meshgroup->getExtruderCount(); extruder_nr++)
    {
        ExtruderTrain& train = *storage.meshgroup->getExtruderTrain(0);
        if (train.getSettingBoolean("machine_extruder_start_pos_abs"))
        {
            ret += Point(train.getSettingInMicrons("machine_extruder_start_pos_x"), train.getSettingInMicrons("machine_extruder_start_pos_y"));
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


void PrimeTower::gotoStartLocation(const SliceDataStorage& storage, LayerPlan& gcode_layer, const int extruder_nr) const
{
    int current_start_location_idx = ((((extruder_nr + 1) * gcode_layer.getLayerNr()) % number_of_prime_tower_start_locations)
            + number_of_prime_tower_start_locations) % number_of_prime_tower_start_locations;

    const ClosestPolygonPoint wipe_location = prime_tower_start_locations[current_start_location_idx];

    const ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder_nr);
    const coord_t inward_dist = train->getSettingInMicrons("machine_nozzle_size") * 3 / 2 ;
    const coord_t start_dist = train->getSettingInMicrons("machine_nozzle_size") * 2;
    const Point prime_end = PolygonUtils::moveInsideDiagonally(wipe_location, inward_dist);
    const Point outward_dir = wipe_location.location - prime_end;
    const Point prime_start = wipe_location.location + normal(outward_dir, start_dist);

    gcode_layer.addTravel(prime_start);
}


}//namespace cura
