#include "PrimeTower.h"

#include <limits>

#include "ExtruderTrain.h"
#include "sliceDataStorage.h"
#include "gcodeExport.h"
#include "gcodePlanner.h"
#include "infill.h"
#include "PrintFeature.h"

namespace cura 
{

PrimeTower::PrimeTower()
: is_hollow(false)
, current_pre_wipe_location_idx(0)
{
    for (int extruder_nr = 0; extruder_nr < MAX_EXTRUDERS; extruder_nr++)
    {
        last_prime_tower_poly_printed[extruder_nr] = -1;
    }
}



void PrimeTower::initConfigs(const MeshGroup* meshgroup)
{
    extruder_count = meshgroup->getExtruderCount();

    for (int extr = 0; extr < extruder_count; extr++)
    {
        config_per_extruder.emplace_back(PrintFeatureType::Support);// so that visualization in the old Cura still works (TODO)
    }
    for (int extr = 0; extr < extruder_count; extr++)
    {
        const ExtruderTrain* train = meshgroup->getExtruderTrain(extr);
        config_per_extruder[extr].init(train->getSettingInMillimetersPerSecond("speed_prime_tower"), train->getSettingInMillimetersPerSecond("acceleration_prime_tower"), train->getSettingInMillimetersPerSecond("jerk_prime_tower"), train->getSettingInMicrons("prime_tower_line_width"), train->getSettingInPercentage("prime_tower_flow"));
    }
}

void PrimeTower::setConfigs(const MeshGroup* meshgroup, const int layer_thickness)
{

    extruder_count = meshgroup->getExtruderCount();

    for (int extr = 0; extr < extruder_count; extr++)
    {
        GCodePathConfig& conf = config_per_extruder[extr];
        conf.setLayerHeight(layer_thickness);
    }
}

void PrimeTower::generateGroundpoly(const SliceDataStorage& storage)
{
    int64_t prime_tower_wall_thickness = storage.getSettingInMicrons("prime_tower_wall_thickness");
    int64_t tower_size = storage.getSettingInMicrons("prime_tower_size");

    if (prime_tower_wall_thickness * 2 < tower_size)
    {
        is_hollow = true;
    }

    PolygonRef p = ground_poly.newPoly();
    int tower_distance = 0; 
    int x = storage.getSettingInMicrons("prime_tower_position_x"); // storage.model_max.x
    int y = storage.getSettingInMicrons("prime_tower_position_y"); // storage.model_max.y
    p.add(Point(x + tower_distance, y + tower_distance));
    p.add(Point(x + tower_distance, y + tower_distance + tower_size));
    p.add(Point(x + tower_distance - tower_size, y + tower_distance + tower_size));
    p.add(Point(x + tower_distance - tower_size, y + tower_distance));
    middle = Point(x - tower_size / 2, y + tower_size / 2);

    if (is_hollow)
    {
        ground_poly = ground_poly.difference(ground_poly.offset(-prime_tower_wall_thickness));
    }

    post_wipe_point = Point(x + tower_distance - tower_size / 2, y + tower_distance + tower_size / 2);
}

void PrimeTower::generatePaths(const SliceDataStorage& storage, unsigned int total_layers)
{
    enabled = storage.max_print_height_second_to_last_extruder >= 0
            && storage.getSettingBoolean("prime_tower_enable")
            && storage.getSettingInMicrons("prime_tower_wall_thickness") > 10
            && storage.getSettingInMicrons("prime_tower_size") > 10;
    if (enabled)
    {
        generatePaths_denseInfill(storage);
        generateWipeLocations(storage);
    }
}

void PrimeTower::generatePaths_denseInfill(const SliceDataStorage& storage)
{
    int n_patterns = 2; // alternating patterns between layers
    int infill_overlap = 60; // so that it can't be zero; EDIT: wtf?
    int extra_infill_shift = 0;

    generateGroundpoly(storage);

    int64_t z = 0; // (TODO) because the prime tower stores the paths for each extruder for once instead of generating each layer, we don't know the z position

    for (int extruder = 0; extruder < extruder_count; extruder++)
    {
        int line_width = storage.meshgroup->getExtruderTrain(extruder)->getSettingInMicrons("prime_tower_line_width");
        patterns_per_extruder.emplace_back(n_patterns);
        std::vector<ExtrusionMoves>& patterns = patterns_per_extruder.back();
        patterns.resize(n_patterns);
        for (int pattern_idx = 0; pattern_idx < n_patterns; pattern_idx++)
        {
            patterns[pattern_idx].polygons = ground_poly.offset(-line_width / 2);
            Polygons& result_lines = patterns[pattern_idx].lines;
            int outline_offset = -line_width;
            int line_distance = line_width;
            double fill_angle = 45 + pattern_idx * 90;
            Polygons result_polygons; // should remain empty, since we generate lines pattern!
            Infill infill_comp(EFillMethod::LINES, ground_poly, outline_offset, line_width, line_distance, infill_overlap, fill_angle, z, extra_infill_shift);
            infill_comp.generate(result_polygons, result_lines);
        }
    }
}


void PrimeTower::addToGcode(const SliceDataStorage& storage, GCodePlanner& gcodeLayer, const GCodeExport& gcode, const int layer_nr, const int prev_extruder, const int new_extruder)
{
    if (!enabled)
    {
        return;
    }
    bool prime_tower_added = false;
    for (int extruder = 0; extruder <  storage.meshgroup->getExtruderCount() && !prime_tower_added; extruder++)
    {
        prime_tower_added = last_prime_tower_poly_printed[extruder] == int(layer_nr);
    }
    if (prime_tower_added)
    { // don't print the prime tower if it has been printed already
        return;
    }

    if (layer_nr > storage.max_print_height_second_to_last_extruder + 1)
    {
        return;
    }

    bool pre_wipe = storage.meshgroup->getExtruderTrain(new_extruder)->getSettingBoolean("dual_pre_wipe");
    bool post_wipe = storage.meshgroup->getExtruderTrain(prev_extruder)->getSettingBoolean("prime_tower_wipe_enabled");

    if (prev_extruder == new_extruder)
    {
        pre_wipe = false;
        post_wipe = false;
    }
    // pre-wipe:
    if (pre_wipe)
    {
        preWipe(storage, gcodeLayer, new_extruder);
    }

    addToGcode_denseInfill(storage, gcodeLayer, gcode, layer_nr, prev_extruder, new_extruder);

    // post-wipe:
    if (post_wipe)
    { //Make sure we wipe the old extruder on the prime tower.
        gcodeLayer.addTravel(post_wipe_point - gcode.getExtruderOffset(prev_extruder) + gcode.getExtruderOffset(new_extruder));
    }
}

void PrimeTower::addToGcode_denseInfill(const SliceDataStorage& storage, GCodePlanner& gcodeLayer, const GCodeExport& gcode, const int layer_nr, const int prev_extruder, const int new_extruder)
{
    ExtrusionMoves& pattern = patterns_per_extruder[new_extruder][layer_nr % 2];

    GCodePathConfig& config = config_per_extruder[new_extruder];

    gcodeLayer.addPolygonsByOptimizer(pattern.polygons, &config);
    gcodeLayer.addLinesByOptimizer(pattern.lines, &config, SpaceFillType::Lines);

    last_prime_tower_poly_printed[new_extruder] = layer_nr;

    CommandSocket::sendPolygons(PrintFeatureType::Support, pattern.polygons, config.getLineWidth());
    CommandSocket::sendPolygons(PrintFeatureType::Support, pattern.lines, config.getLineWidth());
}

Point PrimeTower::getLocationBeforePrimeTower(const SliceDataStorage& storage)
{
    Point ret(0, 0);
    int absolute_starting_points = 0;
    for (int extruder_nr = 0; extruder_nr < storage.meshgroup->getExtruderCount(); extruder_nr++)
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
        if (!storage.getSettingBoolean("machine_center_is_zero"))
        {
            ret = Point(storage.getSettingInMicrons("machine_width"), storage.getSettingInMicrons("machine_depth")) / 2;
        }
        // otherwise keep (0, 0)
    }
    return ret;
}

void PrimeTower::generateWipeLocations(const SliceDataStorage& storage)
{
    Point from = getLocationBeforePrimeTower(storage);


    PolygonsPointIndex segment_start; // from where to start the sequence of wipe points
    PolygonsPointIndex segment_end; // where to end the sequence of wipe points

    if (is_hollow)
    {
        // take the same start as end point so that the whole poly os covered.
        // find the inner polygon.
        segment_start = segment_end = PolygonUtils::findNearestVert(middle, ground_poly);
    }
    else
    {
        // take the closer corner of the wipe tower and generate wipe locations on that side only:
        //
        //     |
        //     |
        //     +-----
        //  .
        //  ^ nozzle switch location

        // find the single line segment closest to [from] pointing most toward [from]
        PolygonsPointIndex closest_vert = PolygonUtils::findNearestVert(from, ground_poly);
        PolygonsPointIndex prev = closest_vert.prev();
        PolygonsPointIndex next = closest_vert.next();
        int64_t prev_dot_score = dot(from - closest_vert.p(), turn90CCW(prev.p() - closest_vert.p()));
        int64_t next_dot_score = dot(from - closest_vert.p(), turn90CCW(closest_vert.p() - next.p()));
        if (prev_dot_score > next_dot_score)
        {
            segment_start = prev;
            segment_end = closest_vert;
        }
        else
        {
            segment_start = closest_vert;
            segment_end = next;
        }
    }

    PolygonUtils::spreadDots(segment_start, segment_end, number_of_pre_wipe_locations, pre_wipe_locations);
}

void PrimeTower::preWipe(const SliceDataStorage& storage, GCodePlanner& gcode_layer, const int extruder_nr)
{
    const ClosestPolygonPoint wipe_location = pre_wipe_locations[current_pre_wipe_location_idx];
    current_pre_wipe_location_idx = (current_pre_wipe_location_idx + pre_wipe_location_skip) % number_of_pre_wipe_locations;

    ExtruderTrain& train = *storage.meshgroup->getExtruderTrain(extruder_nr);
    const int inward_dist = train.getSettingInMicrons("machine_nozzle_size") * 3 / 2 ;
    const int start_dist = train.getSettingInMicrons("machine_nozzle_size") * 2;
    const Point end = PolygonUtils::moveInsideDiagonally(wipe_location, inward_dist);
    const Point outward_dir = wipe_location.location - end;
    const Point start = wipe_location.location + normal(outward_dir, start_dist);
    if (is_hollow)
    {
        // for hollow wipe tower:
        // start from above
        // go to wipe start
        // go to the Z height of the previous/current layer
        // wipe
        // go to normal layer height (automatically on the next extrusion move...
        GCodePath& toward_middle = gcode_layer.addTravel(middle);
        toward_middle.perform_z_hop = true;
        gcode_layer.forceNewPathStart();
        GCodePath& toward_wipe_start = gcode_layer.addTravel_simple(start);
        toward_wipe_start.perform_z_hop = false; // TODO problem: travel moves ignore actual requested height untill destination is reached :(
    }
    else
    {
        gcode_layer.addTravel(start);
    }
    float flow = 0.0001; // force this path being interpreted as an extrusion path, so that no Z hop will occur (TODO: really separately handle travel and extrusion moves)
    gcode_layer.addExtrusionMove(end, &config_per_extruder[extruder_nr], SpaceFillType::None, flow);
}


}//namespace cura
