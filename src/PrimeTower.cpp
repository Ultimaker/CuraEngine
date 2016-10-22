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
: current_pre_wipe_location_idx(0)
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

    

void PrimeTower::computePrimeTowerMax(SliceDataStorage& storage)
{ // compute storage.max_object_height_second_to_last_extruder, which is used to determine the highest point in the prime tower
        
    extruder_count = storage.meshgroup->getExtruderCount();
    
    int *max_object_height_per_extruder = (int*)alloca(sizeof(int)*extruder_count); 
    std::fill_n(max_object_height_per_extruder, extruder_count, -1); // unitialize all as -1
    { // compute max_object_height_per_extruder
        for (SliceMeshStorage& mesh : storage.meshes)
        {
            unsigned int extr_nr = mesh.getSettingAsIndex("extruder_nr");
            max_object_height_per_extruder[extr_nr] = 
                std::max(   max_object_height_per_extruder[extr_nr]
                        ,   mesh.layer_nr_max_filled_layer  ); 
        }
        int support_infill_extruder_nr = storage.getSettingAsIndex("support_infill_extruder_nr"); // TODO: support extruder should be configurable per object
        max_object_height_per_extruder[support_infill_extruder_nr] = 
        std::max(   max_object_height_per_extruder[support_infill_extruder_nr]
                ,   storage.support.layer_nr_max_filled_layer  ); 
        int support_skin_extruder_nr = storage.getSettingAsIndex("support_interface_extruder_nr"); // TODO: support skin extruder should be configurable per object
        max_object_height_per_extruder[support_skin_extruder_nr] = 
        std::max(   max_object_height_per_extruder[support_skin_extruder_nr]
                ,   storage.support.layer_nr_max_filled_layer  ); 
    }
    { // // compute max_object_height_second_to_last_extruder
        int extruder_max_object_height = 0;
        for (int extruder_nr = 1; extruder_nr < extruder_count; extruder_nr++)
        {
            if (max_object_height_per_extruder[extruder_nr] > max_object_height_per_extruder[extruder_max_object_height])
            {
                extruder_max_object_height = extruder_nr;
            }
        }
        int extruder_second_max_object_height = -1;
        for (int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
        {
            if (extruder_nr == extruder_max_object_height) { continue; }
            if (extruder_second_max_object_height == -1 || max_object_height_per_extruder[extruder_nr] > max_object_height_per_extruder[extruder_second_max_object_height])
            {
                extruder_second_max_object_height = extruder_nr;
            }
        }
        if (extruder_second_max_object_height < 0)
        {
            storage.max_object_height_second_to_last_extruder = -1;
        }
        else
        {
            storage.max_object_height_second_to_last_extruder = max_object_height_per_extruder[extruder_second_max_object_height];
        }
    }
}

void PrimeTower::generateGroundpoly(const SliceDataStorage& storage)
{
    PolygonRef p = ground_poly.newPoly();
    int tower_size = storage.getSettingInMicrons("prime_tower_size");
    int tower_distance = 0; 
    int x = storage.getSettingInMicrons("prime_tower_position_x"); // storage.model_max.x
    int y = storage.getSettingInMicrons("prime_tower_position_y"); // storage.model_max.y
    p.add(Point(x + tower_distance, y + tower_distance));
    p.add(Point(x + tower_distance, y + tower_distance + tower_size));
    p.add(Point(x + tower_distance - tower_size, y + tower_distance + tower_size));
    p.add(Point(x + tower_distance - tower_size, y + tower_distance));

    post_wipe_point = Point(x + tower_distance - tower_size / 2, y + tower_distance + tower_size / 2);
}

void PrimeTower::generatePaths(const SliceDataStorage& storage, unsigned int total_layers)
{
    if (storage.max_object_height_second_to_last_extruder >= 0 && storage.getSettingBoolean("prime_tower_enable"))
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
        std::vector<Polygons>& patterns = patterns_per_extruder.back();
        for (int pattern_idx = 0; pattern_idx < n_patterns; pattern_idx++)
        {
            Polygons result_polygons; // should remain empty, since we generate lines pattern!
            int outline_offset = -line_width;
            int line_distance = line_width;
            double fill_angle = 45 + pattern_idx * 90;
            Polygons& result_lines = patterns[pattern_idx];
            Infill infill_comp(EFillMethod::LINES, ground_poly, outline_offset, line_width, line_distance, infill_overlap, fill_angle, z, extra_infill_shift);
            infill_comp.generate(result_polygons, result_lines);
        }
    }
}


void PrimeTower::addToGcode(const SliceDataStorage& storage, GCodePlanner& gcodeLayer, const GCodeExport& gcode, const int layer_nr, const int prev_extruder, bool wipe)
{
    if (!( storage.max_object_height_second_to_last_extruder >= 0 && storage.getSettingInMicrons("prime_tower_size") > 0) )
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

    if (layer_nr > storage.max_object_height_second_to_last_extruder + 1)
    {
        return;
    }

    int new_extruder = gcodeLayer.getExtruder();
    if (prev_extruder == gcodeLayer.getExtruder())
    {
        wipe = false;
    }
    // pre-wipe:
    if (wipe)
    {
        preWipe(storage, gcodeLayer, new_extruder);
    }

    addToGcode_denseInfill(storage, gcodeLayer, gcode, layer_nr, prev_extruder);

    // post-wipe:
    if (false && wipe) // TODO: make a separate setting for the post-wipe!
    { //Make sure we wipe the old extruder on the prime tower.
        gcodeLayer.addTravel(post_wipe_point - gcode.getExtruderOffset(prev_extruder) + gcode.getExtruderOffset(new_extruder));
    }
}

void PrimeTower::addToGcode_denseInfill(const SliceDataStorage& storage, GCodePlanner& gcodeLayer, const GCodeExport& gcode, const int layer_nr, const int prev_extruder)
{
    int new_extruder = gcodeLayer.getExtruder();

    Polygons& pattern = patterns_per_extruder[new_extruder][layer_nr % 2];


    GCodePathConfig& config = config_per_extruder[new_extruder];
    int start_idx = 0; // TODO: figure out which idx is closest to the far right corner

    Polygon outer_wall = ground_poly.offset(-config.getLineWidth() / 2).back();
    gcodeLayer.addPolygon(outer_wall, start_idx, &config);
    gcodeLayer.addLinesByOptimizer(pattern, &config, SpaceFillType::Lines);

    last_prime_tower_poly_printed[new_extruder] = layer_nr;

    CommandSocket::sendPolygons(PrintFeatureType::Support, pattern, config.getLineWidth());
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

    // take the closer corner of the wipe tower and generate wipe locations on that side only:
    //
    //     |
    //     |
    //     +-----
    //  .
    //  ^ nozzle switch location

    PolygonsPointIndex segment_start; // from where to start the sequence of wipe points
    PolygonsPointIndex segment_end; // where to end the sequence of wipe points

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

    // TODO: come up with alternatives for better segments once the prime tower can be different shapes

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
    // for hollow wipe tower:
    // start from above
    // go to the level of the previous layer
    // wipe
    // go to normal layer height (automatically on the next extrusion move...
    gcode_layer.addTravel(start); // TODO: verify that this move has a z hop ==> cylindric wipe tower
//     gcode_layer.makeLastPathZhopped which calls forceNewPathStart TODO ==> cylindric wipe tower
    float flow = 0.0001; // force this path being interpreted as an extrusion path, so that no Z hop will occur (TODO: really separately handle travel and extrusion moves)
    gcode_layer.addExtrusionMove(end, &config_per_extruder[extruder_nr], SpaceFillType::None, flow);
}


}//namespace cura
