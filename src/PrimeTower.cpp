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
: current_wipe_location_idx(0)
{
}



void PrimeTower::initConfigs(MeshGroup* meshgroup, std::vector<RetractionConfig>& retraction_config_per_extruder)
{
    extruder_count = meshgroup->getSettingAsCount("machine_extruder_count");
    
    for (int extr = 0; extr < extruder_count; extr++)
    {
        config_per_extruder.emplace_back(PrintFeatureType::Support);// so that visualization in the old Cura still works (TODO)
    }
    for (int extr = 0; extr < extruder_count; extr++)
    {
        ExtruderTrain* train = meshgroup->getExtruderTrain(extr);
        config_per_extruder[extr].init(train->getSettingInMillimetersPerSecond("speed_prime_tower"), train->getSettingInMillimetersPerSecond("acceleration_prime_tower"), train->getSettingInMillimetersPerSecond("jerk_prime_tower"), train->getSettingInMicrons("prime_tower_line_width"), train->getSettingInPercentage("prime_tower_flow"));
    }
}

void PrimeTower::setConfigs(MeshGroup* meshgroup, int layer_thickness)
{
    
    extruder_count = meshgroup->getSettingAsCount("machine_extruder_count");
    
    for (int extr = 0; extr < extruder_count; extr++)
    {
        GCodePathConfig& conf = config_per_extruder[extr];
        conf.setLayerHeight(layer_thickness);
    }
}

    

void PrimeTower::computePrimeTowerMax(SliceDataStorage& storage)
{ // compute storage.max_object_height_second_to_last_extruder, which is used to determine the highest point in the prime tower
        
    extruder_count = storage.getSettingAsCount("machine_extruder_count");
    
    int max_object_height_per_extruder[extruder_count]; 
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

void PrimeTower::generateGroundpoly(SliceDataStorage& storage) 
{
    PolygonRef p = storage.primeTower.ground_poly.newPoly();
    int tower_size = storage.getSettingInMicrons("prime_tower_size");
    int tower_distance = 0; 
    int x = storage.getSettingInMicrons("prime_tower_position_x"); // storage.model_max.x
    int y = storage.getSettingInMicrons("prime_tower_position_y"); // storage.model_max.y
    p.add(Point(x + tower_distance, y + tower_distance));
    p.add(Point(x + tower_distance, y + tower_distance + tower_size));
    p.add(Point(x + tower_distance - tower_size, y + tower_distance + tower_size));
    p.add(Point(x + tower_distance - tower_size, y + tower_distance));

    wipe_point = Point(x + tower_distance - tower_size / 2, y + tower_distance + tower_size / 2);   
}

void PrimeTower::generatePaths(SliceDataStorage& storage, unsigned int total_layers)
{
    if (storage.max_object_height_second_to_last_extruder >= 0 && storage.getSettingBoolean("prime_tower_enable"))
    {
        generatePaths_denseInfill(storage);
        generateWipeLocations(storage);
    }
}

void PrimeTower::generatePaths_denseInfill(SliceDataStorage& storage)
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
            int outline_offset = -line_width/2;
            int line_distance = line_width;
            double fill_angle = 45 + pattern_idx * 90;
            Polygons& result_lines = patterns[pattern_idx];
            Infill infill_comp(EFillMethod::LINES, ground_poly, outline_offset, line_width, line_distance, infill_overlap, fill_angle, z, extra_infill_shift);
            infill_comp.generate(result_polygons, result_lines);
        }
    }
}

    

void PrimeTower::addToGcode(SliceDataStorage& storage, GCodePlanner& gcodeLayer, GCodeExport& gcode, int layer_nr, int prev_extruder, bool prime_tower_dir_outward, bool wipe, int* last_prime_tower_poly_printed)
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

    if (prev_extruder == gcodeLayer.getExtruder())
    {
        wipe = false;
    }
    // pre-wipe:
    int new_extruder = gcodeLayer.getExtruder();
    preWipe(storage, gcodeLayer, gcodeLayer.getExtruder());

    addToGcode_denseInfill(storage, gcodeLayer, gcode, layer_nr, prev_extruder, prime_tower_dir_outward, wipe, last_prime_tower_poly_printed);

    // post-wipe:
    if (false && wipe) // TODO: make a separate setting for the post-wipe!
    { //Make sure we wipe the old extruder on the prime tower.
        gcodeLayer.addTravel(wipe_point - gcode.getExtruderOffset(prev_extruder) + gcode.getExtruderOffset(new_extruder));
    }
}

void PrimeTower::addToGcode_denseInfill(SliceDataStorage& storage, GCodePlanner& gcodeLayer, GCodeExport& gcode, int layer_nr, int prev_extruder, bool prime_tower_dir_outward, bool wipe, int* last_prime_tower_poly_printed)
{
    if (layer_nr > storage.max_object_height_second_to_last_extruder + 1)
    {
        return;
    }
    
    int new_extruder = gcodeLayer.getExtruder();

    
    Polygons& pattern = patterns_per_extruder[new_extruder][layer_nr % 2];

    
    GCodePathConfig& config = config_per_extruder[new_extruder];
    int start_idx = 0; // TODO: figure out which idx is closest to the far right corner
    gcodeLayer.addPolygon(ground_poly.back(), start_idx, &config);
    gcodeLayer.addLinesByOptimizer(pattern, &config, SpaceFillType::Lines);
    
    last_prime_tower_poly_printed[new_extruder] = layer_nr;

    CommandSocket::sendPolygons(PrintFeatureType::Support, pattern, config.getLineWidth());
}

void PrimeTower::generateWipeLocations(const SliceDataStorage& storage)
{
    PolygonsPointIndex segment_start; // from where to start the sequence of wipe points
    PolygonsPointIndex segment_end; // where to end the sequence of wipe points
    { // find the side of the prime tower which is most likely to be the side from which the prime tower is entered
        Point from(0, 0); // point representing the location before going to the prime tower
        { // find an approriate representation for [from]
            int absolute_starting_points = 0;
            for (int extruder_nr = 0; extruder_nr < storage.meshgroup->getExtruderCount(); extruder_nr++)
            {
                ExtruderTrain& train = *storage.meshgroup->getExtruderTrain(0);
                if (train.getSettingBoolean("machine_extruder_start_pos_abs"))
                {
                    from += Point(train.getSettingInMicrons("machine_extruder_start_pos_x"), train.getSettingInMicrons("machine_extruder_start_pos_y"));
                    absolute_starting_points++;
                }
            }
            if (absolute_starting_points > 0)
            { // take the average over all absolute starting positions
                from /= absolute_starting_points;
            }
            else
            { // use the middle of the bed
                if (!storage.getSettingBoolean("machine_center_is_zero"))
                {
                    from = Point(storage.getSettingInMicrons("machine_width"), storage.getSettingInMicrons("machine_depth")) / 2;
                }
                // otherwise keep (0, 0)
            }
        }

        // naive approach which works for a square prime tower:
        // - find closest vertex
        // - generate wipe locations in the two segments connected to that vertex
        int64_t best_dist2 = std::numeric_limits<int64_t>::max();
        PolygonsPointIndex closest_vert;
        for (unsigned int poly_idx = 0; poly_idx < ground_poly.size(); poly_idx++)
        {
            const PolygonRef poly = ground_poly[poly_idx];
            for (unsigned int point_idx = 0; point_idx < poly.size(); point_idx++)
            {
                int64_t dist2 = vSize2(poly[point_idx] - from);
                if (dist2 < best_dist2)
                {
                    best_dist2 = dist2;
                    closest_vert = PolygonsPointIndex(&ground_poly, poly_idx, point_idx);
                }
            }
        }
        assert(closest_vert.polygons != nullptr && "a closest vert should have been found!");

        const PolygonRef best_poly = (*closest_vert.polygons)[closest_vert.poly_idx];
        segment_start = PolygonsPointIndex(closest_vert.polygons, closest_vert.poly_idx, (closest_vert.point_idx - 1 + best_poly.size()) % best_poly.size());
        segment_end = PolygonsPointIndex(closest_vert.polygons, closest_vert.poly_idx, (closest_vert.point_idx + 1) % best_poly.size());
    }

    int64_t segment_length = 0;
    { // compute segment length
        Point prev_vert = segment_start.p();
        const PolygonRef poly = (*segment_start.polygons)[segment_start.poly_idx];
        for (unsigned int point_idx = segment_start.point_idx + 1; ; point_idx++)
        {
            Point vert = poly[point_idx];
            segment_length += vSize(vert - prev_vert);

            if (point_idx == segment_end.point_idx)
            { // break at the end of the loop, so that segment_end and segment_start may be the same
                break;
            }
            prev_vert = vert;
        }
    }

    int64_t wipe_point_dist = segment_length / (number_of_wipe_locations + 1); // distance between two wipe points; keep a distance at both sides of the segment
    const PolygonRef poly = (*segment_start.polygons)[segment_start.poly_idx];
    int64_t dist_past_vert_to_insert_wipe_point = wipe_point_dist;
    unsigned int number_of_wipe_locations_generated = 0;
    for (unsigned int point_idx = segment_start.point_idx; point_idx != segment_end.point_idx; point_idx++)
    {
        Point p0 = poly[point_idx];
        Point p1 = poly[(point_idx + 1) % poly.size()];
        Point p0p1 = p1 - p0;
        int64_t p0p1_length = vSize(p0p1);

        for ( ; dist_past_vert_to_insert_wipe_point < p0p1_length && number_of_wipe_locations_generated < number_of_wipe_locations; dist_past_vert_to_insert_wipe_point += wipe_point_dist)
        {
            wipe_locations.emplace_back(p0 + normal(p0p1, dist_past_vert_to_insert_wipe_point), point_idx, poly);
            number_of_wipe_locations_generated++;
        }
        dist_past_vert_to_insert_wipe_point -= p0p1_length;
    }
    assert(wipe_locations.size() == number_of_wipe_locations && "we didn't generated as many wipe locations as we asked for.");
}

void PrimeTower::preWipe(SliceDataStorage& storage, GCodePlanner& gcode_layer, const int extruder_nr)
{
    const ClosestPolygonPoint wipe_location = wipe_locations[current_wipe_location_idx];
    current_wipe_location_idx = (current_wipe_location_idx + wipe_location_skip) % number_of_wipe_locations;

    ExtruderTrain& train = *storage.meshgroup->getExtruderTrain(extruder_nr);
    const int inward_dist = train.getSettingInMicrons("machine_nozzle_size") * 3 / 2 ;
    const int start_dist = train.getSettingInMicrons("machine_nozzle_size") * 2;
    const Point end = PolygonUtils::moveInside(wipe_location, inward_dist);
    const Point outward_dir = wipe_location.location - end;
    const Point start = wipe_location.location + normal(outward_dir, start_dist);
    // for hollow wipe tower:
    // start from above
    // go to the level of the previous layer
    // wipe
    // go to normal layer height (automatically on the next extrusion move...
    gcode_layer.addTravel_simple(start); // TODO: verify that this move has a z hop ==> cylindric wipe tower
//     gcode_layer.makeLastPathZhopped which calls forceNewPathStart TODO ==> cylindric wipe tower
    gcode_layer.addTravel_simple(end); // TODO: verify that this move doesn't have a z hop ==> cylindric wipe tower
}


}//namespace cura
