#include "PrimeTower.h"

#include "ExtruderTrain.h"
#include "sliceDataStorage.h"
#include "gcodeExport.h"
#include "gcodePlanner.h"
#include "infill.h"
#include "PrintFeature.h"

namespace cura 
{
    
    
PrimeTower::PrimeTower()
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

    storage.wipePoint = Point(x + tower_distance - tower_size / 2, y + tower_distance + tower_size / 2);   
}

void PrimeTower::generatePaths(SliceDataStorage& storage, unsigned int total_layers)
{
    if (storage.max_object_height_second_to_last_extruder >= 0 && storage.getSettingBoolean("prime_tower_enable"))
    {
        generatePaths3(storage);
    }
}
void PrimeTower::generatePaths_OLD(SliceDataStorage& storage, unsigned int total_layers)
{
    
    if (storage.max_object_height_second_to_last_extruder >= 0 && storage.getSettingBoolean("prime_tower_enable"))
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

        storage.wipePoint = Point(x + tower_distance - tower_size / 2, y + tower_distance + tower_size / 2);
    }
}
    
    
void PrimeTower::generatePaths2(SliceDataStorage& storage) // half baked attempt at spiral shaped prime tower pattern
{
//     extruder_count = storage.getSettingAsCount("machine_extruder_count");
//     
//     int64_t line_dists[extruder_count + 1]; // distance between the lines of different extruders, and half the line dist for beginning and ending
//     int64_t total_width = 0;
//     {
//         int64_t last_line_width = 0;
//         for (int extr = 0; extr < extruder_count; extr++)
//         {
//             int64_t line_width = config_per_extruder[extr].getLineWidth();
//             line_dists[extr] = (line_width + last_line_width) / 2;
//             total_width += line_width;
//             last_line_width = line_width;
//         }
//         line_dists[extruder_count] = last_line_width / 2;
//     }
//     
    
    
}

void PrimeTower::generatePaths3(SliceDataStorage& storage)
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
    addToGcode3(storage, gcodeLayer, gcode, layer_nr, prev_extruder, prime_tower_dir_outward, wipe, last_prime_tower_poly_printed);
}

void PrimeTower::addToGcode3(SliceDataStorage& storage, GCodePlanner& gcodeLayer, GCodeExport& gcode, int layer_nr, int prev_extruder, bool prime_tower_dir_outward, bool wipe, int* last_prime_tower_poly_printed)
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

    if (wipe)
    { //Make sure we wipe the old extruder on the prime tower.
        gcodeLayer.addTravel(storage.wipePoint - gcode.getExtruderOffset(prev_extruder) + gcode.getExtruderOffset(new_extruder));
    }
}

void PrimeTower::addToGcode_OLD(SliceDataStorage& storage, GCodePlanner& gcodeLayer, GCodeExport& gcode, int layer_nr, int prev_extruder, bool prime_tower_dir_outward, bool wipe, int* last_prime_tower_poly_printed)
{
    if (layer_nr > storage.max_object_height_second_to_last_extruder + 1)
    {
        return;
    }
    
    int new_extruder = gcodeLayer.getExtruder();

    int64_t offset = -config_per_extruder[new_extruder].getLineWidth(); 
    if (layer_nr > 0)
        offset *= 2;
    
    //If we changed extruder, print the wipe/prime tower for this nozzle;
    std::vector<Polygons> insets;
    { // generate polygons
        if ((layer_nr % 2) == 1)
            insets.push_back(storage.primeTower.ground_poly.offset(offset / 2));
        else
            insets.push_back(storage.primeTower.ground_poly);
        while(true)
        {
            Polygons new_inset = insets[insets.size() - 1].offset(offset);
            if (new_inset.size() < 1)
                break;
            insets.push_back(new_inset);
        }
    }
    
    
    for(unsigned int n=0; n<insets.size(); n++)
    {
        GCodePathConfig& config = config_per_extruder[new_extruder];
        gcodeLayer.addPolygonsByOptimizer(insets[(prime_tower_dir_outward)? insets.size() - 1 - n : n], &config);
    }
    last_prime_tower_poly_printed[new_extruder] = layer_nr;
    
    if (wipe)
    { //Make sure we wipe the old extruder on the prime tower.
        gcodeLayer.addTravel(storage.wipePoint - gcode.getExtruderOffset(prev_extruder) + gcode.getExtruderOffset(new_extruder));
    }
};
    
    
    
    
    
    
    
    
    
    
    
    
    
    
}//namespace cura
