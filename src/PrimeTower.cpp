#include "PrimeTower.h"

#include "ExtruderTrain.h"
#include "sliceDataStorage.h"
#include "gcodeExport.h"
#include "gcodePlanner.h"

namespace cura 
{
    
    
PrimeTower::PrimeTower(MeshGroup* configs, RetractionConfig* retraction_config)
{
//     setConfigs(configs);
}

void PrimeTower::computePrimeTowerMax(SliceDataStorage& storage)
{ // compute storage.max_object_height_second_to_last_extruder, which is used to determine the highest point in the prime tower
        
    extruder_count = storage.getSettingAsCount("machine_extruder_count");
    
    int max_object_height_per_extruder[extruder_count];
    { // compute max_object_height_per_extruder
        memset(max_object_height_per_extruder, -1, sizeof(max_object_height_per_extruder));
        for (SliceMeshStorage& mesh : storage.meshes)
        {
            max_object_height_per_extruder[mesh.getSettingAsIndex("extruder_nr")] = 
                std::max(   max_object_height_per_extruder[mesh.getSettingAsIndex("extruder_nr")]
                        ,   mesh.layer_nr_max_filled_layer  ); 
        }
        int support_extruder_nr = storage.getSettingAsIndex("support_extruder_nr"); // TODO: support extruder should be configurable per object
        max_object_height_per_extruder[support_extruder_nr] = 
        std::max(   max_object_height_per_extruder[support_extruder_nr]
                ,   storage.support.layer_nr_max_filled_layer  ); 
        int support_roof_extruder_nr = storage.getSettingAsIndex("support_roof_extruder_nr"); // TODO: support roof extruder should be configurable per object
        max_object_height_per_extruder[support_roof_extruder_nr] = 
        std::max(   max_object_height_per_extruder[support_roof_extruder_nr]
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
            if (max_object_height_per_extruder[extruder_nr] > max_object_height_per_extruder[extruder_second_max_object_height])
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

void PrimeTower::generatePaths2(SliceDataStorage& storage) // half baked attempt at spiral shaped prime tower pattern
{
    extruder_count = storage.getSettingAsCount("machine_extruder_count");
    
    int64_t line_dists[extruder_count + 1]; // distance between the lines of different extruders, and half the line dist for beginning and ending
    int64_t total_width = 0;
    {
        int64_t last_line_width = 0;
        for (int extr = 0; extr < extruder_count; extr++)
        {
            int64_t line_width = config_per_extruder[extr].getLineWidth();
            line_dists[extr] = (line_width + last_line_width) / 2;
            total_width += line_width;
            last_line_width = line_width;
        }
        line_dists[extruder_count] = last_line_width / 2;
    }
    
    
    
}


void PrimeTower::generatePaths(SliceDataStorage& storage, unsigned int totalLayers)
{
    extruder_count = storage.getSettingAsCount("machine_extruder_count");
    
    if (storage.max_object_height_second_to_last_extruder >= 0 && storage.getSettingInMicrons("prime_tower_distance") > 0 && storage.getSettingInMicrons("prime_tower_size") > 0)
    {
        PolygonRef p = storage.primeTower.ground_poly.newPoly();
        int tower_size = storage.getSettingInMicrons("prime_tower_size");
        int tower_distance = 0; //storage.getSettingInMicrons("prime_tower_distance");
        int x = storage.getSettingInMicrons("prime_tower_position_x"); // storage.model_max.x
        int y = storage.getSettingInMicrons("prime_tower_position_y"); // storage.model_max.y
        p.add(Point(x + tower_distance, y + tower_distance));
        p.add(Point(x + tower_distance, y + tower_distance + tower_size));
        p.add(Point(x + tower_distance - tower_size, y + tower_distance + tower_size));
        p.add(Point(x + tower_distance - tower_size, y + tower_distance));

        storage.wipePoint = Point(x + tower_distance - tower_size / 2, y + tower_distance + tower_size / 2);
    }
}
    
void PrimeTower::setConfigs(MeshGroup* meshgroup, RetractionConfig* retraction_config, int layer_thickness)
{
    for (int extr = 0; extr < extruder_count; extr++)
    {
        ExtruderTrain* extruder = meshgroup->extruders[extr];
        config_per_extruder.emplace_back(retraction_config, "WALL-INNER");// so that visualization in the old Cura still works (TODO)
        GCodePathConfig& conf = config_per_extruder.back();
        
        conf.setSpeed(extruder->getSettingInMillimetersPerSecond("speed_prime_tower"));
        conf.setLineWidth(extruder->getSettingInMicrons("prime_tower_line_width"));
        conf.setFlow(extruder->getSettingInPercentage("prime_tower_flow"));
        conf.setLayerHeight(layer_thickness);
    }
}

    
    

void PrimeTower::addToGcode(SliceDataStorage& storage, GCodePlanner& gcodeLayer, GCodeExport& gcode, int layer_nr, int prev_extruder, bool prime_tower_dir_outward, bool wipe, int* last_prime_tower_poly_printed)
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