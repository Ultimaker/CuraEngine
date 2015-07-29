#include "FffPolygonGenerator.h"

#include <algorithm>

#include "slicer.h"
#include "utils/gettime.h"
#include "utils/logoutput.h"
#include "PrintObject.h"
#include "support.h"
#include "multiVolumes.h"
#include "layerPart.h"
#include "inset.h"
#include "skirt.h"
#include "skin.h"
#include "infill.h"
#include "raft.h"
#include "debug.h"
#include "Progress.h"

namespace cura
{

    
bool FffPolygonGenerator::generateAreas(SliceDataStorage& storage, PrintObject* object, TimeKeeper& timeKeeper)
{
    if (!sliceModel(object, timeKeeper, storage)) 
    {
        return false;
    }
    
    if (getSettingBoolean("magic_mesh_surface_mode"))
    {
        slices2polygons_magicPolygonMode(storage, timeKeeper);
    }
    else
    {
        slices2polygons(storage, timeKeeper);
    }
    
    return true;
}

bool FffPolygonGenerator::sliceModel(PrintObject* object, TimeKeeper& timeKeeper, SliceDataStorage& storage) /// slices the model
{
    Progress::messageProgressStage(Progress::Stage::SLICING, &timeKeeper, commandSocket);
    
    storage.model_min = object->min();
    storage.model_max = object->max();
    storage.model_size = storage.model_max - storage.model_min;

    log("Slicing model...\n");
    int initial_layer_thickness = object->getSettingInMicrons("layer_height_0");
    int layer_thickness = object->getSettingInMicrons("layer_height");
    if (object->getSettingAsPlatformAdhesion("adhesion_type") == Adhesion_Raft) 
    { 
        initial_layer_thickness = layer_thickness; 
    }
    int initial_slice_z = initial_layer_thickness - layer_thickness / 2;
    int layer_count = (storage.model_max.z - initial_slice_z) / layer_thickness + 1;

    std::vector<Slicer*> slicerList;
    for(unsigned int mesh_idx = 0; mesh_idx < object->meshes.size(); mesh_idx++)
    {
        Mesh& mesh = object->meshes[mesh_idx];
        Slicer* slicer = new Slicer(&mesh, initial_slice_z, layer_thickness, layer_count, mesh.getSettingBoolean("meshfix_keep_open_polygons"), mesh.getSettingBoolean("meshfix_extensive_stitching"));
        slicerList.push_back(slicer);
        /*
        for(SlicerLayer& layer : slicer->layers)
        {
            //Reporting the outline here slows down the engine quite a bit, so only do so when debugging.
            //sendPolygons("outline", layer_nr, layer.z, layer.polygonList);
            //sendPolygons("openoutline", layer_nr, layer.openPolygonList);
        }
        */
        Progress::messageProgress(Progress::Stage::SLICING, mesh_idx + 1, object->meshes.size(), commandSocket);
    }
    
    log("Layer count: %i\n", layer_count);

    object->clear();///Clear the mesh data, it is no longer needed after this point, and it saves a lot of memory.

    Progress::messageProgressStage(Progress::Stage::PARTS, &timeKeeper, commandSocket);
    //carveMultipleVolumes(storage.meshes);
    generateMultipleVolumesOverlap(slicerList, getSettingInMicrons("multiple_mesh_overlap"));
    
    
    for(unsigned int meshIdx=0; meshIdx < slicerList.size(); meshIdx++)
    {
        storage.meshes.emplace_back(&object->meshes[meshIdx]);
        SliceMeshStorage& meshStorage = storage.meshes[meshIdx];
        createLayerParts(meshStorage, slicerList[meshIdx], meshStorage.settings->getSettingBoolean("meshfix_union_all"), meshStorage.settings->getSettingBoolean("meshfix_union_all_remove_holes"));
        delete slicerList[meshIdx];

        bool has_raft = meshStorage.settings->getSettingAsPlatformAdhesion("adhesion_type") == Adhesion_Raft;
        //Add the raft offset to each layer.
        for(unsigned int layer_nr=0; layer_nr<meshStorage.layers.size(); layer_nr++)
        {
            SliceLayer& layer = meshStorage.layers[layer_nr];
            if (has_raft)
            {
                layer.printZ += 
                    meshStorage.settings->getSettingInMicrons("raft_base_thickness") 
                    + meshStorage.settings->getSettingInMicrons("raft_interface_thickness") 
                    + meshStorage.settings->getSettingAsCount("raft_surface_layers") * getSettingInMicrons("layer_height") //raft_surface_thickness") 
                    + meshStorage.settings->getSettingInMicrons("raft_airgap")
                    - initial_slice_z;
            }
            else 
            {
                meshStorage.layers[layer_nr].printZ += 
                    meshStorage.settings->getSettingInMicrons("layer_height_0")
                    - initial_slice_z;
            }
    
 
            if (layer.parts.size() > 0)
            {
                meshStorage.layer_nr_max_filled_layer = layer_nr; // last set by the highest non-empty layer
            }
                
            if (commandSocket)
                commandSocket->sendLayerInfo(layer_nr, layer.printZ, layer_nr == 0 && !has_raft? meshStorage.settings->getSettingInMicrons("layer_height_0") : meshStorage.settings->getSettingInMicrons("layer_height"));
        }
        
        Progress::messageProgress(Progress::Stage::PARTS, meshIdx + 1, slicerList.size(), commandSocket);
    }
    
    Progress::messageProgressStage(Progress::Stage::INSET, &timeKeeper, commandSocket);
    return true;
}

void FffPolygonGenerator::slices2polygons(SliceDataStorage& storage, TimeKeeper& time_keeper)
{
    if (commandSocket)
        commandSocket->beginSendSlicedObject();
    
    // const 
    unsigned int total_layers = storage.meshes.at(0).layers.size();
    //layerparts2HTML(storage, "output/output.html");
    for(unsigned int layer_number = 0; layer_number < total_layers; layer_number++)
    {
        processInsets(storage, layer_number);
        
        Progress::messageProgress(Progress::Stage::INSET, layer_number+1, total_layers, commandSocket);
    }
    
    removeEmptyFirstLayers(storage, getSettingInMicrons("layer_height"), total_layers);
    
    if (total_layers < 1)
    {
        log("Stopping process because there are no layers.\n");
        return;
    }
        
    processOozeShield(storage, total_layers);
    
    Progress::messageProgressStage(Progress::Stage::SUPPORT, &time_keeper, commandSocket);  
            
    for(SliceMeshStorage& mesh : storage.meshes)
    {
        generateSupportAreas(storage, &mesh, total_layers, commandSocket);
        for (unsigned int layer_idx = 0; layer_idx < total_layers; layer_idx++)
        {
            Polygons& support = storage.support.supportLayers[layer_idx].supportAreas;
            sendPolygons(SupportType, layer_idx, support, getSettingInMicrons("support_line_width"));
        }
    }
    if (getSettingBoolean("support_roof_enable"))
    {
        generateSupportRoofs(storage, commandSocket, getSettingInMicrons("layer_height"), getSettingInMicrons("support_roof_height"));
    }
    
    Progress::messageProgressStage(Progress::Stage::SKIN, &time_keeper, commandSocket);
    for(unsigned int layer_number = 0; layer_number < total_layers; layer_number++)
    {
        if (!getSettingBoolean("magic_spiralize") || static_cast<int>(layer_number) < getSettingAsCount("bottom_layers"))    //Only generate up/downskin and infill for the first X layers when spiralize is choosen.
        {
            processSkins(storage, layer_number);
        }
        Progress::messageProgress(Progress::Stage::SKIN, layer_number+1, total_layers, commandSocket);
    }
    
    for(unsigned int layer_number = total_layers-1; layer_number > 0; layer_number--)
    {
        for(SliceMeshStorage& mesh : storage.meshes)
            combineSparseLayers(layer_number, mesh, mesh.settings->getSettingAsCount("fill_sparse_combine"));
    }

    processPrimeTower(storage, total_layers);
    
    processDraftShield(storage, total_layers);
    
    processPlatformAdhesion(storage);

}

void FffPolygonGenerator::processInsets(SliceDataStorage& storage, unsigned int layer_nr) 
{
    for(SliceMeshStorage& mesh : storage.meshes)
    {
        int inset_count = mesh.settings->getSettingAsCount("wall_line_count");
        if (mesh.settings->getSettingBoolean("magic_spiralize") && static_cast<int>(layer_nr) < mesh.settings->getSettingAsCount("bottom_layers") && layer_nr % 2 == 1)//Add extra insets every 2 layers when spiralizing, this makes bottoms of cups watertight.
            inset_count += 5;
        SliceLayer* layer = &mesh.layers[layer_nr];
        int line_width_x = mesh.settings->getSettingInMicrons("wall_line_width_x");
        int line_width_0 = mesh.settings->getSettingInMicrons("wall_line_width_0");
        if (mesh.settings->getSettingBoolean("alternate_extra_perimeter"))
            inset_count += layer_nr % 2; 
        generateInsets(layer, line_width_0, line_width_x, inset_count, mesh.settings->getSettingBoolean("remove_overlapping_walls_0_enabled"), mesh.settings->getSettingBoolean("remove_overlapping_walls_x_enabled"));

        for(unsigned int partNr=0; partNr<layer->parts.size(); partNr++)
        {
            if (layer->parts[partNr].insets.size() > 0)
            {
                sendPolygons(Inset0Type, layer_nr, layer->parts[partNr].insets[0], line_width_0);
                for(unsigned int inset=1; inset<layer->parts[partNr].insets.size(); inset++)
                    sendPolygons(InsetXType, layer_nr, layer->parts[partNr].insets[inset], line_width_x);
            }
        }
    }
}

void FffPolygonGenerator::removeEmptyFirstLayers(SliceDataStorage& storage, int layer_height, unsigned int totalLayers)
{ 
    int n_empty_first_layers = 0;
    for (unsigned int layer_idx = 0; layer_idx < totalLayers; layer_idx++)
    { 
        bool layer_is_empty = true;
        for (SliceMeshStorage& mesh : storage.meshes)
        {
            if (mesh.layers[layer_idx].parts.size() > 0)
            {
                layer_is_empty = false;
                break;
            }
        }
        
        if (layer_is_empty) 
        {
            n_empty_first_layers++;
        } else
        {
            break;
        }
    }
    
    if (n_empty_first_layers > 0)
    {
        log("Removing %d layers because they are empty\n", n_empty_first_layers);
        for (SliceMeshStorage& mesh : storage.meshes)
        {
            std::vector<SliceLayer>& layers = mesh.layers;
            layers.erase(layers.begin(), layers.begin() + n_empty_first_layers);
            for (SliceLayer& layer : layers)
            {
                layer.printZ -= n_empty_first_layers * layer_height;
            }
        }
        totalLayers -= n_empty_first_layers;
    }
}

void FffPolygonGenerator::processOozeShield(SliceDataStorage& storage, unsigned int totalLayers)
{
    if (!getSettingBoolean("ooze_shield_enabled"))
    {
        return;
    }
    
    int ooze_shield_dist = getSettingInMicrons("ooze_shield_dist");
    
    for(unsigned int layer_nr=0; layer_nr<totalLayers; layer_nr++)
    {
        Polygons oozeShield;
        for(SliceMeshStorage& mesh : storage.meshes)
        {
            for(SliceLayerPart& part : mesh.layers[layer_nr].parts)
            {
                oozeShield = oozeShield.unionPolygons(part.outline.offset(ooze_shield_dist)); 
            }
        }
        storage.oozeShield.push_back(oozeShield);
    }
    
    int largest_printed_radius = MM2INT(1.0); // TODO: make var a parameter, and perhaps even a setting?
    for(unsigned int layer_nr=0; layer_nr<totalLayers; layer_nr++)
        storage.oozeShield[layer_nr] = storage.oozeShield[layer_nr].offset(-largest_printed_radius).offset(largest_printed_radius); 
    int offsetAngle = tan(getSettingInAngleRadians("ooze_shield_angle")) * getSettingInMicrons("layer_height");//Allow for a 60deg angle in the oozeShield.
    for(unsigned int layer_nr=1; layer_nr<totalLayers; layer_nr++)
        storage.oozeShield[layer_nr] = storage.oozeShield[layer_nr].unionPolygons(storage.oozeShield[layer_nr-1].offset(-offsetAngle));
    for(unsigned int layer_nr=totalLayers-1; layer_nr>0; layer_nr--)
        storage.oozeShield[layer_nr-1] = storage.oozeShield[layer_nr-1].unionPolygons(storage.oozeShield[layer_nr].offset(-offsetAngle));
}
  
void FffPolygonGenerator::processSkins(SliceDataStorage& storage, unsigned int layer_nr) 
{
    for(SliceMeshStorage& mesh : storage.meshes)
    {
        int extrusionWidth = mesh.settings->getSettingInMicrons("wall_line_width_x");
        generateSkins(layer_nr, mesh, extrusionWidth, mesh.settings->getSettingAsCount("bottom_layers"), mesh.settings->getSettingAsCount("top_layers"), mesh.settings->getSettingAsCount("skin_outline_count"), mesh.settings->getSettingBoolean("remove_overlapping_walls_0_enabled"), mesh.settings->getSettingBoolean("remove_overlapping_walls_x_enabled"));
        if (mesh.settings->getSettingInMicrons("infill_line_distance") > 0)
        {
            int infill_skin_overlap = 0;
            if (mesh.settings->getSettingInMicrons("infill_line_distance") > mesh.settings->getSettingInMicrons("infill_line_width") + 10)
            {
                infill_skin_overlap = extrusionWidth / 2;
            }
            generateSparse(layer_nr, mesh, extrusionWidth, infill_skin_overlap);
            if (mesh.settings->getSettingString("fill_perimeter_gaps") == "Skin")
            {
                generatePerimeterGaps(layer_nr, mesh, extrusionWidth, mesh.settings->getSettingAsCount("bottom_layers"), mesh.settings->getSettingAsCount("top_layers"));
            }
            else if (mesh.settings->getSettingString("fill_perimeter_gaps") == "Everywhere")
            {
                generatePerimeterGaps(layer_nr, mesh, extrusionWidth, 0, 0);
            }
        }

        SliceLayer& layer = mesh.layers[layer_nr];
        for(SliceLayerPart& part : layer.parts)
        {
            for (SkinPart& skin_part : part.skin_parts)
            {
                sendPolygons(SkinType, layer_nr, skin_part.outline, extrusionWidth);
            }
        }
    }
}

void FffPolygonGenerator::processDraftShield(SliceDataStorage& storage, unsigned int totalLayers)
{
    int draft_shield_height = getSettingInMicrons("draft_shield_height");
    int draft_shield_dist = getSettingInMicrons("draft_shield_dist");
    int layer_height_0 = getSettingInMicrons("layer_height_0");
    int layer_height = getSettingInMicrons("layer_height");
    
    if (draft_shield_height < layer_height_0)
    {
        return;
    }
    
    unsigned int max_screen_layer = (draft_shield_height - layer_height_0) / layer_height + 1;
    
    int layer_skip = 500 / layer_height + 1;
    
    Polygons& draft_shield = storage.draft_protection_shield;
    for (unsigned int layer_nr = 0; layer_nr < totalLayers && layer_nr < max_screen_layer; layer_nr += layer_skip)
    {
        for (SliceMeshStorage& mesh : storage.meshes)
        {
            for (SliceLayerPart& part : mesh.layers[layer_nr].parts)
            {
                draft_shield = draft_shield.unionPolygons(part.outline);
            }
        }
        draft_shield = draft_shield.unionPolygons(storage.support.supportLayers[layer_nr].supportAreas);
//         draft_shield = draft_shield.unionPolygons(storage.support.supportLayers[layer_nr].roofs); // will already be included by supportAreas below, and if the roof is at a low level, the screen will most likely avoid it at a higher level
    }
    
    storage.draft_protection_shield = draft_shield.convexHull(draft_shield_dist);
}

void FffPolygonGenerator::processPrimeTower(SliceDataStorage& storage, unsigned int totalLayers)
{
    
    int extruder_count = getSettingAsCount("machine_extruder_count");
    // TODO: move this code into its own function?
    { // compute storage.max_object_height_second_to_last_extruder, which is used to determine the highest point in the prime tower
        
        int max_object_height_per_extruder[extruder_count];
        { // compute max_object_height_per_extruder
            memset(max_object_height_per_extruder, -1, sizeof(max_object_height_per_extruder));
            for (SliceMeshStorage& mesh : storage.meshes)
            {
                max_object_height_per_extruder[mesh.settings->getSettingAsIndex("extruder_nr")] = 
                    std::max(   max_object_height_per_extruder[mesh.settings->getSettingAsIndex("extruder_nr")]
                            ,   mesh.layer_nr_max_filled_layer  ); 
            }
            int support_extruder_nr = getSettingAsIndex("support_extruder_nr");
            max_object_height_per_extruder[support_extruder_nr] = 
            std::max(   max_object_height_per_extruder[support_extruder_nr]
                    ,   storage.support.layer_nr_max_filled_layer  ); 
            int support_roof_extruder_nr = getSettingAsIndex("support_roof_extruder_nr");
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
       
        
    if (storage.max_object_height_second_to_last_extruder >= 0 && getSettingInMicrons("prime_tower_distance") > 0 && getSettingInMicrons("prime_tower_size") > 0)
    {
        PolygonRef p = storage.primeTower.newPoly();
        int tower_size = getSettingInMicrons("prime_tower_size");
        int tower_distance = getSettingInMicrons("prime_tower_distance");
        p.add(Point(storage.model_max.x + tower_distance, storage.model_max.y + tower_distance));
        p.add(Point(storage.model_max.x + tower_distance, storage.model_max.y + tower_distance + tower_size));
        p.add(Point(storage.model_max.x + tower_distance - tower_size, storage.model_max.y + tower_distance + tower_size));
        p.add(Point(storage.model_max.x + tower_distance - tower_size, storage.model_max.y + tower_distance));

        storage.wipePoint = Point(storage.model_max.x + tower_distance - tower_size / 2, storage.model_max.y + tower_distance + tower_size / 2);
    }
}

void FffPolygonGenerator::processPlatformAdhesion(SliceDataStorage& storage)
{
    switch(getSettingAsPlatformAdhesion("adhesion_type"))
    {
    case Adhesion_Skirt:
        if (getSettingInMicrons("draft_shield_height") == 0)
        { // draft screen replaces skirt
            generateSkirt(storage, getSettingInMicrons("skirt_gap"), getSettingInMicrons("skirt_line_width"), getSettingAsCount("skirt_line_count"), getSettingInMicrons("skirt_minimal_length"));
        }
        break;
    case Adhesion_Brim:
        generateSkirt(storage, 0, getSettingInMicrons("skirt_line_width"), getSettingAsCount("brim_line_count"), getSettingInMicrons("skirt_minimal_length"));
        break;
    case Adhesion_Raft:
        generateRaft(storage, getSettingInMicrons("raft_margin"));
        break;
    }
    
    sendPolygons(SkirtType, 0, storage.skirt, getSettingInMicrons("skirt_line_width"));
}

void FffPolygonGenerator::slices2polygons_magicPolygonMode(SliceDataStorage& storage, TimeKeeper& timeKeeper)
{
    // const 
    unsigned int totalLayers = storage.meshes[0].layers.size();
    
    for(unsigned int layer_nr=0; layer_nr<totalLayers; layer_nr++)
    {
        for(SliceMeshStorage& mesh : storage.meshes)
        {
            SliceLayer* layer = &mesh.layers[layer_nr];
            for(SliceLayerPart& part : layer->parts)
            {
                sendPolygons(Inset0Type, layer_nr, part.outline, mesh.settings->getSettingInMicrons("wall_line_width_0"));
            }
        }
    }
}

} // namespace cura
