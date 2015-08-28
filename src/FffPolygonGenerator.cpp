#include "FffPolygonGenerator.h"

#include <algorithm>

#include "slicer.h"
#include "utils/gettime.h"
#include "utils/logoutput.h"
#include "MeshGroup.h"
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

    
bool FffPolygonGenerator::generateAreas(SliceDataStorage& storage, MeshGroup* meshgroup, TimeKeeper& timeKeeper)
{
    if (commandSocket)
        commandSocket->beginSendSlicedObject();
    
    if (!sliceModel(meshgroup, timeKeeper, storage)) 
    {
        return false;
    }
    
    if (getSettingBoolean("magic_mesh_surface_mode"))
    {
        slices2polygons_meshSurfaceMode(storage, timeKeeper);
    }
    else
    {
        slices2polygons(storage, timeKeeper);
    }
    
    return true;
}

bool FffPolygonGenerator::sliceModel(MeshGroup* meshgroup, TimeKeeper& timeKeeper, SliceDataStorage& storage) /// slices the model
{
    Progress::messageProgressStage(Progress::Stage::SLICING, &timeKeeper, commandSocket);
    
    storage.model_min = meshgroup->min();
    storage.model_max = meshgroup->max();
    storage.model_size = storage.model_max - storage.model_min;

    log("Slicing model...\n");
    int initial_layer_thickness = meshgroup->getSettingInMicrons("layer_height_0");
    int layer_thickness = meshgroup->getSettingInMicrons("layer_height");
    if (meshgroup->getSettingAsPlatformAdhesion("adhesion_type") == Adhesion_Raft) 
    { 
        initial_layer_thickness = layer_thickness; 
    }
    int initial_slice_z = initial_layer_thickness - layer_thickness / 2;
    int layer_count = (storage.model_max.z - initial_slice_z) / layer_thickness + 1;

    std::vector<Slicer*> slicerList;
    for(unsigned int mesh_idx = 0; mesh_idx < meshgroup->meshes.size(); mesh_idx++)
    {
        Mesh& mesh = meshgroup->meshes[mesh_idx];
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
        Progress::messageProgress(Progress::Stage::SLICING, mesh_idx + 1, meshgroup->meshes.size(), commandSocket);
    }
    
    log("Layer count: %i\n", layer_count);

    meshgroup->clear();///Clear the mesh face and vertex data, it is no longer needed after this point, and it saves a lot of memory.

    Progress::messageProgressStage(Progress::Stage::PARTS, &timeKeeper, commandSocket);
    //carveMultipleVolumes(storage.meshes);
    generateMultipleVolumesOverlap(slicerList, getSettingInMicrons("multiple_mesh_overlap"));
    
    storage.meshes.reserve(slicerList.size()); // causes there to be no resize in meshes so that the pointers in sliceMeshStorage._config to retraction_config don't get invalidated.
    for(unsigned int meshIdx=0; meshIdx < slicerList.size(); meshIdx++)
    {
        storage.meshes.emplace_back(&meshgroup->meshes[meshIdx]); // new mesh in storage had settings from the Mesh
        SliceMeshStorage& meshStorage = storage.meshes.back();
        Mesh& mesh = storage.meshgroup->meshes[meshIdx];
        createLayerParts(meshStorage, slicerList[meshIdx], mesh.getSettingBoolean("meshfix_union_all"), mesh.getSettingBoolean("meshfix_union_all_remove_holes"));
        delete slicerList[meshIdx];

        bool has_raft = meshStorage.getSettingAsPlatformAdhesion("adhesion_type") == Adhesion_Raft;
        //Add the raft offset to each layer.
        for(unsigned int layer_nr=0; layer_nr<meshStorage.layers.size(); layer_nr++)
        {
            SliceLayer& layer = meshStorage.layers[layer_nr];
            if (has_raft)
            {
                layer.printZ += 
                    meshStorage.getSettingInMicrons("raft_base_thickness") 
                    + meshStorage.getSettingInMicrons("raft_interface_thickness") 
                    + meshStorage.getSettingAsCount("raft_surface_layers") * getSettingInMicrons("layer_height") //raft_surface_thickness") 
                    + meshStorage.getSettingInMicrons("raft_airgap")
                    - initial_slice_z;
            }
            else 
            {
                meshStorage.layers[layer_nr].printZ += 
                    meshStorage.getSettingInMicrons("layer_height_0")
                    - initial_slice_z;
            }
    
 
            if (layer.parts.size() > 0 || (mesh.getSettingBoolean("magic_mesh_surface_mode") && layer.openPolyLines.size() > 0) )
            {
                meshStorage.layer_nr_max_filled_layer = layer_nr; // last set by the highest non-empty layer
            } 
                
            if (commandSocket)
            {
                commandSocket->sendLayerInfo(layer_nr, layer.printZ, layer_nr == 0 && !has_raft? meshStorage.getSettingInMicrons("layer_height_0") : meshStorage.getSettingInMicrons("layer_height"));
            }
        }
        
        Progress::messageProgress(Progress::Stage::PARTS, meshIdx + 1, slicerList.size(), commandSocket);
    }
    
    Progress::messageProgressStage(Progress::Stage::INSET, &timeKeeper, commandSocket);
    return true;
}

void FffPolygonGenerator::slices2polygons(SliceDataStorage& storage, TimeKeeper& time_keeper)
{
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
        if (storage.support.generated)
        {
            for (unsigned int layer_idx = 0; layer_idx < total_layers; layer_idx++)
            {
                Polygons& support = storage.support.supportLayers[layer_idx].supportAreas;
                sendPolygons(SupportType, layer_idx, support, getSettingInMicrons("support_line_width"));
            }
        }
    }
    if (getSettingBoolean("support_roof_enable"))
    {
        generateSupportRoofs(storage, commandSocket, getSettingInMicrons("layer_height"), getSettingInMicrons("support_roof_height"));
    }
    
    Progress::messageProgressStage(Progress::Stage::SKIN, &time_keeper, commandSocket);
    int mesh_max_bottom_layer_count = 0;
    if (getSettingBoolean("magic_spiralize"))
    {
        for(SliceMeshStorage& mesh : storage.meshes)
        {
            mesh_max_bottom_layer_count = std::max(mesh_max_bottom_layer_count, mesh.getSettingAsCount("bottom_layers"));
        }
    }
    for(unsigned int layer_number = 0; layer_number < total_layers; layer_number++)
    {
        if (!getSettingBoolean("magic_spiralize") || static_cast<int>(layer_number) < mesh_max_bottom_layer_count)    //Only generate up/downskin and infill for the first X layers when spiralize is choosen.
        {
            processSkins(storage, layer_number);
        }
        Progress::messageProgress(Progress::Stage::SKIN, layer_number+1, total_layers, commandSocket);
    }
    
    for(unsigned int layer_number = total_layers-1; layer_number > 0; layer_number--)
    {
        for(SliceMeshStorage& mesh : storage.meshes)
            combineInfillLayers(layer_number, mesh, mesh.getSettingAsCount("infill_sparse_combine"));
    }

    storage.primeTower.computePrimeTowerMax(storage);
    storage.primeTower.generatePaths(storage, total_layers);
    
    processDraftShield(storage, total_layers);
    
    processPlatformAdhesion(storage);

}

void FffPolygonGenerator::processInsets(SliceDataStorage& storage, unsigned int layer_nr) 
{
    for(SliceMeshStorage& mesh : storage.meshes)
    {
        int inset_count = mesh.getSettingAsCount("wall_line_count");
        if (mesh.getSettingBoolean("magic_spiralize") && static_cast<int>(layer_nr) < mesh.getSettingAsCount("bottom_layers") && layer_nr % 2 == 1)//Add extra insets every 2 layers when spiralizing, this makes bottoms of cups watertight.
            inset_count += 5;
        SliceLayer* layer = &mesh.layers[layer_nr];
        int line_width_x = mesh.getSettingInMicrons("wall_line_width_x");
        int line_width_0 = mesh.getSettingInMicrons("wall_line_width_0");
        if (mesh.getSettingBoolean("alternate_extra_perimeter"))
            inset_count += layer_nr % 2; 
        generateInsets(layer, mesh.getSettingInMicrons("machine_nozzle_size"), line_width_0, line_width_x, inset_count, mesh.getSettingBoolean("remove_overlapping_walls_0_enabled"), mesh.getSettingBoolean("remove_overlapping_walls_x_enabled"));

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
            SliceLayer& layer = mesh.layers[layer_idx];
            if (layer.parts.size() > 0 || (mesh.getSettingBoolean("magic_mesh_surface_mode") && layer.openPolyLines.size() > 0) )
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
        int extrusionWidth = mesh.getSettingInMicrons("wall_line_width_x");
        int extrusionWidth_infill = mesh.getSettingInMicrons("infill_line_width");
        generateSkins(layer_nr, mesh, extrusionWidth, mesh.getSettingAsCount("bottom_layers"), mesh.getSettingAsCount("top_layers"), mesh.getSettingAsCount("skin_outline_count"), mesh.getSettingBoolean("remove_overlapping_walls_0_enabled"), mesh.getSettingBoolean("remove_overlapping_walls_x_enabled"));
        if (mesh.getSettingInMicrons("infill_line_distance") > 0)
        {
            int infill_skin_overlap = 0;
            if (mesh.getSettingInMicrons("infill_line_distance") > mesh.getSettingInMicrons("infill_line_width") + 10)
            {
                infill_skin_overlap = extrusionWidth / 2;
            }
            generateInfill(layer_nr, mesh, extrusionWidth_infill, infill_skin_overlap);
            if (mesh.getSettingString("fill_perimeter_gaps") == "Skin")
            {
                generatePerimeterGaps(layer_nr, mesh, extrusionWidth, mesh.getSettingAsCount("bottom_layers"), mesh.getSettingAsCount("top_layers"));
            }
            else if (mesh.getSettingString("fill_perimeter_gaps") == "Everywhere")
            {
                generatePerimeterGaps(layer_nr, mesh, extrusionWidth, 0, 0);
            }
        }

        SliceLayer& layer = mesh.layers[layer_nr];
        for(SliceLayerPart& part : layer.parts)
        {
//             sendPolygons(InfillType, layer_nr, part.infill_area[0], extrusionWidth_infill); // sends the outline, not the actual infill
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

void FffPolygonGenerator::processPlatformAdhesion(SliceDataStorage& storage)
{
    switch(getSettingAsPlatformAdhesion("adhesion_type"))
    {
    case Adhesion_Skirt:
        if (getSettingInMicrons("draft_shield_height") == 0)
        { // draft screen replaces skirt
            generateSkirt(storage, getSettingInMicrons("skirt_gap"), getSettingAsCount("skirt_line_count"), getSettingInMicrons("skirt_minimal_length"));
        }
        break;
    case Adhesion_Brim:
        generateSkirt(storage, 0, getSettingAsCount("brim_line_count"), getSettingInMicrons("skirt_minimal_length"));
        break;
    case Adhesion_Raft:
        generateRaft(storage, getSettingInMicrons("raft_margin"));
        break;
    }
    
    Polygons skirt_sent = storage.skirt[0];
    for (int extruder = 1; extruder < storage.meshgroup->getExtruderCount(); extruder++)
        skirt_sent.add(storage.skirt[extruder]);
    sendPolygons(SkirtType, 0, skirt_sent, getSettingInMicrons("skirt_line_width"));
}

void FffPolygonGenerator::slices2polygons_meshSurfaceMode(SliceDataStorage& storage, TimeKeeper& timeKeeper)
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
                sendPolygons(Inset0Type, layer_nr, part.outline, mesh.getSettingInMicrons("wall_line_width_0"));
            }
            for (PolygonRef polyline : layer->openPolyLines)
            {
                for (unsigned int point_idx = 1; point_idx < polyline.size(); point_idx++)
                {
                    Polygon segment;
                    segment.add(polyline[point_idx-1]);
                    segment.add(polyline[point_idx]);
                    sendPolygons(Inset0Type, layer_nr, segment, mesh.getSettingInMicrons("wall_line_width_0"));
                }
            }
        }
    }
}

}//namespace cura
