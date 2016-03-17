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
#include "PrintFeature.h"

namespace cura
{

    
bool FffPolygonGenerator::generateAreas(SliceDataStorage& storage, MeshGroup* meshgroup, TimeKeeper& timeKeeper)
{
    if (CommandSocket::isInstantiated())
        CommandSocket::getInstance()->beginSendSlicedObject();
    
    if (!sliceModel(meshgroup, timeKeeper, storage)) 
    {
        return false;
    }
    
    slices2polygons(storage, timeKeeper);
    
    return true;
}

bool FffPolygonGenerator::sliceModel(MeshGroup* meshgroup, TimeKeeper& timeKeeper, SliceDataStorage& storage) /// slices the model
{
    Progress::messageProgressStage(Progress::Stage::SLICING, &timeKeeper);
    
    storage.model_min = meshgroup->min();
    storage.model_max = meshgroup->max();
    storage.model_size = storage.model_max - storage.model_min;

    log("Slicing model...\n");
    int initial_layer_thickness = meshgroup->getSettingInMicrons("layer_height_0");
    if(initial_layer_thickness <= 0) //Initial layer height of 0 is not allowed. Negative layer height is nonsense.
    {
        logError("Initial layer height %i is disallowed.",initial_layer_thickness);
        return false;
    }
    int layer_thickness = meshgroup->getSettingInMicrons("layer_height");
    if(layer_thickness <= 0) //Layer height of 0 is not allowed. Negative layer height is nonsense.
    {
        logError("Layer height %i is disallowed.",layer_thickness);
        return false;
    }
    if (meshgroup->getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::RAFT) 
    { 
        initial_layer_thickness = layer_thickness; 
    }
    int initial_slice_z = initial_layer_thickness - layer_thickness / 2;
    int layer_count = (storage.model_max.z - initial_slice_z) / layer_thickness + 1;
    if(layer_count <= 0) //Model is shallower than layer_height_0, so not even the first layer is sliced. Return an empty model then.
    {
        Progress::messageProgressStage(Progress::Stage::INSET,&timeKeeper); //Continue directly with the inset stage, which will also immediately stop.
        return true; //This is NOT an error state!
    }

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
            sendPolygons("outline", layer_nr, layer.z, layer.polygonList);
            sendPolygons("openoutline", layer_nr, layer.openPolygonList);
        }
        */
        Progress::messageProgress(Progress::Stage::SLICING, mesh_idx + 1, meshgroup->meshes.size());
    }
    
    log("Layer count: %i\n", layer_count);

    meshgroup->clear();///Clear the mesh face and vertex data, it is no longer needed after this point, and it saves a lot of memory.

    Progress::messageProgressStage(Progress::Stage::PARTS, &timeKeeper);
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

        bool has_raft = meshStorage.getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::RAFT;
        //Add the raft offset to each layer.
        for(unsigned int layer_nr=0; layer_nr<meshStorage.layers.size(); layer_nr++)
        {
            SliceLayer& layer = meshStorage.layers[layer_nr];
            meshStorage.layers[layer_nr].printZ += 
                meshStorage.getSettingInMicrons("layer_height_0")
                - initial_slice_z;
            if (has_raft)
            {
                layer.printZ += 
                    meshStorage.getSettingInMicrons("raft_base_thickness") 
                    + meshStorage.getSettingInMicrons("raft_interface_thickness") 
                    + meshStorage.getSettingAsCount("raft_surface_layers") * getSettingInMicrons("raft_surface_thickness")
                    + meshStorage.getSettingInMicrons("raft_airgap");
            }
    
 
            if (layer.parts.size() > 0 || (mesh.getSettingAsSurfaceMode("magic_mesh_surface_mode") != ESurfaceMode::NORMAL && layer.openPolyLines.size() > 0) )
            {
                meshStorage.layer_nr_max_filled_layer = layer_nr; // last set by the highest non-empty layer
            } 
                
            if (CommandSocket::isInstantiated())
            {
                CommandSocket::getInstance()->sendLayerInfo(layer_nr, layer.printZ, layer_nr == 0? meshStorage.getSettingInMicrons("layer_height_0") : meshStorage.getSettingInMicrons("layer_height"));
            }
        }
        
        Progress::messageProgress(Progress::Stage::PARTS, meshIdx + 1, slicerList.size());
    }
    
    Progress::messageProgressStage(Progress::Stage::INSET, &timeKeeper);
    return true;
}

void FffPolygonGenerator::slices2polygons(SliceDataStorage& storage, TimeKeeper& time_keeper)
{
    size_t total_layers = 0;
    for (SliceMeshStorage& mesh : storage.meshes)
    {
        total_layers = std::max<unsigned int>(total_layers, mesh.layers.size());
    }
    
    //layerparts2HTML(storage, "output/output.html");
    for(unsigned int layer_number = 0; layer_number < total_layers; layer_number++)
    {
        processInsets(storage, layer_number);
        
        Progress::messageProgress(Progress::Stage::INSET, layer_number+1, total_layers);
    }
    
    removeEmptyFirstLayers(storage, getSettingInMicrons("layer_height"), total_layers);
    
    if (total_layers < 1)
    {
        log("Stopping process because there are no layers.\n");
        return;
    }
    
    Progress::messageProgressStage(Progress::Stage::SUPPORT, &time_keeper);  
            
    AreaSupport::generateSupportAreas(storage, total_layers);
    /*
    if (storage.support.generated)
    {
        for (unsigned int layer_idx = 0; layer_idx < total_layers; layer_idx++)
        {
            Polygons& support = storage.support.supportLayers[layer_idx].supportAreas;
            if (CommandSocket::isInstantiated()) 
            {
                CommandSocket::getInstance()->sendPolygons(PrintFeatureType::Infill, layer_idx, support, 100); //getSettingInMicrons("support_line_width"));
            }
        }
    }
    */
    
    Progress::messageProgressStage(Progress::Stage::SKIN, &time_keeper);
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
            processSkinsAndInfill(storage, layer_number);
        }
        Progress::messageProgress(Progress::Stage::SKIN, layer_number+1, total_layers);
    }
    
    for(SliceMeshStorage& mesh : storage.meshes)
    {
        unsigned int combined_infill_layers = mesh.getSettingInMicrons("infill_sparse_thickness") / std::max(mesh.getSettingInMicrons("layer_height"), 1); //How many infill layers to combine to obtain the requested sparse thickness.
        combineInfillLayers(mesh,combined_infill_layers);
    }

    storage.primeTower.computePrimeTowerMax(storage);
    storage.primeTower.generatePaths(storage, total_layers);
    
    processOozeShield(storage, total_layers);
        
    processDraftShield(storage, total_layers);
    
    processPlatformAdhesion(storage);

    
    for(SliceMeshStorage& mesh : storage.meshes)
    {
        if (mesh.getSettingBoolean("magic_fuzzy_skin_enabled"))
        {
            processFuzzyWalls(mesh);
        }
    }
}

void FffPolygonGenerator::processInsets(SliceDataStorage& storage, unsigned int layer_nr) 
{
    for(SliceMeshStorage& mesh : storage.meshes)
    {
        SliceLayer* layer = &mesh.layers[layer_nr];
        if (mesh.getSettingAsSurfaceMode("magic_mesh_surface_mode") != ESurfaceMode::SURFACE)
        {
            int inset_count = mesh.getSettingAsCount("wall_line_count");
            if (mesh.getSettingBoolean("magic_spiralize") && static_cast<int>(layer_nr) < mesh.getSettingAsCount("bottom_layers") && layer_nr % 2 == 1)//Add extra insets every 2 layers when spiralizing, this makes bottoms of cups watertight.
                inset_count += 5;
            int line_width_x = mesh.getSettingInMicrons("wall_line_width_x");
            int line_width_0 = mesh.getSettingInMicrons("wall_line_width_0");
            if (mesh.getSettingBoolean("alternate_extra_perimeter"))
                inset_count += layer_nr % 2; 
            generateInsets(layer, mesh.getSettingInMicrons("wall_0_inset"), line_width_0, line_width_x, inset_count, mesh.getSettingBoolean("remove_overlapping_walls_0_enabled"), mesh.getSettingBoolean("remove_overlapping_walls_x_enabled"));
        }
        if (mesh.getSettingAsSurfaceMode("magic_mesh_surface_mode") != ESurfaceMode::NORMAL)
        {
            for (PolygonRef polyline : layer->openPolyLines)
            {
                Polygons segments;
                for (unsigned int point_idx = 1; point_idx < polyline.size(); point_idx++)
                {
                    PolygonRef segment = segments.newPoly();
                    segment.add(polyline[point_idx-1]);
                    segment.add(polyline[point_idx]);
                }
            }
        }
    }
}

void FffPolygonGenerator::removeEmptyFirstLayers(SliceDataStorage& storage, int layer_height, unsigned int total_layers)
{ 
    int n_empty_first_layers = 0;
    for (unsigned int layer_idx = 0; layer_idx < total_layers; layer_idx++)
    { 
        bool layer_is_empty = true;
        for (SliceMeshStorage& mesh : storage.meshes)
        {
            SliceLayer& layer = mesh.layers[layer_idx];
            if (layer.parts.size() > 0 || (mesh.getSettingAsSurfaceMode("magic_mesh_surface_mode") != ESurfaceMode::NORMAL && layer.openPolyLines.size() > 0) )
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
        total_layers -= n_empty_first_layers;
    }
}
  
void FffPolygonGenerator::processSkinsAndInfill(SliceDataStorage& storage, unsigned int layer_nr) 
{
    for(SliceMeshStorage& mesh : storage.meshes)
    {
        if (mesh.getSettingAsSurfaceMode("magic_mesh_surface_mode") == ESurfaceMode::SURFACE) { continue; }
        
        int wall_line_count = mesh.getSettingAsCount("wall_line_count");
        int skin_extrusion_width = mesh.getSettingInMicrons("skin_line_width");
        int innermost_wall_extrusion_width = (wall_line_count == 1)? mesh.getSettingInMicrons("wall_line_width_0") : mesh.getSettingInMicrons("wall_line_width_x");
        generateSkins(layer_nr, mesh, skin_extrusion_width, mesh.getSettingAsCount("bottom_layers"), mesh.getSettingAsCount("top_layers"), wall_line_count, innermost_wall_extrusion_width, mesh.getSettingAsCount("skin_outline_count"), mesh.getSettingBoolean("skin_no_small_gaps_heuristic"), mesh.getSettingBoolean("remove_overlapping_walls_0_enabled"), mesh.getSettingBoolean("remove_overlapping_walls_x_enabled"));
        if (mesh.getSettingInMicrons("infill_line_distance") > 0)
        {
            int infill_skin_overlap = 0;
            bool infill_is_dense = mesh.getSettingInMicrons("infill_line_distance") < mesh.getSettingInMicrons("infill_line_width") + 10;
            if (!infill_is_dense && mesh.getSettingAsFillMethod("infill_pattern") != EFillMethod::CONCENTRIC)
            {
                infill_skin_overlap = skin_extrusion_width / 2;
            }
            generateInfill(layer_nr, mesh, innermost_wall_extrusion_width, infill_skin_overlap, wall_line_count);
            if (mesh.getSettingAsFillPerimeterGapMode("fill_perimeter_gaps") == FillPerimeterGapMode::SKIN)
            {
                generatePerimeterGaps(layer_nr, mesh, skin_extrusion_width, mesh.getSettingAsCount("bottom_layers"), mesh.getSettingAsCount("top_layers"));
            }
            else if (mesh.getSettingAsFillPerimeterGapMode("fill_perimeter_gaps") == FillPerimeterGapMode::EVERYWHERE)
            {
                generatePerimeterGaps(layer_nr, mesh, skin_extrusion_width, 0, 0);
            }
        }
    }
}

void FffPolygonGenerator::processOozeShield(SliceDataStorage& storage, unsigned int total_layers)
{
    if (!getSettingBoolean("ooze_shield_enabled"))
    {
        return;
    }
    
    int ooze_shield_dist = getSettingInMicrons("ooze_shield_dist");
    
    for(unsigned int layer_nr=0; layer_nr<total_layers; layer_nr++)
    {
        storage.oozeShield.push_back(storage.getLayerOutlines(layer_nr, true).offset(ooze_shield_dist));
    }
    
    int largest_printed_radius = MM2INT(1.0); // TODO: make var a parameter, and perhaps even a setting?
    for(unsigned int layer_nr=0; layer_nr<total_layers; layer_nr++)
    {
        storage.oozeShield[layer_nr] = storage.oozeShield[layer_nr].offset(-largest_printed_radius).offset(largest_printed_radius); 
    }
    int allowed_angle_offset = tan(getSettingInAngleRadians("ooze_shield_angle")) * getSettingInMicrons("layer_height");//Allow for a 60deg angle in the oozeShield.
    for(unsigned int layer_nr=1; layer_nr<total_layers; layer_nr++)
    {
        storage.oozeShield[layer_nr] = storage.oozeShield[layer_nr].unionPolygons(storage.oozeShield[layer_nr-1].offset(-allowed_angle_offset));
    }
    for(unsigned int layer_nr=total_layers-1; layer_nr>0; layer_nr--)
    {
        storage.oozeShield[layer_nr-1] = storage.oozeShield[layer_nr-1].unionPolygons(storage.oozeShield[layer_nr].offset(-allowed_angle_offset));
    }
}

void FffPolygonGenerator::processDraftShield(SliceDataStorage& storage, unsigned int total_layers)
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
    for (unsigned int layer_nr = 0; layer_nr < total_layers && layer_nr < max_screen_layer; layer_nr += layer_skip)
    {
        draft_shield = draft_shield.unionPolygons(storage.getLayerOutlines(layer_nr, true));
    }
    
    storage.draft_protection_shield = draft_shield.convexHull(draft_shield_dist);
}

void FffPolygonGenerator::processPlatformAdhesion(SliceDataStorage& storage)
{
    switch(getSettingAsPlatformAdhesion("adhesion_type"))
    {
    case EPlatformAdhesion::SKIRT:
        if (getSettingInMicrons("draft_shield_height") == 0)
        { // draft screen replaces skirt
            generateSkirt(storage, getSettingInMicrons("skirt_gap"), getSettingAsCount("skirt_line_count"), getSettingInMicrons("skirt_minimal_length"));
        }
        break;
    case EPlatformAdhesion::BRIM:
        generateSkirt(storage, 0, getSettingAsCount("brim_line_count"), getSettingInMicrons("skirt_minimal_length"));
        break;
    case EPlatformAdhesion::RAFT:
        generateRaft(storage, getSettingInMicrons("raft_margin"));
        break;
    }
    
    Polygons skirt_sent = storage.skirt[0];
    for (int extruder = 1; extruder < storage.meshgroup->getExtruderCount(); extruder++)
        skirt_sent.add(storage.skirt[extruder]);
}


void FffPolygonGenerator::processFuzzyWalls(SliceMeshStorage& mesh)
{
    if (mesh.getSettingAsCount("wall_line_count") == 0)
    {
        return;
    }
    int64_t fuzziness = mesh.getSettingInMicrons("magic_fuzzy_skin_thickness");
    int64_t avg_dist_between_points = mesh.getSettingInMicrons("magic_fuzzy_skin_point_dist");
    int64_t min_dist_between_points = avg_dist_between_points * 3 / 4; // hardcoded: the point distance may vary between 3/4 and 5/4 the supplied value
    int64_t range_random_point_dist = avg_dist_between_points / 2;
    for (unsigned int layer_nr = 0; layer_nr < mesh.layers.size(); layer_nr++)
    {
        SliceLayer& layer = mesh.layers[layer_nr];
        for (SliceLayerPart& part : layer.parts)
        {
            Polygons results;
            Polygons& skin = (mesh.getSettingAsSurfaceMode("magic_mesh_surface_mode") == ESurfaceMode::SURFACE)? part.outline : part.insets[0];
            for (PolygonRef poly : skin)
            {
                // generate points in between p0 and p1
                PolygonRef result = results.newPoly();
                
                int64_t dist_left_over = rand() % (min_dist_between_points / 2); // the distance to be traversed on the line before making the first new point
                Point* p0 = &poly.back();
                for (Point& p1 : poly)
                { // 'a' is the (next) new point between p0 and p1
                    Point p0p1 = p1 - *p0;
                    int64_t p0p1_size = vSize(p0p1);    
                    int64_t dist_last_point = dist_left_over + p0p1_size * 2; // so that p0p1_size - dist_last_point evaulates to dist_left_over - p0p1_size
                    for (int64_t p0pa_dist = dist_left_over; p0pa_dist < p0p1_size; p0pa_dist += min_dist_between_points + rand() % range_random_point_dist)
                    {
                        int r = rand() % (fuzziness * 2) - fuzziness;
                        Point perp_to_p0p1 = turn90CCW(p0p1);
                        Point fuzz = normal(perp_to_p0p1, r);
                        Point pa = *p0 + normal(p0p1, p0pa_dist) + fuzz;
                        result.add(pa);
                        dist_last_point = p0pa_dist;
                    }
                    dist_left_over = p0p1_size - dist_last_point;
                    
                    p0 = &p1;
                }
                while (result.size() < 3 )
                {
                    unsigned int point_idx = poly.size() - 2;
                    result.add(poly[point_idx]);
                    if (point_idx == 0) { break; }
                    point_idx--;
                }
                if (result.size() < 3)
                {
                    result.clear();
                    for (Point& p : poly)
                        result.add(p);
                }
            }
            skin = results;
        }
    }
}


}//namespace cura
