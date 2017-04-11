#include "FffPolygonGenerator.h"

#include <algorithm>
#include <map> // multimap (ordered map allowing duplicate keys)
#include <functional> // function

#include "utils/math.h"
#include "slicer/Slicer.h"
#include "utils/algorithm.h"
#include "utils/gettime.h"
#include "utils/logoutput.h"
#include "MeshGroup.h"
#include "support.h"
#include "slicer/MultiVolumes.h"
#include "slicer/LayerPart.h"
#include "textureProcessing/TextureBumpMapProcessor.h"
#include "textureProcessing/TextureProximityProcessor.h"
#include "textureProcessing/AreaTextureProcessor.h"
#include "WallsComputation.h"
#include "SkirtBrim.h"
#include "skin.h"
#include "infill.h"
#include "raft.h"
#include "progress/Progress.h"
#include "PrintFeature.h"
#include "ConicalOverhang.h"
#include "progress/ProgressEstimator.h"
#include "progress/ProgressStageEstimator.h"
#include "progress/ProgressEstimatorLinear.h"


namespace cura
{


bool FffPolygonGenerator::generateAreas(SliceDataStorage& storage, MeshGroup* meshgroup, TimeKeeper& timeKeeper)
{
    if (!sliceModel(meshgroup, timeKeeper, storage)) 
    {
        return false;
    }
    
    slices2polygons(storage, timeKeeper);
    
    return true;
}

unsigned int FffPolygonGenerator::getDraftShieldLayerCount(const unsigned int total_layers) const
{
    if (!getSettingBoolean("draft_shield_enabled"))
    {
        return 0;
    }
    switch (getSettingAsDraftShieldHeightLimitation("draft_shield_height_limitation"))
    {
        default:
        case DraftShieldHeightLimitation::FULL:
            return total_layers;
        case DraftShieldHeightLimitation::LIMITED:
            return std::max((coord_t)0, (getSettingInMicrons("draft_shield_height") - getSettingInMicrons("layer_height_0")) / getSettingInMicrons("layer_height") + 1);
    }
}

bool FffPolygonGenerator::sliceModel(MeshGroup* meshgroup, TimeKeeper& timeKeeper, SliceDataStorage& storage) /// slices the model
{
    Progress::messageProgressStage(Progress::Stage::SLICING, &timeKeeper);
    
    storage.model_min = meshgroup->min();
    storage.model_max = meshgroup->max();
    storage.model_size = storage.model_max - storage.model_min;

    log("Slicing model...\n");
    int initial_layer_thickness = getSettingInMicrons("layer_height_0");
    if(initial_layer_thickness <= 0) //Initial layer height of 0 is not allowed. Negative layer height is nonsense.
    {
        logError("Initial layer height %i is disallowed.\n", initial_layer_thickness);
        return false;
    }
    int layer_thickness = getSettingInMicrons("layer_height");
    if(layer_thickness <= 0) //Layer height of 0 is not allowed. Negative layer height is nonsense.
    {
        logError("Layer height %i is disallowed.\n", layer_thickness);
        return false;
    }
    int initial_slice_z = initial_layer_thickness - layer_thickness / 2;
    int slice_layer_count = (storage.model_max.z - initial_slice_z) / layer_thickness + 1;
    if (slice_layer_count <= 0) //Model is shallower than layer_height_0, so not even the first layer is sliced. Return an empty model then.
    {
        return true; //This is NOT an error state!
    }

    
    storage.meshes.reserve(meshgroup->meshes.size()); // causes there to be no resize in meshes so that the pointers in sliceMeshStorage._config to retraction_config don't get invalidated.
    for(unsigned int meshIdx=0; meshIdx < meshgroup->meshes.size(); meshIdx++)
    {
        // always make a new SliceMeshStorage, so that they have the same ordering / indexing as meshgroup.meshes
        // even make a mesh for a support mesh, which doesn't introduce any parts.
        storage.meshes.emplace_back(meshgroup->meshes[meshIdx], slice_layer_count); // new mesh in storage had settings from the Mes
    }
    // ^ needs to be set already for fuzzy wall texture map processing

    std::vector<Slicer*> slicerList;
    for(unsigned int mesh_idx = 0; mesh_idx < meshgroup->meshes.size(); mesh_idx++)
    {
        Mesh& mesh = *meshgroup->meshes[mesh_idx];
        if (mesh.getSettingBoolean("fuzz_map_enabled"))
        {
            TextureProximityProcessor::Settings texture_proximity_processor_settings(mesh.getSettingInMicrons("wall_line_width_0"));
            storage.meshes[mesh_idx].texture_proximity_processor = new TextureProximityProcessor(texture_proximity_processor_settings, slice_layer_count);
        }
        bool keep_open_polylines = mesh.getSettingBoolean("meshfix_keep_open_polygons");
        bool extensive_stitching = mesh.getSettingBoolean("meshfix_extensive_stitching");
        Slicer* slicer = new Slicer(&mesh, initial_slice_z, layer_thickness, slice_layer_count, keep_open_polylines, extensive_stitching, storage.meshes[mesh_idx].texture_proximity_processor);
        slicerList.push_back(slicer);
        /*
        for(SlicerLayer& layer : slicer->layers)
        {
            //Reporting the outline here slows down the engine quite a bit, so only do so when debugging.
            sendPolygons("outline", layer_nr, layer.z, layer.polygonList);
            sendPolygons("openoutline", layer_nr, layer.openPolygonList);
        }
        */
        if (mesh.getSettingBoolean("area_texture_mapping"))
        {
            TexturedMesh& textured_mesh = *dynamic_cast<TexturedMesh*>(&mesh);
            AreaTextureProcessor(textured_mesh, slicer->layers.size());
        }
        
        Progress::messageProgress(Progress::Stage::SLICING, mesh_idx + 1, meshgroup->meshes.size());
    }

    meshgroup->clear();///Clear the mesh face and vertex data, it is no longer needed after this point, and it saves a lot of memory.


    for(unsigned int meshIdx=0; meshIdx < slicerList.size(); meshIdx++)
    {
        Mesh& mesh = *storage.meshgroup->meshes[meshIdx];
        if (mesh.getSettingBoolean("conical_overhang_enabled") && !mesh.getSettingBoolean("anti_overhang_mesh"))
        {
            ConicalOverhang::apply(slicerList[meshIdx], mesh.getSettingInAngleRadians("conical_overhang_angle"), layer_thickness);
        }
    }

    Progress::messageProgressStage(Progress::Stage::PARTS, &timeKeeper);


    if (storage.getSettingBoolean("carve_multiple_volumes"))
    {
        carveMultipleVolumes(slicerList, storage.getSettingBoolean("alternate_carve_order"));
    }
    generateMultipleVolumesOverlap(slicerList);

    storage.print_layer_count = 0;
    for (unsigned int meshIdx = 0; meshIdx < slicerList.size(); meshIdx++)
    {
        Mesh& mesh = *storage.meshgroup->meshes[meshIdx];
        Slicer* slicer = slicerList[meshIdx];
        if (!mesh.getSettingBoolean("anti_overhang_mesh") && !mesh.getSettingBoolean("infill_mesh"))
        {
            storage.print_layer_count = std::max(storage.print_layer_count, (unsigned int)slicer->layers.size());
        }
    }
    storage.support.supportLayers.resize(storage.print_layer_count);

    for (unsigned int meshIdx = 0; meshIdx < slicerList.size(); meshIdx++)
    {
        Slicer* slicer = slicerList[meshIdx];
        Mesh& mesh = *storage.meshgroup->meshes[meshIdx];

       SliceMeshStorage& meshStorage = storage.meshes[meshIdx];

        if (mesh.getSettingBoolean("anti_overhang_mesh"))
        {
            for (unsigned int layer_nr = 0; layer_nr < slicer->layers.size(); layer_nr++)
            {
                SupportLayer& support_layer = storage.support.supportLayers[layer_nr];
                SlicerLayer& slicer_layer = slicer->layers[layer_nr];
                support_layer.anti_overhang = support_layer.anti_overhang.unionPolygons(slicer_layer.polygons);
            }
            continue;
        }
        if (mesh.getSettingBoolean("support_mesh"))
        {
            for (unsigned int layer_nr = 0; layer_nr < slicer->layers.size(); layer_nr++)
            {
                SupportLayer& support_layer = storage.support.supportLayers[layer_nr];
                SlicerLayer& slicer_layer = slicer->layers[layer_nr];
                support_layer.support_mesh.add(slicer_layer.polygons);
            }
            continue;
        }

        createLayerParts(meshStorage, slicer, mesh.getSettingBoolean("meshfix_union_all"), mesh.getSettingBoolean("meshfix_union_all_remove_holes"));
        delete slicerList[meshIdx];

        bool has_raft = getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::RAFT;
        //Add the raft offset to each layer.
        for(unsigned int layer_nr=0; layer_nr<meshStorage.layers.size(); layer_nr++)
        {
            SliceLayer& layer = meshStorage.layers[layer_nr];
            meshStorage.layers[layer_nr].printZ += 
                getSettingInMicrons("layer_height_0")
                - initial_slice_z;
            if (has_raft)
            {
                ExtruderTrain* train = storage.meshgroup->getExtruderTrain(getSettingAsIndex("adhesion_extruder_nr"));
                layer.printZ += 
                    Raft::getTotalThickness(storage)
                    + train->getSettingInMicrons("raft_airgap")
                    - train->getSettingInMicrons("layer_0_z_overlap"); // shift all layers (except 0) down
                if (layer_nr == 0)
                {
                    layer.printZ += train->getSettingInMicrons("layer_0_z_overlap"); // undo shifting down of first layer
                }
            }

            if (layer.parts.size() > 0 || (mesh.getSettingAsSurfaceMode("magic_mesh_surface_mode") != ESurfaceMode::NORMAL && layer.openPolyLines.size() > 0) )
            {
                meshStorage.layer_nr_max_filled_layer = layer_nr; // last set by the highest non-empty layer
            } 
        }

        Progress::messageProgress(Progress::Stage::PARTS, meshIdx + 1, slicerList.size());
    }
    return true;
}

void FffPolygonGenerator::slices2polygons(SliceDataStorage& storage, TimeKeeper& time_keeper)
{
    // compute layer count and remove first empty layers
    // there is no separate progress stage for removeEmptyFisrtLayer (TODO)
    unsigned int slice_layer_count = 0;
    for (SliceMeshStorage& mesh : storage.meshes)
    {
        if (!mesh.getSettingBoolean("infill_mesh") && !mesh.getSettingBoolean("anti_overhang_mesh"))
        {
            slice_layer_count = std::max<unsigned int>(slice_layer_count, mesh.layers.size());
        }
    }

    // handle meshes
    std::vector<double> mesh_timings;
    for (unsigned int mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
    {
        mesh_timings.push_back(1.0); // TODO: have a more accurate estimate of the relative time it takes per mesh, based on the height and number of polygons
    }
    ProgressStageEstimator inset_skin_progress_estimate(mesh_timings);

    Progress::messageProgressStage(Progress::Stage::INSET_SKIN, &time_keeper);
    std::vector<unsigned int> mesh_order;
    { // compute mesh order
        std::multimap<int, unsigned int> order_to_mesh_indices;
        for (unsigned int mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
        {
            order_to_mesh_indices.emplace(storage.meshes[mesh_idx].getSettingAsIndex("infill_mesh_order"), mesh_idx);
        }
        for (std::pair<const int, unsigned int>& order_and_mesh_idx : order_to_mesh_indices)
        {
            mesh_order.push_back(order_and_mesh_idx.second);
        }
    }
    for (unsigned int mesh_order_idx(0); mesh_order_idx < mesh_order.size(); ++mesh_order_idx)
    {
        processBasicWallsSkinInfill(storage, mesh_order_idx, mesh_order, inset_skin_progress_estimate);
        Progress::messageProgress(Progress::Stage::INSET_SKIN, mesh_order_idx + 1, storage.meshes.size());
    }

    for (unsigned int layer_nr = 0; layer_nr < slice_layer_count; layer_nr++)
    {
        SliceLayer* layer = nullptr;
        for (unsigned int mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
        { // find first mesh which has this layer
            SliceMeshStorage& mesh = storage.meshes[mesh_idx];
            if (int(layer_nr) <= mesh.layer_nr_max_filled_layer)
            {
                layer = &mesh.layers[layer_nr];
                break;
            }
        }
        if (layer != nullptr)
        {
            if (CommandSocket::isInstantiated())
            { // send layer info
                CommandSocket::getInstance()->sendOptimizedLayerInfo(layer_nr, layer->printZ, layer_nr == 0? getSettingInMicrons("layer_height_0") : getSettingInMicrons("layer_height"));
            }
        }
    }

    log("Layer count: %i\n", storage.print_layer_count);

    //layerparts2HTML(storage, "output/output.html");

    Progress::messageProgressStage(Progress::Stage::SUPPORT, &time_keeper);

    AreaSupport::generateSupportAreas(storage, storage.print_layer_count);

    // we need to remove empty layers after we have procesed the insets
    // processInsets might throw away parts if they have no wall at all (cause it doesn't fit)
    // brim depends on the first layer not being empty
    // only remove empty layers if we haven't generate support, because then support was added underneath the model.
    //   for some materials it's better to print on support than on the buildplate.
    removeEmptyFirstLayers(storage, getSettingInMicrons("layer_height"), storage.print_layer_count); // changes storage.print_layer_count!
    if (storage.print_layer_count == 0)
    {
        log("Stopping process because there are no non-empty layers.\n");
        return;
    }
    
    /*
    if (storage.support.generated)
    {
        for (unsigned int layer_idx = 0; layer_idx < storage.print_layer_count; layer_idx++)
        {
            Polygons& support = storage.support.supportLayers[layer_idx].supportAreas;
            ExtruderTrain* infill_extr = storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("support_infill_extruder_nr"));
            CommandSocket::sendPolygons(PrintFeatureType::Infill, support, 100); // infill_extr->getSettingInMicrons("support_line_width"));
        }
    }
    */

    computePrintHeightStatistics(storage);

    // handle helpers
    storage.primeTower.generatePaths(storage);
    storage.primeTower.subtractFromSupport(storage);
    
    logDebug("Processing ooze shield\n");
    processOozeShield(storage);

    logDebug("Processing draft shield\n");
    processDraftShield(storage);

    logDebug("Processing platform adhesion\n");
    processPlatformAdhesion(storage);
    
    // meshes post processing
    for (SliceMeshStorage& mesh : storage.meshes)
    {
        processDerivedWallsSkinInfill(mesh);
    }
}

void FffPolygonGenerator::processBasicWallsSkinInfill(SliceDataStorage& storage, unsigned int mesh_order_idx, std::vector<unsigned int>& mesh_order, ProgressStageEstimator& inset_skin_progress_estimate)
{
    unsigned int mesh_idx = mesh_order[mesh_order_idx];
    SliceMeshStorage& mesh = storage.meshes[mesh_idx];
    size_t mesh_layer_count = mesh.layers.size();
    if (mesh.getSettingBoolean("infill_mesh"))
    {
        processInfillMesh(storage, mesh_order_idx, mesh_order);
    }
    
    // TODO: make progress more accurate!!
    // note: estimated time for     insets : skins = 22.953 : 48.858
    std::vector<double> walls_vs_skin_timing({22.953, 48.858});
    ProgressStageEstimator* mesh_inset_skin_progress_estimator = new ProgressStageEstimator(walls_vs_skin_timing);
    
    inset_skin_progress_estimate.nextStage(mesh_inset_skin_progress_estimator); // the stage of this function call
    
    ProgressEstimatorLinear* inset_estimator = new ProgressEstimatorLinear(mesh_layer_count);
    mesh_inset_skin_progress_estimator->nextStage(inset_estimator);
    
    
    // walls
    for (unsigned int layer_number = 0; layer_number < mesh.layers.size(); layer_number++)
    {
        logDebug("Processing insets for layer %i of %i\n", layer_number, mesh_layer_count);
        processInsets(mesh, layer_number);
        double progress = inset_skin_progress_estimate.progress(layer_number);
        Progress::messageProgress(Progress::Stage::INSET_SKIN, progress * 100, 100);
    }

    ProgressEstimatorLinear* skin_estimator = new ProgressEstimatorLinear(mesh_layer_count);
    mesh_inset_skin_progress_estimator->nextStage(skin_estimator);

    bool process_infill = mesh.getSettingInMicrons("infill_line_distance") > 0;
    if (!process_infill)
    { // do process infill anyway if it's modified by modifier meshes
        for (unsigned int other_mesh_order_idx(mesh_order_idx + 1); other_mesh_order_idx < mesh_order.size(); ++other_mesh_order_idx)
        {
            unsigned int other_mesh_idx = mesh_order[other_mesh_order_idx];
            SliceMeshStorage& other_mesh = storage.meshes[other_mesh_idx];
            if (other_mesh.getSettingBoolean("infill_mesh"))
            {
                AABB3D aabb = storage.meshgroup->meshes[mesh_idx]->getAABB();
                AABB3D other_aabb = storage.meshgroup->meshes[other_mesh_idx]->getAABB();
                if (aabb.hit(other_aabb))
                {
                    process_infill = true;
                }
            }
        }
    }
    // skin & infill
//     Progress::messageProgressStage(Progress::Stage::SKIN, &time_keeper);
    int mesh_max_bottom_layer_count = 0;
    if (mesh.getSettingBoolean("magic_spiralize"))
    {
        mesh_max_bottom_layer_count = std::max(mesh_max_bottom_layer_count, mesh.getSettingAsCount("bottom_layers"));
    }
    for (unsigned int layer_number = 0; layer_number < mesh.layers.size(); layer_number++)
    {
        logDebug("Processing skins and infill layer %i of %i\n", layer_number, mesh_layer_count);
        if (!mesh.getSettingBoolean("magic_spiralize") || static_cast<int>(layer_number) < mesh_max_bottom_layer_count)    //Only generate up/downskin and infill for the first X layers when spiralize is choosen.
        {
            processSkinsAndInfill(mesh, layer_number, process_infill);
        }
        double progress = inset_skin_progress_estimate.progress(layer_number);
        Progress::messageProgress(Progress::Stage::INSET_SKIN, progress * 100, 100);
    }
}

void FffPolygonGenerator::processInfillMesh(SliceDataStorage& storage, unsigned int mesh_order_idx, std::vector<unsigned int>& mesh_order)
{
    unsigned int mesh_idx = mesh_order[mesh_order_idx];
    SliceMeshStorage& mesh = storage.meshes[mesh_idx];
    mesh.layer_nr_max_filled_layer = -1;
    for (unsigned int layer_idx = 0; layer_idx < mesh.layers.size(); layer_idx++)
    {
        SliceLayer& layer = mesh.layers[layer_idx];
        std::vector<PolygonsPart> new_parts;

        for (unsigned int other_mesh_idx : mesh_order)
        { // limit the infill mesh's outline to within the infill of all meshes with lower order
            if (other_mesh_idx == mesh_idx)
            {
                break; // all previous meshes have been processed
            }
            SliceMeshStorage& other_mesh = storage.meshes[other_mesh_idx];
            if (layer_idx >= other_mesh.layers.size())
            { // there can be no interaction between the infill mesh and this other non-infill mesh
                continue;
            }

            SliceLayer& other_layer = other_mesh.layers[layer_idx];

            for (SliceLayerPart& part : layer.parts)
            {
                for (SliceLayerPart& other_part : other_layer.parts)
                { // limit the outline of each part of this infill mesh to the infill of parts of the other mesh with lower infill mesh order
                    if (!part.boundaryBox.hit(other_part.boundaryBox))
                    { // early out
                        continue;
                    }
                    Polygons new_outline = part.outline.intersection(other_part.getOwnInfillArea());
                    if (new_outline.size() == 1)
                    { // we don't have to call splitIntoParts, because a single polygon can only be a single part
                        PolygonsPart outline_part_here;
                        outline_part_here.add(new_outline[0]);
                        new_parts.push_back(outline_part_here);
                    }
                    else if (new_outline.size() > 1)
                    { // we don't know whether it's a multitude of parts because of newly introduced holes, or because the polygon has been split up
                        std::vector<PolygonsPart> new_parts_here = new_outline.splitIntoParts();
                        for (PolygonsPart& new_part_here : new_parts_here)
                        {
                            new_parts.push_back(new_part_here);
                        }
                    }
                    // change the infill area of the non-infill mesh which is to be filled with e.g. lines
                    other_part.infill_area_own = other_part.getOwnInfillArea().difference(part.outline);
                    // note: don't change the part.infill_area, because we change the structure of that area, while the basic area in which infill is printed remains the same
                    //       the infill area remains the same for combing
                }
            }
        }
        
        layer.parts.clear();
        for (PolygonsPart& part : new_parts)
        {
            layer.parts.emplace_back();
            layer.parts.back().outline = part;
            layer.parts.back().boundaryBox.calculate(part);
        }

        if (layer.parts.size() > 0 || (mesh.getSettingAsSurfaceMode("magic_mesh_surface_mode") != ESurfaceMode::NORMAL && layer.openPolyLines.size() > 0) )
        {
            mesh.layer_nr_max_filled_layer = layer_idx; // last set by the highest non-empty layer
        } 
    }
    
}

void FffPolygonGenerator::processDerivedWallsSkinInfill(SliceMeshStorage& mesh)
{
    // create gradual infill areas
    SkinInfillAreaComputation::generateGradualInfill(mesh, mesh.getSettingInMicrons("gradual_infill_step_height"), mesh.getSettingAsCount("gradual_infill_steps"));

    //SubDivCube Pre-compute Octree
    if (mesh.getSettingAsFillMethod("infill_pattern") == EFillMethod::CUBICSUBDIV)
    {
        SubDivCube::precomputeOctree(mesh);
    }

    // combine infill
    unsigned int combined_infill_layers = std::max(1U, round_divide(mesh.getSettingInMicrons("infill_sparse_thickness"), std::max(getSettingInMicrons("layer_height"), (coord_t)1))); //How many infill layers to combine to obtain the requested sparse thickness.
    combineInfillLayers(mesh,combined_infill_layers);
}

void FffPolygonGenerator::processInsets(SliceMeshStorage& mesh, unsigned int layer_nr) 
{
    SliceLayer* layer = &mesh.layers[layer_nr];
    if (mesh.getSettingAsSurfaceMode("magic_mesh_surface_mode") != ESurfaceMode::SURFACE)
    {
        int inset_count = mesh.getSettingAsCount("wall_line_count");
        if (mesh.getSettingBoolean("magic_spiralize") && static_cast<int>(layer_nr) < mesh.getSettingAsCount("bottom_layers") && ((layer_nr % 2) + 2) % 2 == 1)//Add extra insets every 2 layers when spiralizing, this makes bottoms of cups watertight.
            inset_count += 5;
        int line_width_x = mesh.getSettingInMicrons("wall_line_width_x");
        int line_width_0 = mesh.getSettingInMicrons("wall_line_width_0");
        if (mesh.getSettingBoolean("alternate_extra_perimeter"))
        {
            inset_count += ((layer_nr % 2) + 2) % 2;
        }
        bool recompute_outline_based_on_outer_wall = mesh.getSettingBoolean("support_enable");
        WallsComputation walls_computation(mesh.getSettingInMicrons("wall_0_inset"), line_width_0, line_width_x, inset_count, recompute_outline_based_on_outer_wall);
        walls_computation.generateInsets(layer);
    }
}

void FffPolygonGenerator::removeEmptyFirstLayers(SliceDataStorage& storage, const int layer_height, unsigned int& total_layers)
{
    int n_empty_first_layers = 0;
    for (unsigned int layer_idx = 0; layer_idx < total_layers; layer_idx++)
    { 
        bool layer_is_empty = true;
        if (storage.support.generated && layer_idx < storage.support.supportLayers.size())
        {
            SupportLayer& support_layer = storage.support.supportLayers[layer_idx];
            if (support_layer.supportAreas.size() > 0 || support_layer.skin.size() > 0)
            {
                layer_is_empty = false;
                break;
            }
        }
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
            mesh.layer_nr_max_filled_layer -= n_empty_first_layers;
        }
        total_layers -= n_empty_first_layers;
        storage.support.layer_nr_max_filled_layer -= n_empty_first_layers;
        std::vector<SupportLayer>& support_layers = storage.support.supportLayers;
        support_layers.erase(support_layers.begin(), support_layers.begin() + n_empty_first_layers);
    }
}
  
void FffPolygonGenerator::processSkinsAndInfill(SliceMeshStorage& mesh, unsigned int layer_nr, bool process_infill) 
{
    if (mesh.getSettingAsSurfaceMode("magic_mesh_surface_mode") == ESurfaceMode::SURFACE) 
    { 
        return;
    }

    const int wall_line_count = mesh.getSettingAsCount("wall_line_count");
    const int innermost_wall_line_width = (wall_line_count == 1) ? mesh.getSettingInMicrons("wall_line_width_0") : mesh.getSettingInMicrons("wall_line_width_x");
    generateSkins(layer_nr, mesh, mesh.getSettingAsCount("bottom_layers"), mesh.getSettingAsCount("top_layers"), wall_line_count, innermost_wall_line_width, mesh.getSettingAsCount("skin_outline_count"), mesh.getSettingBoolean("skin_no_small_gaps_heuristic"));

    if (process_infill)
    { // process infill when infill density > 0
        // or when other infill meshes want to modify this infill
        int infill_skin_overlap = 0;
        bool infill_is_dense = mesh.getSettingInMicrons("infill_line_distance") < mesh.getSettingInMicrons("infill_line_width") + 10;
        if (!infill_is_dense && mesh.getSettingAsFillMethod("infill_pattern") != EFillMethod::CONCENTRIC)
        {
            infill_skin_overlap = innermost_wall_line_width / 2;
        }
        generateInfill(layer_nr, mesh, innermost_wall_line_width, infill_skin_overlap, wall_line_count);
    }
}

void FffPolygonGenerator::computePrintHeightStatistics(SliceDataStorage& storage)
{
    int extruder_count = storage.meshgroup->getExtruderCount();

    std::vector<int>& max_print_height_per_extruder = storage.max_print_height_per_extruder;
    assert(max_print_height_per_extruder.size() == 0 && "storage.max_print_height_per_extruder shouldn't have been initialized yet!");
    max_print_height_per_extruder.resize(extruder_count, -1); //Initialize all as -1.
    { // compute max_object_height_per_extruder
        //Height of the meshes themselves.
        for (SliceMeshStorage& mesh : storage.meshes)
        {
            if (mesh.getSettingBoolean("anti_overhang_mesh") || mesh.getSettingBoolean("support_mesh"))
            {
                continue; //Special type of mesh that doesn't get printed.
            }
            const unsigned int extr_nr = mesh.getSettingAsIndex("extruder_nr");
            max_print_height_per_extruder[extr_nr] = std::max(max_print_height_per_extruder[extr_nr], mesh.layer_nr_max_filled_layer);
        }

        //Height of where the support reaches.
        const unsigned int support_infill_extruder_nr = storage.getSettingAsIndex("support_infill_extruder_nr"); // TODO: support extruder should be configurable per object
        max_print_height_per_extruder[support_infill_extruder_nr] =
            std::max(max_print_height_per_extruder[support_infill_extruder_nr],
                     storage.support.layer_nr_max_filled_layer);
        const unsigned int support_skin_extruder_nr = storage.getSettingAsIndex("support_interface_extruder_nr"); // TODO: support skin extruder should be configurable per object
        max_print_height_per_extruder[support_skin_extruder_nr] =
            std::max(max_print_height_per_extruder[support_skin_extruder_nr],
                     storage.support.layer_nr_max_filled_layer);

        //Height of where the platform adhesion reaches.
        if (storage.getSettingAsPlatformAdhesion("adhesion_type") != EPlatformAdhesion::NONE)
        {
            const unsigned int adhesion_extruder_nr = storage.getSettingAsIndex("adhesion_extruder_nr");
            max_print_height_per_extruder[adhesion_extruder_nr] =
                std::max(0, max_print_height_per_extruder[adhesion_extruder_nr]);
        }
    }

    storage.max_print_height_order = order(max_print_height_per_extruder);
    if (extruder_count >= 2)
    {
        int second_highest_extruder = storage.max_print_height_order[extruder_count - 2];
        storage.max_print_height_second_to_last_extruder = max_print_height_per_extruder[second_highest_extruder];
    }
    else
    {
        storage.max_print_height_second_to_last_extruder = -1;
    }
}


void FffPolygonGenerator::processOozeShield(SliceDataStorage& storage)
{
    if (!getSettingBoolean("ooze_shield_enabled"))
    {
        return;
    }

    const int ooze_shield_dist = getSettingInMicrons("ooze_shield_dist");

    for (int layer_nr = 0; layer_nr <= storage.max_print_height_second_to_last_extruder; layer_nr++)
    {
        storage.oozeShield.push_back(storage.getLayerOutlines(layer_nr, true).offset(ooze_shield_dist, ClipperLib::jtRound));
    }

    double angle = getSettingInAngleDegrees("ooze_shield_angle");
    if (angle <= 89)
    {
        int allowed_angle_offset = tan(getSettingInAngleRadians("ooze_shield_angle")) * getSettingInMicrons("layer_height"); // Allow for a 60deg angle in the oozeShield.
        for (int layer_nr = 1; layer_nr <= storage.max_print_height_second_to_last_extruder; layer_nr++)
        {
            storage.oozeShield[layer_nr] = storage.oozeShield[layer_nr].unionPolygons(storage.oozeShield[layer_nr - 1].offset(-allowed_angle_offset));
        }
        for (int layer_nr = storage.max_print_height_second_to_last_extruder; layer_nr > 0; layer_nr--)
        {
            storage.oozeShield[layer_nr - 1] = storage.oozeShield[layer_nr - 1].unionPolygons(storage.oozeShield[layer_nr].offset(-allowed_angle_offset));
        }
    }

    const float largest_printed_area = 1.0; // TODO: make var a parameter, and perhaps even a setting?
    for (int layer_nr = 0; layer_nr <= storage.max_print_height_second_to_last_extruder; layer_nr++)
    {
        storage.oozeShield[layer_nr].removeSmallAreas(largest_printed_area);
    }
}

void FffPolygonGenerator::processDraftShield(SliceDataStorage& storage)
{
    const unsigned int draft_shield_layers = getDraftShieldLayerCount(storage.print_layer_count);
    if (draft_shield_layers <= 0)
    {
        return;
    }
    const int layer_height = getSettingInMicrons("layer_height");

    const unsigned int layer_skip = 500 / layer_height + 1;

    Polygons& draft_shield = storage.draft_protection_shield;
    for (unsigned int layer_nr = 0; layer_nr < storage.print_layer_count && layer_nr < draft_shield_layers; layer_nr += layer_skip)
    {
        draft_shield = draft_shield.unionPolygons(storage.getLayerOutlines(layer_nr, true));
    }

    const int draft_shield_dist = getSettingInMicrons("draft_shield_dist");
    storage.draft_protection_shield = draft_shield.approxConvexHull(draft_shield_dist);
}

void FffPolygonGenerator::processPlatformAdhesion(SliceDataStorage& storage)
{
    SettingsBaseVirtual* train = storage.meshgroup->getExtruderTrain(getSettingBoolean("adhesion_extruder_nr"));
    switch(getSettingAsPlatformAdhesion("adhesion_type"))
    {
    case EPlatformAdhesion::SKIRT:
        {
            constexpr bool outside_polygons_only = true;
            SkirtBrim::generate(storage, train->getSettingInMicrons("skirt_gap"), train->getSettingAsCount("skirt_line_count"), outside_polygons_only);
        }
        break;
    case EPlatformAdhesion::BRIM:
        SkirtBrim::generate(storage, 0, train->getSettingAsCount("brim_line_count"), train->getSettingBoolean("brim_outside_only"));
        break;
    case EPlatformAdhesion::RAFT:
        Raft::generate(storage, train->getSettingInMicrons("raft_margin"));
        break;
    case EPlatformAdhesion::NONE:
        break;
    }
}


}//namespace cura
