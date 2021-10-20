//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <algorithm>
#include <map> // multimap (ordered map allowing duplicate keys)
#include <fstream> // ifstream.good()

#ifdef _OPENMP
    #include <omp.h>
#endif // _OPENMP

#include "Application.h"
#include "ConicalOverhang.h"
#include "ExtruderTrain.h"
#include "FffPolygonGenerator.h"
#include "infill.h"
#include "layerPart.h"
#include "MeshGroup.h"
#include "Mold.h"
#include "multiVolumes.h"
#include "PrintFeature.h"
#include "raft.h"
#include "skin.h"
#include "SkirtBrim.h"
#include "Slice.h"
#include "sliceDataStorage.h"
#include "slicer.h"
#include "support.h"
#include "TopSurface.h"
#include "TreeSupport.h"
#include "WallsComputation.h"
#include "infill/DensityProvider.h"
#include "infill/ImageBasedDensityProvider.h"
#include "infill/LightningGenerator.h"
#include "infill/SierpinskiFillProvider.h"
#include "infill/SubDivCube.h"
#include "infill/UniformDensityProvider.h"
#include "progress/Progress.h"
#include "progress/ProgressEstimator.h"
#include "progress/ProgressEstimatorLinear.h"
#include "progress/ProgressStageEstimator.h"
#include "settings/AdaptiveLayerHeights.h"
#include "settings/types/Angle.h"
#include "settings/types/LayerIndex.h"
#include "utils/algorithm.h"
#include "utils/gettime.h"
#include "utils/logoutput.h"
#include "utils/math.h"


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

size_t FffPolygonGenerator::getDraftShieldLayerCount(const size_t total_layers) const
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    if (!mesh_group_settings.get<bool>("draft_shield_enabled"))
    {
        return 0;
    }
    switch (mesh_group_settings.get<DraftShieldHeightLimitation>("draft_shield_height_limitation"))
    {
        case DraftShieldHeightLimitation::FULL:
            return total_layers;
        case DraftShieldHeightLimitation::LIMITED:
            return std::max((coord_t)0, (mesh_group_settings.get<coord_t>("draft_shield_height") - mesh_group_settings.get<coord_t>("layer_height_0")) / mesh_group_settings.get<coord_t>("layer_height") + 1);
        default:
            logWarning("A draft shield height limitation option was added without implementing the new option in getDraftShieldLayerCount.");
            return total_layers;
    }
}

bool FffPolygonGenerator::sliceModel(MeshGroup* meshgroup, TimeKeeper& timeKeeper, SliceDataStorage& storage) /// slices the model
{
    Progress::messageProgressStage(Progress::Stage::SLICING, &timeKeeper);

    storage.model_min = meshgroup->min();
    storage.model_max = meshgroup->max();
    storage.model_size = storage.model_max - storage.model_min;

    log("Slicing model...\n");

    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;

    // regular layers
    int slice_layer_count = 0; //Use signed int because we need to subtract the initial layer in a calculation temporarily.

    // Initial layer height of 0 is not allowed. Negative layer height is nonsense.
    coord_t initial_layer_thickness = mesh_group_settings.get<coord_t>("layer_height_0");
    if (initial_layer_thickness <= 0)
    {
        logError("Initial layer height %i is disallowed.\n", initial_layer_thickness);
        return false;
    }

    // Layer height of 0 is not allowed. Negative layer height is nonsense.
    const coord_t layer_thickness = mesh_group_settings.get<coord_t>("layer_height");
    if(layer_thickness <= 0)
    {
        logError("Layer height %i is disallowed.\n", layer_thickness);
        return false;
    }

    // variable layers
    AdaptiveLayerHeights* adaptive_layer_heights = nullptr;
    const bool use_variable_layer_heights = mesh_group_settings.get<bool>("adaptive_layer_height_enabled");

    if (use_variable_layer_heights)
    {
        // Calculate adaptive layer heights
        const coord_t variable_layer_height_max_variation = mesh_group_settings.get<coord_t>("adaptive_layer_height_variation");
        const coord_t variable_layer_height_variation_step = mesh_group_settings.get<coord_t>("adaptive_layer_height_variation_step");
        const coord_t adaptive_threshold = mesh_group_settings.get<coord_t>("adaptive_layer_height_threshold");
        adaptive_layer_heights = new AdaptiveLayerHeights(layer_thickness, variable_layer_height_max_variation,
                                                          variable_layer_height_variation_step, adaptive_threshold);

        // Get the amount of layers
        slice_layer_count = adaptive_layer_heights->getLayerCount();
    }
    else
    {
        slice_layer_count = round_divide_signed(storage.model_max.z - initial_layer_thickness, layer_thickness) + 1;
    }

    // Model is shallower than layer_height_0, so not even the first layer is sliced. Return an empty model then.
    if (slice_layer_count <= 0)
    {
        return true; // This is NOT an error state!
    }

    std::vector<Slicer*> slicerList;
    for(unsigned int mesh_idx = 0; mesh_idx < meshgroup->meshes.size(); mesh_idx++)
    {
        // Check if adaptive layers is populated to prevent accessing a method on NULL
        std::vector<AdaptiveLayer>* adaptive_layer_height_values = {};
        if (adaptive_layer_heights != nullptr)
        {
            adaptive_layer_height_values = adaptive_layer_heights->getLayers();
        }

        Mesh& mesh = meshgroup->meshes[mesh_idx];
        Slicer* slicer = new Slicer(&mesh, layer_thickness, slice_layer_count, use_variable_layer_heights, adaptive_layer_height_values);

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

    // Clear the mesh face and vertex data, it is no longer needed after this point, and it saves a lot of memory.
    meshgroup->clear();

    Mold::process(slicerList);

    Scene& scene = Application::getInstance().current_slice->scene;
    for (unsigned int mesh_idx = 0; mesh_idx < slicerList.size(); mesh_idx++)
    {
        Mesh& mesh = scene.current_mesh_group->meshes[mesh_idx];
        if (mesh.settings.get<bool>("conical_overhang_enabled") && !mesh.settings.get<bool>("anti_overhang_mesh"))
        {
            ConicalOverhang::apply(slicerList[mesh_idx], mesh);
        }
    }

    MultiVolumes::carveCuttingMeshes(slicerList, scene.current_mesh_group->meshes);

    Progress::messageProgressStage(Progress::Stage::PARTS, &timeKeeper);

    if (scene.current_mesh_group->settings.get<bool>("carve_multiple_volumes"))
    {
        carveMultipleVolumes(slicerList);
    }

    generateMultipleVolumesOverlap(slicerList);

    storage.print_layer_count = 0;
    for (unsigned int meshIdx = 0; meshIdx < slicerList.size(); meshIdx++)
    {
        Mesh& mesh = scene.current_mesh_group->meshes[meshIdx];
        Slicer* slicer = slicerList[meshIdx];
        if (!mesh.settings.get<bool>("anti_overhang_mesh") && !mesh.settings.get<bool>("infill_mesh") && !mesh.settings.get<bool>("cutting_mesh"))
        {
            storage.print_layer_count = std::max(storage.print_layer_count, slicer->layers.size());
        }
    }
    storage.support.supportLayers.resize(storage.print_layer_count);

    storage.meshes.reserve(slicerList.size()); // causes there to be no resize in meshes so that the pointers in sliceMeshStorage._config to retraction_config don't get invalidated.
    for (unsigned int meshIdx = 0; meshIdx < slicerList.size(); meshIdx++)
    {
        Slicer* slicer = slicerList[meshIdx];
        Mesh& mesh = scene.current_mesh_group->meshes[meshIdx];

        // always make a new SliceMeshStorage, so that they have the same ordering / indexing as meshgroup.meshes
        storage.meshes.emplace_back(&meshgroup->meshes[meshIdx], slicer->layers.size()); // new mesh in storage had settings from the Mesh
        SliceMeshStorage& meshStorage = storage.meshes.back();

        // only create layer parts for normal meshes
        const bool is_support_modifier = AreaSupport::handleSupportModifierMesh(storage, mesh.settings, slicer);
        if (!is_support_modifier)
        {
            createLayerParts(meshStorage, slicer);
        }

        // Do not add and process support _modifier_ meshes further, and ONLY skip support _modifiers_. They have been
        // processed in AreaSupport::handleSupportModifierMesh(), but other helper meshes such as infill meshes are
        // processed in a later stage, except for support mesh itself, so an exception is made for that.
        if(is_support_modifier && !mesh.settings.get<bool>("support_mesh"))
        {
            storage.meshes.pop_back();
            continue;
        }

        // check one if raft offset is needed
        const bool has_raft = mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::RAFT;

        // calculate the height at which each layer is actually printed (printZ)
        for (unsigned int layer_nr = 0; layer_nr < meshStorage.layers.size(); layer_nr++)
        {
            SliceLayer& layer = meshStorage.layers[layer_nr];

            if (use_variable_layer_heights)
            {
                meshStorage.layers[layer_nr].printZ = adaptive_layer_heights->getLayers()->at(layer_nr).z_position;
                meshStorage.layers[layer_nr].thickness = adaptive_layer_heights->getLayers()->at(layer_nr).layer_height;
            }
            else
            {
                meshStorage.layers[layer_nr].printZ = initial_layer_thickness + (layer_nr * layer_thickness);

                if (layer_nr == 0)
                {
                    meshStorage.layers[layer_nr].thickness = initial_layer_thickness;
                }
                else
                {
                    meshStorage.layers[layer_nr].thickness = layer_thickness;
                }
            }

            // add the raft offset to each layer
            if (has_raft)
            {
                const ExtruderTrain& train = mesh_group_settings.get<ExtruderTrain&>("adhesion_extruder_nr");
                layer.printZ +=
                    Raft::getTotalThickness()
                    + train.settings.get<coord_t>("raft_airgap")
                    - train.settings.get<coord_t>("layer_0_z_overlap"); // shift all layers (except 0) down

                if (layer_nr == 0)
                {
                    layer.printZ += train.settings.get<coord_t>("layer_0_z_overlap"); // undo shifting down of first layer
                }
            }
        }

        delete slicerList[meshIdx];

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
        if (!mesh.settings.get<bool>("infill_mesh") && !mesh.settings.get<bool>("anti_overhang_mesh"))
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
    std::vector<size_t> mesh_order;
    { // compute mesh order
        std::multimap<int, size_t> order_to_mesh_indices;
        for (size_t mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
        {
            order_to_mesh_indices.emplace(storage.meshes[mesh_idx].settings.get<int>("infill_mesh_order"), mesh_idx);
        }
        for (std::pair<const int, size_t>& order_and_mesh_idx : order_to_mesh_indices)
        {
            mesh_order.push_back(order_and_mesh_idx.second);
        }
    }
    for (size_t mesh_order_idx = 0; mesh_order_idx < mesh_order.size(); ++mesh_order_idx)
    {
        processBasicWallsSkinInfill(storage, mesh_order_idx, mesh_order, inset_skin_progress_estimate);
        Progress::messageProgress(Progress::Stage::INSET_SKIN, mesh_order_idx + 1, storage.meshes.size());
    }

    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    if (isEmptyLayer(storage, 0) && !isEmptyLayer(storage, 1))
    {
        // the first layer is empty, the second is not empty, so remove the empty first layer as support isn't going to be generated under it.
        // Do this irrespective of the value of remove_empty_first_layers as that setting is hidden when support is enabled and so cannot be relied upon

        removeEmptyFirstLayers(storage, storage.print_layer_count); // changes storage.print_layer_count!
    }

    log("Layer count: %i\n", storage.print_layer_count);

    //layerparts2HTML(storage, "output/output.html");

    Progress::messageProgressStage(Progress::Stage::SUPPORT, &time_keeper);

    AreaSupport::generateOverhangAreas(storage);
    AreaSupport::generateSupportAreas(storage);
    TreeSupport tree_support_generator(storage);
    tree_support_generator.generateSupportAreas(storage);

    // we need to remove empty layers after we have processed the insets
    // processInsets might throw away parts if they have no wall at all (cause it doesn't fit)
    // brim depends on the first layer not being empty
    // only remove empty layers if we haven't generate support, because then support was added underneath the model.
    //   for some materials it's better to print on support than on the build plate.
    if (mesh_group_settings.get<bool>("remove_empty_first_layers"))
    {
        removeEmptyFirstLayers(storage, storage.print_layer_count); // changes storage.print_layer_count!
    }
    if (storage.print_layer_count == 0)
    {
        log("Stopping process because there are no non-empty layers.\n");
        return;
    }

    computePrintHeightStatistics(storage);

    // handle helpers
    storage.primeTower.generateGroundpoly();
    storage.primeTower.generatePaths(storage);
    storage.primeTower.subtractFromSupport(storage);

    logDebug("Processing ooze shield\n");
    processOozeShield(storage);

    logDebug("Processing draft shield\n");
    processDraftShield(storage);

    // This catches a special case in which the models are in the air, and then
    // the adhesion mustn't be calculated.
    if (!isEmptyLayer(storage, 0) || storage.primeTower.enabled)
    {
        log("Processing platform adhesion\n");
        processPlatformAdhesion(storage);
    }

    logDebug("Processing gaps\n");
    processOutlineGaps(storage);
    processPerimeterGaps(storage);

    logDebug("Meshes post-processing\n");
    // meshes post processing
    for (SliceMeshStorage& mesh : storage.meshes)
    {
        processDerivedWallsSkinInfill(mesh);
    }

    logDebug("Processing gradual support\n");
    // generate gradual support
    AreaSupport::generateSupportInfillFeatures(storage);
}

void FffPolygonGenerator::processBasicWallsSkinInfill(SliceDataStorage& storage, const size_t mesh_order_idx, const std::vector<size_t>& mesh_order, ProgressStageEstimator& inset_skin_progress_estimate)
{
    size_t mesh_idx = mesh_order[mesh_order_idx];
    SliceMeshStorage& mesh = storage.meshes[mesh_idx];
    size_t mesh_layer_count = mesh.layers.size();
    if (mesh.settings.get<bool>("infill_mesh"))
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
    size_t processed_layer_count = 0;
#pragma omp parallel for default(none) shared(mesh_layer_count, storage, mesh, inset_skin_progress_estimate, processed_layer_count) schedule(dynamic)
    // Use a signed type for the loop counter so MSVC compiles (because it uses OpenMP 2.0, an old version).
    for (int layer_number = 0; layer_number < static_cast<int>(mesh.layers.size()); layer_number++)
    {
        logDebug("Processing insets for layer %i of %i\n", layer_number, mesh_layer_count);
        processInsets(mesh, layer_number);
#ifdef _OPENMP
        if (omp_get_thread_num() == 0)
#endif
        { // progress estimation is done only in one thread so that no two threads message progress at the same time
            int _processed_layer_count;
#if _OPENMP < 201107
#pragma omp critical
#else
#pragma omp atomic read
#endif
                _processed_layer_count = processed_layer_count;
            double progress = inset_skin_progress_estimate.progress(_processed_layer_count);
            Progress::messageProgress(Progress::Stage::INSET_SKIN, progress * 100, 100);
        }
#pragma omp atomic
        processed_layer_count++;
    }

    ProgressEstimatorLinear* skin_estimator = new ProgressEstimatorLinear(mesh_layer_count);
    mesh_inset_skin_progress_estimator->nextStage(skin_estimator);

    bool process_infill = mesh.settings.get<coord_t>("infill_line_distance") > 0;
    if (!process_infill)
    { // do process infill anyway if it's modified by modifier meshes
        const Scene& scene = Application::getInstance().current_slice->scene;
        for (size_t other_mesh_order_idx = mesh_order_idx + 1; other_mesh_order_idx < mesh_order.size(); ++other_mesh_order_idx)
        {
            const size_t other_mesh_idx = mesh_order[other_mesh_order_idx];
            SliceMeshStorage& other_mesh = storage.meshes[other_mesh_idx];
            if (other_mesh.settings.get<bool>("infill_mesh"))
            {
                AABB3D aabb = scene.current_mesh_group->meshes[mesh_idx].getAABB();
                AABB3D other_aabb = scene.current_mesh_group->meshes[other_mesh_idx].getAABB();
                if (aabb.hit(other_aabb))
                {
                    process_infill = true;
                }
            }
        }
    }
    // skin & infill

    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    size_t mesh_max_initial_bottom_layer_count = 0;
    if (mesh_group_settings.get<bool>("magic_spiralize"))
    {
        mesh_max_initial_bottom_layer_count = std::max(mesh_max_initial_bottom_layer_count, mesh.settings.get<size_t>("initial_bottom_layers"));
    }

    processed_layer_count = 0;
#pragma omp parallel default(none) shared(mesh_layer_count, mesh, mesh_max_initial_bottom_layer_count, process_infill, inset_skin_progress_estimate, processed_layer_count, mesh_group_settings)
    {

#pragma omp for schedule(dynamic)
        // Use a signed type for the loop counter so MSVC compiles (because it uses OpenMP 2.0, an old version).
        for (int layer_number = 0; layer_number < static_cast<int>(mesh.layers.size()); layer_number++)
        {
            logDebug("Processing skins and infill layer %i of %i\n", layer_number, mesh_layer_count);
            if (!mesh_group_settings.get<bool>("magic_spiralize") || layer_number < static_cast<int>(mesh_max_initial_bottom_layer_count))    //Only generate up/downskin and infill for the first X layers when spiralize is choosen.
            {
                processSkinsAndInfill(mesh, layer_number, process_infill);
            }
#ifdef _OPENMP
            if (omp_get_thread_num() == 0)
#endif
            { // progress estimation is done only in one thread so that no two threads message progress at the same time
                int _processed_layer_count;
#if _OPENMP < 201107
#pragma omp critical
#else
#pragma omp atomic read
#endif
                    _processed_layer_count = processed_layer_count;
                double progress = inset_skin_progress_estimate.progress(_processed_layer_count);
                Progress::messageProgress(Progress::Stage::INSET_SKIN, progress * 100, 100);
            }
#pragma omp atomic
                processed_layer_count++;
        }
    }
}

void FffPolygonGenerator::processOutlineGaps(SliceDataStorage& storage)
{
    for (SliceMeshStorage& mesh : storage.meshes)
    {
        constexpr int perimeter_gaps_extra_offset = 15; // extra offset so that the perimeter gaps aren't created everywhere due to rounding errors
        const coord_t wall_0_inset = mesh.settings.get<coord_t>("wall_0_inset");
        if (!mesh.settings.get<bool>("fill_outline_gaps") || mesh.settings.get<size_t>("wall_line_count") <= 0)
        {
            continue;
        }
        for (unsigned int layer_nr = 0; layer_nr < mesh.layers.size(); layer_nr++)
        {
            SliceLayer& layer = mesh.layers[layer_nr];
            coord_t wall_line_width_0 = mesh.settings.get<coord_t>("wall_line_width_0");
            if (layer_nr == 0)
            {
                const ExtruderTrain& train = mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr");
                Ratio initial_layer_line_width_factor = train.settings.get<Ratio>("initial_layer_line_width_factor");
                wall_line_width_0 *= initial_layer_line_width_factor;
            }
            for (SliceLayerPart& part : layer.parts)
            {
                // handle outline gaps
                if (mesh.settings.get<bool>("fill_outline_gaps"))
                {
                    const Polygons& outer = part.outline;
                    Polygons inner;
                    if (part.insets.size() > 0)
                    {
                        inner.add(part.insets[0].offset(wall_line_width_0 / 2 + perimeter_gaps_extra_offset + wall_0_inset));
                    }
                    Polygons outline_gaps = outer.difference(inner);
                    outline_gaps.removeSmallAreas(2 * INT2MM(wall_line_width_0) * INT2MM(wall_line_width_0)); // remove small outline gaps to reduce blobs on outside of model
                    part.outline_gaps.add(outline_gaps);
                }
            }
        }
    }
}

void FffPolygonGenerator::processPerimeterGaps(SliceDataStorage& storage)
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    for (SliceMeshStorage& mesh : storage.meshes)
    {
        constexpr int perimeter_gaps_extra_offset = 15; // extra offset so that the perimeter gaps aren't created everywhere due to rounding errors
        const bool fill_perimeter_gaps = mesh.settings.get<FillPerimeterGapMode>("fill_perimeter_gaps") != FillPerimeterGapMode::NOWHERE
            && !mesh_group_settings.get<bool>("magic_spiralize");
        bool filter_out_tiny_gaps = mesh.settings.get<bool>("filter_out_tiny_gaps");

        if (!fill_perimeter_gaps)
        {
            continue;
        }
        for (LayerIndex layer_nr = 0; layer_nr < static_cast<LayerIndex>(mesh.layers.size()); layer_nr++)
        {
            const ExtruderTrain& train_wall_x = mesh.settings.get<ExtruderTrain&>("wall_x_extruder_nr");
            bool fill_gaps_between_inner_wall_and_skin_or_infill =
                mesh.settings.get<coord_t>("infill_line_distance") > 0
                && mesh.settings.get<coord_t>("infill_overlap_mm") >= 0
                && !(mesh.settings.get<EFillMethod>("infill_pattern") == EFillMethod::CONCENTRIC
                    && (mesh.settings.get<bool>("alternate_extra_perimeter") || (layer_nr == 0 && train_wall_x.settings.get<Ratio>("initial_layer_line_width_factor") > 1.0))
                );
            SliceLayer& layer = mesh.layers[layer_nr];
            coord_t wall_line_width_0 = mesh.settings.get<coord_t>("wall_line_width_0");
            coord_t wall_line_width_x = mesh.settings.get<coord_t>("wall_line_width_x");
            coord_t skin_line_width = mesh.settings.get<coord_t>("skin_line_width");
            coord_t infill_line_width = mesh.settings.get<coord_t>("infill_line_width");
            if (layer_nr == 0)
            {
                const ExtruderTrain& train_wall_0 = mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr");
                wall_line_width_0 *= train_wall_0.settings.get<Ratio>("initial_layer_line_width_factor");
                const ExtruderTrain& train_wall_x = mesh.settings.get<ExtruderTrain&>("wall_x_extruder_nr");
                wall_line_width_x *= train_wall_x.settings.get<Ratio>("initial_layer_line_width_factor");
                const ExtruderTrain& train_skin = mesh.settings.get<ExtruderTrain&>("top_bottom_extruder_nr");
                skin_line_width *= train_skin.settings.get<Ratio>("initial_layer_line_width_factor");
            }
            for (SliceLayerPart& part : layer.parts)
            {
                 // handle perimeter gaps of normal insets
                int line_width = wall_line_width_0;
                for (unsigned int inset_idx = 0; static_cast<int>(inset_idx) < static_cast<int>(part.insets.size()) - 1; inset_idx++)
                {
                    const Polygons outer = part.insets[inset_idx].offset(-1 * line_width / 2 - perimeter_gaps_extra_offset);
                    line_width = wall_line_width_x;

                    Polygons inner = part.insets[inset_idx + 1].offset(line_width / 2);
                    part.perimeter_gaps.add(outer.difference(inner));
                }

                if (filter_out_tiny_gaps) {
                    part.perimeter_gaps.removeSmallAreas(2 * INT2MM(wall_line_width_0) * INT2MM(wall_line_width_0)); // remove small outline gaps to reduce blobs on outside of model
                }

                // gap between inner wall and skin/infill
                if (fill_gaps_between_inner_wall_and_skin_or_infill && part.insets.size() > 0)
                {
                    const Polygons outer = part.insets.back().offset(-1 * line_width / 2 - perimeter_gaps_extra_offset);

                    // accumulate area of skin and infill that will be printed
                    Polygons inner;
                    for (const SkinPart& skin_part : part.skin_parts)
                    {
                        inner.add(skin_part.outline);
                    }
                    // for some reason the zig-zag and lines patterns behave differently and a narrow region that isn't filled with zig-zag pattern can be filled with
                    // lines pattern so we only add the narrow region to the perimeter gaps when the pattern is zig-zag.
                    if (((layer_nr == 0) ? mesh.settings.get<EFillMethod>("top_bottom_pattern_0") : mesh.settings.get<EFillMethod>("top_bottom_pattern")) == EFillMethod::ZIG_ZAG)
                    {
                        // remove skin areas that are narrower than skin_line_width as they won't get printed unless
                        // we print them as a perimeter gap
                        inner = inner.offset(-skin_line_width / 2).offset(skin_line_width / 2);
                    }
                    inner.add(part.infill_area.offset(-infill_line_width / 2).offset(infill_line_width / 2));
                    inner = inner.unionPolygons();
                    part.perimeter_gaps.add(outer.difference(inner));

                    if (filter_out_tiny_gaps) {
                        part.perimeter_gaps.removeSmallAreas(2 * INT2MM(infill_line_width) * INT2MM(infill_line_width));
                    }
                }

                // add perimeter gaps for skin insets
                for (SkinPart& skin_part : part.skin_parts)
                {
                    if (skin_part.insets.size() > 0)
                    {
                        // add perimeter gaps between the outer skin inset and the innermost wall
                        const Polygons outer = skin_part.outline;
                        const Polygons inner = skin_part.insets[0].offset(skin_line_width / 2 + perimeter_gaps_extra_offset);
                        skin_part.perimeter_gaps.add(outer.difference(inner));

                        for (unsigned int inset_idx = 1; inset_idx < skin_part.insets.size(); inset_idx++)
                        { // add perimeter gaps between consecutive skin walls
                            const Polygons outer = skin_part.insets[inset_idx - 1].offset(-1 * skin_line_width / 2 - perimeter_gaps_extra_offset);
                            const Polygons inner = skin_part.insets[inset_idx].offset(skin_line_width / 2);
                            skin_part.perimeter_gaps.add(outer.difference(inner));
                        }

                        if (filter_out_tiny_gaps) {
                            skin_part.perimeter_gaps.removeSmallAreas(2 * INT2MM(skin_line_width) * INT2MM(skin_line_width)); // remove small outline gaps to reduce blobs on outside of model
                        }
                    }
                }
            }
        }
    }
}

void FffPolygonGenerator::processInfillMesh(SliceDataStorage& storage, const size_t mesh_order_idx, const std::vector<size_t>& mesh_order)
{
    size_t mesh_idx = mesh_order[mesh_order_idx];
    SliceMeshStorage& mesh = storage.meshes[mesh_idx];
    mesh.layer_nr_max_filled_layer = -1;
    for (LayerIndex layer_idx = 0; layer_idx < static_cast<LayerIndex>(mesh.layers.size()); layer_idx++)
    {
        SliceLayer& layer = mesh.layers[layer_idx];
        std::vector<PolygonsPart> new_parts;

        for (const size_t other_mesh_idx : mesh_order)
        { // limit the infill mesh's outline to within the infill of all meshes with lower order
            if (other_mesh_idx == mesh_idx)
            {
                break; // all previous meshes have been processed
            }
            SliceMeshStorage& other_mesh = storage.meshes[other_mesh_idx];
            if (layer_idx >= static_cast<LayerIndex>(other_mesh.layers.size()))
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

        if (layer.parts.size() > 0 || (mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::NORMAL && layer.openPolyLines.size() > 0) )
        {
            mesh.layer_nr_max_filled_layer = layer_idx; // last set by the highest non-empty layer
        }
    }

}

void FffPolygonGenerator::processDerivedWallsSkinInfill(SliceMeshStorage& mesh)
{
    if (mesh.settings.get<bool>("infill_support_enabled"))
    {// create gradual infill areas
        SkinInfillAreaComputation::generateInfillSupport(mesh);
    }

    // create gradual infill areas
    SkinInfillAreaComputation::generateGradualInfill(mesh);

    //SubDivCube Pre-compute Octree
    if (mesh.settings.get<coord_t>("infill_line_distance") > 0
        && mesh.settings.get<EFillMethod>("infill_pattern") == EFillMethod::CUBICSUBDIV)
    {
        const Point3 mesh_middle = mesh.bounding_box.getMiddle();
        const Point infill_origin(mesh_middle.x + mesh.settings.get<coord_t>("infill_offset_x"), mesh_middle.y + mesh.settings.get<coord_t>("infill_offset_y"));
        SubDivCube::precomputeOctree(mesh, infill_origin);
    }

    // Pre-compute Cross Fractal
    if (mesh.settings.get<coord_t>("infill_line_distance") > 0
        && (mesh.settings.get<EFillMethod>("infill_pattern") == EFillMethod::CROSS
            || mesh.settings.get<EFillMethod>("infill_pattern") == EFillMethod::CROSS_3D)
    )
    {
        const std::string cross_subdivision_spec_image_file = mesh.settings.get<std::string>("cross_infill_density_image");
        std::ifstream cross_fs(cross_subdivision_spec_image_file.c_str());
        if (!cross_subdivision_spec_image_file.empty() && cross_fs.good())
        {
            mesh.cross_fill_provider = new SierpinskiFillProvider(mesh.bounding_box, mesh.settings.get<coord_t>("infill_line_distance"), mesh.settings.get<coord_t>("infill_line_width"), cross_subdivision_spec_image_file);
        }
        else
        {
            if (!cross_subdivision_spec_image_file.empty() && cross_subdivision_spec_image_file != " ")
            {
                logError("Cannot find density image \'%s\'.", cross_subdivision_spec_image_file.c_str());
            }
            mesh.cross_fill_provider = new SierpinskiFillProvider(mesh.bounding_box, mesh.settings.get<coord_t>("infill_line_distance"), mesh.settings.get<coord_t>("infill_line_width"));
        }
    }

    // Pre-compute lightning fill (aka minfill, aka ribbed support vaults)
    if (mesh.settings.get<coord_t>("infill_line_distance") > 0 && mesh.settings.get<EFillMethod>("infill_pattern") == EFillMethod::LIGHTNING)
    {
        // TODO: Make all of these into new type pointers (but the cross fill things need to happen too then, otherwise it'd just look weird).
        mesh.lightning_generator = new LightningGenerator(mesh);
    }

    // combine infill
    SkinInfillAreaComputation::combineInfillLayers(mesh);

    // fuzzy skin
    if (mesh.settings.get<bool>("magic_fuzzy_skin_enabled"))
    {
        processFuzzyWalls(mesh);
    }
}

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * processInsets only reads and writes data for the current layer
 */
void FffPolygonGenerator::processInsets(SliceMeshStorage& mesh, size_t layer_nr)
{
    SliceLayer* layer = &mesh.layers[layer_nr];
    if (mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::SURFACE)
    {
        WallsComputation walls_computation(mesh.settings, layer_nr);
        walls_computation.generateInsets(layer);
    }
    else
    {
        for (SliceLayerPart& part : layer->parts)
        {
            part.insets.push_back(part.outline); // Fake an inset
            part.print_outline = part.outline;
        }
    }
}

bool FffPolygonGenerator::isEmptyLayer(SliceDataStorage& storage, const unsigned int layer_idx)
{
    if (storage.support.generated && layer_idx < storage.support.supportLayers.size())
    {
        SupportLayer& support_layer = storage.support.supportLayers[layer_idx];
        if (!support_layer.support_infill_parts.empty() || !support_layer.support_bottom.empty() || !support_layer.support_roof.empty())
        {
            return false;
        }
    }
    for (SliceMeshStorage& mesh : storage.meshes)
    {
        if (layer_idx >= mesh.layers.size())
        {
            continue;
        }
        SliceLayer& layer = mesh.layers[layer_idx];
        if (mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::NORMAL && layer.openPolyLines.size() > 0)
        {
            return false;
        }
        for (const SliceLayerPart& part : layer.parts)
        {
            if (part.print_outline.size() > 0)
            {
                return false;
            }
        }
    }
    return true;
}

void FffPolygonGenerator::removeEmptyFirstLayers(SliceDataStorage& storage, size_t& total_layers)
{
    size_t n_empty_first_layers = 0;
    for (size_t layer_idx = 0; layer_idx < total_layers; layer_idx++)
    {
        if (isEmptyLayer(storage, layer_idx))
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
        const coord_t layer_height = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<coord_t>("layer_height");
        for (SliceMeshStorage& mesh : storage.meshes)
        {
            std::vector<SliceLayer>& layers = mesh.layers;
            if (layers.size() > n_empty_first_layers)
            {
                // transfer initial layer thickness to new initial layer
                layers[n_empty_first_layers].thickness = layers[0].thickness;
            }
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

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * generateSkins read (depend on) data from mesh.layers[*].parts[*].insets and write mesh.layers[n].parts[*].skin_parts
 * generateInfill read mesh.layers[n].parts[*].{insets,skin_parts,boundingBox} and write mesh.layers[n].parts[*].infill_area
 *
 * processSkinsAndInfill read (depend on) mesh.layers[*].parts[*].{insets,boundingBox}.
 *                       write mesh.layers[n].parts[*].{skin_parts,infill_area}.
 */
void FffPolygonGenerator::processSkinsAndInfill(SliceMeshStorage& mesh, const LayerIndex layer_nr, bool process_infill)
{
    if (mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") == ESurfaceMode::SURFACE)
    {
        return;
    }

    SkinInfillAreaComputation skin_infill_area_computation(layer_nr, mesh, process_infill);
    skin_infill_area_computation.generateSkinsAndInfill();

    if (mesh.settings.get<bool>("ironing_enabled") && (!mesh.settings.get<bool>("ironing_only_highest_layer") || mesh.layer_nr_max_filled_layer == layer_nr))
    {
        // Generate the top surface to iron over.
        mesh.layers[layer_nr].top_surface.setAreasFromMeshAndLayerNumber(mesh, layer_nr);
    }
}

void FffPolygonGenerator::computePrintHeightStatistics(SliceDataStorage& storage)
{
    const size_t extruder_count = Application::getInstance().current_slice->scene.extruders.size();

    std::vector<int>& max_print_height_per_extruder = storage.max_print_height_per_extruder;
    assert(max_print_height_per_extruder.size() == 0 && "storage.max_print_height_per_extruder shouldn't have been initialized yet!");
    max_print_height_per_extruder.resize(extruder_count, -(Raft::getTotalExtraLayers() + 1)); //Initialize all as -1 (or lower in case of raft).
    { // compute max_object_height_per_extruder
        //Height of the meshes themselves.
        for (SliceMeshStorage& mesh : storage.meshes)
        {
            if (mesh.settings.get<bool>("anti_overhang_mesh") || mesh.settings.get<bool>("support_mesh"))
            {
                continue; //Special type of mesh that doesn't get printed.
            }
            for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
            {
                for (LayerIndex layer_nr = static_cast<LayerIndex>(mesh.layers.size()) - 1; layer_nr > max_print_height_per_extruder[extruder_nr]; layer_nr--)
                {
                    if (mesh.getExtruderIsUsed(extruder_nr, layer_nr))
                    {
                        assert(max_print_height_per_extruder[extruder_nr] <= layer_nr);
                        max_print_height_per_extruder[extruder_nr] = layer_nr;
                    }
                }
            }
        }

        //Height of where the support reaches.
        Scene& scene = Application::getInstance().current_slice->scene;
        const size_t support_infill_extruder_nr = scene.current_mesh_group->settings.get<ExtruderTrain&>("support_infill_extruder_nr").extruder_nr; // TODO: Support extruder should be configurable per object.
        max_print_height_per_extruder[support_infill_extruder_nr] =
            std::max(max_print_height_per_extruder[support_infill_extruder_nr],
                     storage.support.layer_nr_max_filled_layer);
        const size_t support_roof_extruder_nr = scene.current_mesh_group->settings.get<ExtruderTrain&>("support_roof_extruder_nr").extruder_nr; // TODO: Support roof extruder should be configurable per object.
        max_print_height_per_extruder[support_roof_extruder_nr] =
            std::max(max_print_height_per_extruder[support_roof_extruder_nr],
                     storage.support.layer_nr_max_filled_layer);
        const size_t support_bottom_extruder_nr = scene.current_mesh_group->settings.get<ExtruderTrain&>("support_bottom_extruder_nr").extruder_nr; //TODO: Support bottom extruder should be configurable per object.
        max_print_height_per_extruder[support_bottom_extruder_nr] =
            std::max(max_print_height_per_extruder[support_bottom_extruder_nr],
                     storage.support.layer_nr_max_filled_layer);

        //Height of where the platform adhesion reaches.
        if (scene.current_mesh_group->settings.get<EPlatformAdhesion>("adhesion_type") != EPlatformAdhesion::NONE)
        {
            const size_t adhesion_extruder_nr = scene.current_mesh_group->settings.get<ExtruderTrain&>("adhesion_extruder_nr").extruder_nr;
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
        storage.max_print_height_second_to_last_extruder = -(Raft::getTotalExtraLayers() + 1);
    }
}


void FffPolygonGenerator::processOozeShield(SliceDataStorage& storage)
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    if (!mesh_group_settings.get<bool>("ooze_shield_enabled"))
    {
        return;
    }

    const coord_t ooze_shield_dist = mesh_group_settings.get<coord_t>("ooze_shield_dist");

    for (int layer_nr = 0; layer_nr <= storage.max_print_height_second_to_last_extruder; layer_nr++)
    {
        constexpr bool around_support = true;
        constexpr bool around_prime_tower = false;
        storage.oozeShield.push_back(storage.getLayerOutlines(layer_nr, around_support, around_prime_tower).offset(ooze_shield_dist, ClipperLib::jtRound).getOutsidePolygons());
    }

    const AngleDegrees angle = mesh_group_settings.get<AngleDegrees>("ooze_shield_angle");
    if (angle <= 89)
    {
        const coord_t allowed_angle_offset = tan(mesh_group_settings.get<AngleRadians>("ooze_shield_angle")) * mesh_group_settings.get<coord_t>("layer_height"); // Allow for a 60deg angle in the oozeShield.
        for (LayerIndex layer_nr = 1; layer_nr <= storage.max_print_height_second_to_last_extruder; layer_nr++)
        {
            storage.oozeShield[layer_nr] = storage.oozeShield[layer_nr].unionPolygons(storage.oozeShield[layer_nr - 1].offset(-allowed_angle_offset));
        }
        for (LayerIndex layer_nr = storage.max_print_height_second_to_last_extruder; layer_nr > 0; layer_nr--)
        {
            storage.oozeShield[layer_nr - 1] = storage.oozeShield[layer_nr - 1].unionPolygons(storage.oozeShield[layer_nr].offset(-allowed_angle_offset));
        }
    }

    const float largest_printed_area = 1.0; // TODO: make var a parameter, and perhaps even a setting?
    for (LayerIndex layer_nr = 0; layer_nr <= storage.max_print_height_second_to_last_extruder; layer_nr++)
    {
        storage.oozeShield[layer_nr].removeSmallAreas(largest_printed_area);
    }
}

void FffPolygonGenerator::processDraftShield(SliceDataStorage& storage)
{
    const size_t draft_shield_layers = getDraftShieldLayerCount(storage.print_layer_count);
    if (draft_shield_layers <= 0)
    {
        return;
    }
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const coord_t layer_height = mesh_group_settings.get<coord_t>("layer_height");

    const unsigned int layer_skip = 500 / layer_height + 1;

    Polygons& draft_shield = storage.draft_protection_shield;
    for (unsigned int layer_nr = 0; layer_nr < storage.print_layer_count && layer_nr < draft_shield_layers; layer_nr += layer_skip)
    {
        constexpr bool around_support = true;
        constexpr bool around_prime_tower = false;
        draft_shield = draft_shield.unionPolygons(storage.getLayerOutlines(layer_nr, around_support, around_prime_tower));
    }

    const coord_t draft_shield_dist = mesh_group_settings.get<coord_t>("draft_shield_dist");
    storage.draft_protection_shield = draft_shield.approxConvexHull(draft_shield_dist);

    //Extra offset has rounded joints, so simplify again.
    coord_t maximum_resolution = 0; //Draft shield is printed with every extruder, so resolve with the max() or min() of them to meet the requirements of all extruders.
    coord_t maximum_deviation = std::numeric_limits<coord_t>::max();
    for(const ExtruderTrain& extruder : Application::getInstance().current_slice->scene.extruders)
    {
        maximum_resolution = std::max(maximum_resolution, extruder.settings.get<coord_t>("meshfix_maximum_resolution"));
        maximum_deviation = std::min(maximum_deviation, extruder.settings.get<coord_t>("meshfix_maximum_deviation"));
    }
    storage.draft_protection_shield.simplify(maximum_resolution, maximum_deviation);
}

void FffPolygonGenerator::processPlatformAdhesion(SliceDataStorage& storage)
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    ExtruderTrain& train = mesh_group_settings.get<ExtruderTrain&>("adhesion_extruder_nr");

    Polygons first_layer_outline;
    coord_t primary_line_count;

    EPlatformAdhesion adhesion_type = mesh_group_settings.get<EPlatformAdhesion>("adhesion_type");

    if (adhesion_type == EPlatformAdhesion::SKIRT)
    {
        primary_line_count = train.settings.get<size_t>("skirt_line_count");
        SkirtBrim::getFirstLayerOutline(storage, primary_line_count, true, first_layer_outline);
        SkirtBrim::generate(storage, first_layer_outline, train.settings.get<coord_t>("skirt_gap"), primary_line_count);
    }

    // Generate any brim for the prime tower, should happen _after_ any skirt, but _before_ any other brim (since FffGCodeWriter assumes that the outermost contour is last).
    if (adhesion_type != EPlatformAdhesion::RAFT && storage.primeTower.enabled && mesh_group_settings.get<bool>("prime_tower_brim_enable"))
    {
        constexpr bool dont_allow_helpers = false;
        SkirtBrim::generate(storage, storage.primeTower.outer_poly, 0, train.settings.get<size_t>("brim_line_count"), dont_allow_helpers);
    }

    switch(adhesion_type)
    {
    case EPlatformAdhesion::SKIRT:
        // Already done, because of prime-tower-brim & ordering, see above.
        break;
    case EPlatformAdhesion::BRIM:
        primary_line_count = train.settings.get<size_t>("brim_line_count");
        SkirtBrim::getFirstLayerOutline(storage, primary_line_count, false, first_layer_outline);
        SkirtBrim::generate(storage, first_layer_outline, 0, primary_line_count);
        break;
    case EPlatformAdhesion::RAFT:
        Raft::generate(storage);
        break;
    case EPlatformAdhesion::NONE:
        if (mesh_group_settings.get<bool>("support_brim_enable"))
        {
            SkirtBrim::generate(storage, Polygons(), 0, 0);
        }
        break;
    }

    // Also apply maximum_[deviation|resolution] to skirt/brim.
    const coord_t line_segment_resolution = train.settings.get<coord_t>("meshfix_maximum_resolution");
    const coord_t line_segment_deviation = train.settings.get<coord_t>("meshfix_maximum_deviation");
    for (Polygons& polygons : storage.skirt_brim)
    {
        polygons.simplify(line_segment_resolution, line_segment_deviation);
    }
}


void FffPolygonGenerator::processFuzzyWalls(SliceMeshStorage& mesh)
{
    if (mesh.settings.get<size_t>("wall_line_count") == 0)
    {
        return;
    }
    const coord_t fuzziness = mesh.settings.get<coord_t>("magic_fuzzy_skin_thickness");
    const coord_t avg_dist_between_points = mesh.settings.get<coord_t>("magic_fuzzy_skin_point_dist");
    const coord_t min_dist_between_points = avg_dist_between_points * 3 / 4; // hardcoded: the point distance may vary between 3/4 and 5/4 the supplied value
    const coord_t range_random_point_dist = avg_dist_between_points / 2;
    unsigned int start_layer_nr = (mesh.settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::BRIM)? 1 : 0; // don't make fuzzy skin on first layer if there's a brim
    for (unsigned int layer_nr = start_layer_nr; layer_nr < mesh.layers.size(); layer_nr++)
    {
        SliceLayer& layer = mesh.layers[layer_nr];
        for (SliceLayerPart& part : layer.parts)
        {
            Polygons results;
            Polygons& skin = (mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") == ESurfaceMode::SURFACE)? part.outline : part.insets[0];
            for (PolygonRef poly : skin)
            {
                if (mesh.settings.get<bool>("magic_fuzzy_skin_outside_only") && poly.area() < 0)
                {
                    results.add(poly);
                    continue;
                }
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
                while (result.size() < 3)
                {
                    size_t point_idx = poly.size() - 2;
                    result.add(poly[point_idx]);
                    if (point_idx == 0)
                    {
                        break;
                    }
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
