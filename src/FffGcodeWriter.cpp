//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <list>
#include <limits> // numeric_limits

#include "Application.h"
#include "bridge.h"
#include "ExtruderTrain.h"
#include "FffGcodeWriter.h"
#include "FffProcessor.h"
#include "GcodeLayerThreader.h"
#include "infill.h"
#include "InsetOrderOptimizer.h"
#include "LayerPlan.h"
#include "raft.h"
#include "Slice.h"
#include "wallOverlap.h"
#include "communication/Communication.h" //To send layer view data.
#include "infill/SpaghettiInfillPathGenerator.h"
#include "progress/Progress.h"
#include "utils/math.h"
#include "utils/orderOptimizer.h"

#define OMP_MAX_ACTIVE_LAYERS_PROCESSED 30 // TODO: hardcoded-value for the max number of layers being in the pipeline while writing away and destroying layers in a multi-threaded context

namespace cura
{

FffGcodeWriter::FffGcodeWriter()
: max_object_height(0)
, layer_plan_buffer(gcode)
{
    for (unsigned int extruder_nr = 0; extruder_nr < MAX_EXTRUDERS; extruder_nr++)
    { // initialize all as max layer_nr, so that they get updated to the lowest layer on which they are used.
        extruder_prime_layer_nr[extruder_nr] = std::numeric_limits<int>::max();
    }
}

void FffGcodeWriter::writeGCode(SliceDataStorage& storage, TimeKeeper& time_keeper)
{
    const size_t start_extruder_nr = getStartExtruder(storage);
    gcode.preSetup(start_extruder_nr);

    Scene& scene = Application::getInstance().current_slice->scene;
    if (scene.current_mesh_group == scene.mesh_groups.begin()) //First mesh group.
    {
        gcode.resetTotalPrintTimeAndFilament();
        gcode.setInitialAndBuildVolumeTemps(start_extruder_nr);
    }

    Application::getInstance().communication->beginGCode();

    setConfigFanSpeedLayerTime();

    setConfigRetraction(storage);

    setConfigWipe(storage);

    if (scene.current_mesh_group == scene.mesh_groups.begin())
    {
        processStartingCode(storage, start_extruder_nr);
    }
    else
    {
        processNextMeshGroupCode(storage);
    }

    size_t total_layers = 0;
    for (SliceMeshStorage& mesh : storage.meshes)
    {
        if (mesh.isPrinted()) //No need to process higher layers if the non-printed meshes are higher than the normal meshes.
        {
            total_layers = std::max(total_layers, mesh.layers.size());
        }

        setInfillAndSkinAngles(mesh);
    }
    
    gcode.writeLayerCountComment(total_layers);

    { // calculate the mesh order for each extruder
        const size_t extruder_count = Application::getInstance().current_slice->scene.extruders.size();
        mesh_order_per_extruder.reserve(extruder_count);
        for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
        {
            mesh_order_per_extruder.push_back(calculateMeshOrder(storage, extruder_nr));
        }
    }
    calculateExtruderOrderPerLayer(storage);

    if (scene.current_mesh_group->settings.get<bool>("magic_spiralize"))
    {
        findLayerSeamsForSpiralize(storage, total_layers);
    }

    int process_layer_starting_layer_nr = 0;
    const bool has_raft = scene.current_mesh_group->settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::RAFT;
    if (has_raft)
    {
        processRaft(storage);
        // process filler layers to fill the airgap with helper object (support etc) so that they stick better to the raft.
        // only process the filler layers if there is anything to print in them.
        for (bool extruder_is_used_in_filler_layers : storage.getExtrudersUsed(-1))
        {
            if (extruder_is_used_in_filler_layers)
            {
                process_layer_starting_layer_nr = -Raft::getFillerLayerCount();
                break;
            }
        }
    }


    const std::function<LayerPlan* (int)>& produce_item =
        [&storage, total_layers, this](int layer_nr)
        {
            LayerPlan& gcode_layer = processLayer(storage, layer_nr, total_layers);
            return &gcode_layer;
        };
    const std::function<void (LayerPlan*)>& consume_item =
        [this, total_layers](LayerPlan* gcode_layer)
        {
            Progress::messageProgress(Progress::Stage::EXPORT, std::max(0, gcode_layer->getLayerNr()) + 1, total_layers);
            layer_plan_buffer.handle(*gcode_layer, gcode);
        };
    const unsigned int max_task_count = OMP_MAX_ACTIVE_LAYERS_PROCESSED;
    GcodeLayerThreader<LayerPlan> threader(
        process_layer_starting_layer_nr
        , static_cast<int>(total_layers)
        , produce_item
        , consume_item
        , max_task_count
    );

    // process all layers, process buffer for preheating and minimal layer time etc, write layers to gcode:
    threader.run();

    layer_plan_buffer.flush();

    Progress::messageProgressStage(Progress::Stage::FINISH, &time_keeper);

    //Store the object height for when we are printing multiple objects, as we need to clear every one of them when moving to the next position.
    max_object_height = std::max(max_object_height, storage.model_max.z);


    constexpr bool force = true;
    gcode.writeRetraction(storage.retraction_config_per_extruder[gcode.getExtruderNr()], force); // retract after finishing each meshgroup
}

unsigned int FffGcodeWriter::findSpiralizedLayerSeamVertexIndex(const SliceDataStorage& storage, const SliceMeshStorage& mesh, const int layer_nr, const int last_layer_nr)
{
    const SliceLayer& layer = mesh.layers[layer_nr];

    // last_layer_nr will be < 0 until we have processed the first non-empty layer
    if (last_layer_nr < 0)
    {
        // If the user has specified a z-seam location, use the vertex closest to that location for the seam vertex
        // in the first layer that has a part with insets. This allows the user to alter the seam start location which
        // could be useful if the spiralization has a problem with a particular seam path.
        Point seam_pos(0, 0);
        if (mesh.settings.get<EZSeamType>("z_seam_type") == EZSeamType::USER_SPECIFIED)
        {
            seam_pos = mesh.getZSeamHint();
        }
        return PolygonUtils::findClosest(seam_pos, layer.parts[0].insets[0][0]).point_idx;
    }
    else
    {
        // note that the code below doesn't assume that last_layer_nr is one less than layer_nr but the print is going
        // to come out pretty weird if that isn't true as it implies that there are empty layers

        ConstPolygonRef last_wall = (*storage.spiralize_wall_outlines[last_layer_nr])[0];
        ConstPolygonRef wall = layer.parts[0].insets[0][0];
        const int n_points = wall.size();
        Point last_wall_seam_vertex = last_wall[storage.spiralize_seam_vertex_indices[last_layer_nr]];

        // seam_vertex_idx is going to be the index of the seam vertex in the current wall polygon
        // initially we choose the vertex that is closest to the seam vertex in the last spiralized layer processed

        int seam_vertex_idx = PolygonUtils::findClosest(last_wall_seam_vertex, wall).point_idx;

        // now we check that the vertex following the seam vertex is to the left of the seam vertex in the last layer
        // and if it isn't, we move forward

        // get the inward normal of the last layer seam vertex
        Point last_wall_seam_vertex_inward_normal = PolygonUtils::getVertexInwardNormal(last_wall, storage.spiralize_seam_vertex_indices[last_layer_nr]);

        // create a vector from the normal so that we can then test the vertex following the candidate seam vertex to make sure it is on the correct side
        Point last_wall_seam_vertex_vector = last_wall_seam_vertex + last_wall_seam_vertex_inward_normal;

        // now test the vertex following the candidate seam vertex and if it lies to the left of the vector, it's good to use
        const int first_seam_vertex_idx = seam_vertex_idx;
        float a = LinearAlg2D::getAngleLeft(last_wall_seam_vertex_vector, last_wall_seam_vertex, wall[(seam_vertex_idx + 1) % n_points]);

        while (a <= 0 || a >= M_PI)
        {
            // the vertex was not on the left of the vector so move the seam vertex on
            seam_vertex_idx = (seam_vertex_idx + 1) % n_points;
            a = LinearAlg2D::getAngleLeft(last_wall_seam_vertex_vector, last_wall_seam_vertex, wall[(seam_vertex_idx + 1) % n_points]);

            if (seam_vertex_idx == first_seam_vertex_idx)
            {
                logWarning("WARNING: findLayerSeamsForSpiralize() failed to find a suitable seam vertex on layer %d\n", layer_nr);
                // this shouldn't happen very often - I have seen it occur when the seam moves into a very sharp corner
                break;
            }
        }

        return seam_vertex_idx;
    }
}

void FffGcodeWriter::findLayerSeamsForSpiralize(SliceDataStorage& storage, size_t total_layers)
{
    // The spiral has to continue on in an anti-clockwise direction from where the last layer finished, it can't jump backwards

    // we track the seam position for each layer and ensure that the seam position for next layer continues in the right direction

    storage.spiralize_wall_outlines.reserve(total_layers);
    storage.spiralize_seam_vertex_indices.reserve(total_layers);

    int last_layer_nr = -1; // layer number of the last non-empty layer processed (for any extruder or mesh)

    for (unsigned layer_nr = 0; layer_nr < total_layers; ++layer_nr)
    {
        bool done_this_layer = false;

        // default is no information available
        storage.spiralize_wall_outlines[layer_nr] = nullptr;
        storage.spiralize_seam_vertex_indices[layer_nr] = 0;

        // iterate through extruders until we find a mesh that has a part with insets
        const std::vector<size_t>& extruder_order = extruder_order_per_layer[layer_nr];
        for (unsigned int extruder_idx = 0; !done_this_layer && extruder_idx < extruder_order.size(); ++extruder_idx)
        {
            const size_t extruder_nr = extruder_order[extruder_idx];
            // iterate through this extruder's meshes until we find a part with insets
            const std::vector<size_t>& mesh_order = mesh_order_per_extruder[extruder_nr];
            for (unsigned int mesh_idx : mesh_order)
            {
                SliceMeshStorage& mesh = storage.meshes[mesh_idx];
                // if this mesh has layer data for this layer process it
                if (!done_this_layer && mesh.layers.size() > layer_nr)
                {
                    SliceLayer& layer = mesh.layers[layer_nr];
                    // if the first part in the layer (if any) has insets, process it
                    if (layer.parts.size() != 0 && layer.parts[0].insets.size() != 0)
                    {
                        // save the seam vertex index for this layer as we need it to determine the seam vertex index for the next layer
                        storage.spiralize_seam_vertex_indices[layer_nr] = findSpiralizedLayerSeamVertexIndex(storage, mesh, layer_nr, last_layer_nr);
                        // save the wall outline for this layer so it can be used in the spiralize interpolation calculation
                        storage.spiralize_wall_outlines[layer_nr] = &layer.parts[0].insets[0];
                        last_layer_nr = layer_nr;
                        // ignore any further meshes/extruders for this layer
                        done_this_layer = true;
                    }
                }
            }
        }
    }
}

void FffGcodeWriter::setConfigFanSpeedLayerTime()
{
    for (const ExtruderTrain& train : Application::getInstance().current_slice->scene.extruders)
    {
        fan_speed_layer_time_settings_per_extruder.emplace_back();
        FanSpeedLayerTimeSettings& fan_speed_layer_time_settings = fan_speed_layer_time_settings_per_extruder.back();
        fan_speed_layer_time_settings.cool_min_layer_time = train.settings.get<Duration>("cool_min_layer_time");
        fan_speed_layer_time_settings.cool_min_layer_time_fan_speed_max = train.settings.get<Duration>("cool_min_layer_time_fan_speed_max");
        fan_speed_layer_time_settings.cool_fan_speed_0 = train.settings.get<Ratio>("cool_fan_speed_0") * 100.0;
        fan_speed_layer_time_settings.cool_fan_speed_min = train.settings.get<Ratio>("cool_fan_speed_min") * 100.0;
        fan_speed_layer_time_settings.cool_fan_speed_max = train.settings.get<Ratio>("cool_fan_speed_max") * 100.0;
        fan_speed_layer_time_settings.cool_min_speed = train.settings.get<Velocity>("cool_min_speed");
        fan_speed_layer_time_settings.cool_fan_full_layer = train.settings.get<LayerIndex>("cool_fan_full_layer");
        if (!train.settings.get<bool>("cool_fan_enabled"))
        {
            fan_speed_layer_time_settings.cool_fan_speed_0 = 0;
            fan_speed_layer_time_settings.cool_fan_speed_min = 0;
            fan_speed_layer_time_settings.cool_fan_speed_max = 0;
        }
    }
}

void FffGcodeWriter::setConfigRetraction(SliceDataStorage& storage) 
{
    Scene& scene = Application::getInstance().current_slice->scene;
    for (size_t extruder_index = 0; extruder_index < scene.extruders.size(); extruder_index++)
    {
        ExtruderTrain& train = scene.extruders[extruder_index];
        RetractionConfig& retraction_config = storage.retraction_config_per_extruder[extruder_index];
        retraction_config.distance = (train.settings.get<bool>("retraction_enable")) ? train.settings.get<double>("retraction_amount") : 0; //Retraction distance in mm.
        retraction_config.prime_volume = train.settings.get<double>("retraction_extra_prime_amount"); //Extra prime volume in mm^3.
        retraction_config.speed = train.settings.get<Velocity>("retraction_retract_speed");
        retraction_config.primeSpeed = train.settings.get<Velocity>("retraction_prime_speed");
        retraction_config.zHop = train.settings.get<coord_t>("retraction_hop");
        retraction_config.retraction_min_travel_distance = train.settings.get<coord_t>("retraction_min_travel");
        retraction_config.retraction_extrusion_window = train.settings.get<double>("retraction_extrusion_window"); //Window to count retractions in in mm of extruded filament.
        retraction_config.retraction_count_max = train.settings.get<size_t>("retraction_count_max");

        RetractionConfig& switch_retraction_config = storage.extruder_switch_retraction_config_per_extruder[extruder_index];
        switch_retraction_config.distance = train.settings.get<double>("switch_extruder_retraction_amount"); //Retraction distance in mm.
        switch_retraction_config.prime_volume = 0.0;
        switch_retraction_config.speed = train.settings.get<Velocity>("switch_extruder_retraction_speed");
        switch_retraction_config.primeSpeed = train.settings.get<Velocity>("switch_extruder_prime_speed");
        switch_retraction_config.zHop = train.settings.get<coord_t>("retraction_hop_after_extruder_switch_height");
        switch_retraction_config.retraction_min_travel_distance = 0; // no limitation on travel distance for an extruder switch retract
        switch_retraction_config.retraction_extrusion_window = 99999.9; // so that extruder switch retractions won't affect the retraction buffer (extruded_volume_at_previous_n_retractions)
        switch_retraction_config.retraction_count_max = 9999999; // extruder switch retraction is never limited
    }
}

void FffGcodeWriter::setConfigWipe(SliceDataStorage& storage)
{
    Scene& scene = Application::getInstance().current_slice->scene;
    for (size_t extruder_index = 0; extruder_index < scene.extruders.size(); extruder_index++)
    {
        ExtruderTrain& train = scene.extruders[extruder_index];
        WipeScriptConfig& wipe_config = storage.wipe_config_per_extruder[extruder_index];

        wipe_config.retraction_enable = train.settings.get<bool>("wipe_retraction_enable");
        wipe_config.retraction_config.distance = train.settings.get<double>("wipe_retraction_amount");
        wipe_config.retraction_config.speed = train.settings.get<Velocity>("wipe_retraction_retract_speed");
        wipe_config.retraction_config.primeSpeed = train.settings.get<Velocity>("wipe_retraction_prime_speed");
        wipe_config.retraction_config.prime_volume = train.settings.get<double>("wipe_retraction_extra_prime_amount");
        wipe_config.retraction_config.retraction_min_travel_distance = 0;
        wipe_config.retraction_config.retraction_extrusion_window = std::numeric_limits<double>::max();
        wipe_config.retraction_config.retraction_count_max = std::numeric_limits<size_t>::max();

        wipe_config.pause = train.settings.get<Duration>("wipe_pause");

        wipe_config.hop_enable = train.settings.get<bool>("wipe_hop_enable");
        wipe_config.hop_amount = train.settings.get<coord_t>("wipe_hop_amount");
        wipe_config.hop_speed = train.settings.get<Velocity>("wipe_hop_speed");

        wipe_config.brush_pos_x = train.settings.get<coord_t>("wipe_brush_pos_x");
        wipe_config.repeat_count = train.settings.get<size_t>("wipe_repeat_count");
        wipe_config.move_distance = train.settings.get<coord_t>("wipe_move_distance");
        wipe_config.move_speed = train.settings.get<Velocity>("speed_travel");
        wipe_config.max_extrusion_mm3 = train.settings.get<double>("max_extrusion_before_wipe");
        wipe_config.clean_between_layers = train.settings.get<bool>("clean_between_layers");
    }
}

unsigned int FffGcodeWriter::getStartExtruder(const SliceDataStorage& storage)
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    size_t start_extruder_nr = mesh_group_settings.get<ExtruderTrain&>("adhesion_extruder_nr").extruder_nr;
    if (mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::NONE)
    {
        std::vector<bool> extruder_is_used = storage.getExtrudersUsed();
        for (size_t extruder_nr = 0; extruder_nr < extruder_is_used.size(); extruder_nr++)
        {
            start_extruder_nr = extruder_nr;
            if (extruder_is_used[extruder_nr])
            {
                break;
            }
        }
    }
    assert(start_extruder_nr < Application::getInstance().current_slice->scene.extruders.size() && "start_extruder_nr must be a valid extruder");
    return start_extruder_nr;
}

void FffGcodeWriter::setInfillAndSkinAngles(SliceMeshStorage& mesh)
{
    if (mesh.infill_angles.size() == 0)
    {
        mesh.infill_angles = mesh.settings.get<std::vector<AngleDegrees>>("infill_angles");
        if (mesh.infill_angles.size() == 0)
        {
            // user has not specified any infill angles so use defaults
            const EFillMethod infill_pattern = mesh.settings.get<EFillMethod>("infill_pattern");
            if (infill_pattern == EFillMethod::CROSS || infill_pattern == EFillMethod::CROSS_3D)
            {
                mesh.infill_angles.push_back(22); // put most infill lines in between 45 and 0 degrees
            }
            else
            {
                mesh.infill_angles.push_back(45); // generally all infill patterns use 45 degrees
                if (infill_pattern == EFillMethod::LINES || infill_pattern == EFillMethod::ZIG_ZAG)
                {
                    // lines and zig zag patterns default to also using 135 degrees
                    mesh.infill_angles.push_back(135);
                }
            }
        }
    }

    if (mesh.roofing_angles.size() == 0)
    {
        mesh.roofing_angles = mesh.settings.get<std::vector<AngleDegrees>>("roofing_angles");
        if (mesh.roofing_angles.size() == 0)
        {
            // user has not specified any infill angles so use defaults
            mesh.roofing_angles.push_back(45);
            mesh.roofing_angles.push_back(135);
        }
    }

    if (mesh.skin_angles.size() == 0)
    {
        mesh.skin_angles = mesh.settings.get<std::vector<AngleDegrees>>("skin_angles");
        if (mesh.skin_angles.size() == 0)
        {
            // user has not specified any infill angles so use defaults
            mesh.skin_angles.push_back(45);
            mesh.skin_angles.push_back(135);
        }
    }
}

void FffGcodeWriter::processInitialLayerTemperature(const SliceDataStorage& storage, const size_t start_extruder_nr)
{
    std::vector<bool> extruder_is_used = storage.getExtrudersUsed();
    Scene& scene = Application::getInstance().current_slice->scene;
    const size_t num_extruders = scene.extruders.size();

    if (gcode.getFlavor() == EGCodeFlavor::GRIFFIN)
    {
        ExtruderTrain& train = scene.extruders[start_extruder_nr];
        constexpr bool wait = true;
        const Temperature print_temp_0 = train.settings.get<Temperature>("material_print_temperature_layer_0");
        const Temperature print_temp_here = (print_temp_0 != 0) ? print_temp_0 : train.settings.get<Temperature>("material_print_temperature");
        gcode.writeTemperatureCommand(start_extruder_nr, print_temp_here, wait);
    }
    else if (gcode.getFlavor() != EGCodeFlavor::ULTIGCODE)
    {
        if (num_extruders > 1 || gcode.getFlavor() == EGCodeFlavor::REPRAP)
        {
            std::ostringstream tmp;
            tmp << "T" << start_extruder_nr;
            gcode.writeLine(tmp.str().c_str());
        }

        if (scene.current_mesh_group->settings.get<bool>("material_bed_temp_prepend"))
        {
            if (scene.current_mesh_group->settings.get<bool>("machine_heated_bed"))
            {
                const Temperature bed_temp = scene.current_mesh_group->settings.get<Temperature>("material_bed_temperature_layer_0");
                if (bed_temp != 0)
                {
                    gcode.writeBedTemperatureCommand(bed_temp, scene.current_mesh_group->settings.get<bool>("material_bed_temp_wait"));
                }
            }
        }

        if (scene.current_mesh_group->settings.get<bool>("material_print_temp_prepend"))
        {
            for (unsigned extruder_nr = 0; extruder_nr < num_extruders; extruder_nr++)
            {
                if (extruder_is_used[extruder_nr])
                {
                    const ExtruderTrain& train = scene.extruders[extruder_nr];
                    Temperature extruder_temp;
                    if (extruder_nr == start_extruder_nr)
                    {
                        const Temperature print_temp_0 = train.settings.get<Temperature>("material_print_temperature_layer_0");
                        extruder_temp = (print_temp_0 != 0) ? print_temp_0 : train.settings.get<Temperature>("material_print_temperature");
                    }
                    else
                    {
                        extruder_temp = train.settings.get<Temperature>("material_standby_temperature");
                    }
                    gcode.writeTemperatureCommand(extruder_nr, extruder_temp);
                }
            }
            if (scene.current_mesh_group->settings.get<bool>("material_print_temp_wait"))
            {
                for (unsigned extruder_nr = 0; extruder_nr < num_extruders; extruder_nr++)
                {
                    if (extruder_is_used[extruder_nr])
                    {
                        const ExtruderTrain& train = scene.extruders[extruder_nr];
                        Temperature extruder_temp;
                        if (extruder_nr == start_extruder_nr)
                        {
                            const Temperature print_temp_0 = train.settings.get<Temperature>("material_print_temperature_layer_0");
                            extruder_temp = (print_temp_0 != 0) ? print_temp_0 : train.settings.get<Temperature>("material_print_temperature");
                        }
                        else
                        {
                            extruder_temp = train.settings.get<Temperature>("material_standby_temperature");
                        }
                        gcode.writeTemperatureCommand(extruder_nr, extruder_temp, true);
                    }
                }
            }
        }
    }
}

void FffGcodeWriter::processStartingCode(const SliceDataStorage& storage, const size_t start_extruder_nr)
{
    std::vector<bool> extruder_is_used = storage.getExtrudersUsed();
    if (Application::getInstance().communication->isSequential()) //If we must output the g-code sequentially, we must already place the g-code header here even if we don't know the exact time/material usages yet.
    {
        std::string prefix = gcode.getFileHeader(extruder_is_used);
        gcode.writeCode(prefix.c_str());
    }

    gcode.writeComment("Generated with Cura_SteamEngine " VERSION);

    if (gcode.getFlavor() == EGCodeFlavor::GRIFFIN)
    {
        std::ostringstream tmp;
        tmp << "T" << start_extruder_nr;
        gcode.writeLine(tmp.str().c_str());
    }
    else
    {
        processInitialLayerTemperature(storage, start_extruder_nr);
    }

    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;

    gcode.writeExtrusionMode(false); // ensure absolute extrusion mode is set before the start gcode
    gcode.writeCode(mesh_group_settings.get<std::string>("machine_start_gcode").c_str());

    Application::getInstance().communication->sendCurrentPosition(gcode.getPositionXY());
    gcode.startExtruder(start_extruder_nr);

    if (gcode.getFlavor() == EGCodeFlavor::BFB)
    {
        gcode.writeComment("enable auto-retraction");
        std::ostringstream tmp;
        tmp << "M227 S" << (mesh_group_settings.get<coord_t>("retraction_amount") * 2560 / 1000) << " P" << (mesh_group_settings.get<coord_t>("retraction_amount") * 2560 / 1000);
        gcode.writeLine(tmp.str().c_str());
    }
    else if (gcode.getFlavor() == EGCodeFlavor::GRIFFIN)
    { // initialize extruder trains
        ExtruderTrain& train = Application::getInstance().current_slice->scene.extruders[start_extruder_nr];
        processInitialLayerTemperature(storage, start_extruder_nr);
        gcode.writePrimeTrain(train.settings.get<Velocity>("speed_travel"));
        extruder_prime_layer_nr[start_extruder_nr] = std::numeric_limits<int>::min(); // set to most negative number so that layer processing never primes this extruder any more.
        const RetractionConfig& retraction_config = storage.retraction_config_per_extruder[start_extruder_nr];
        gcode.writeRetraction(retraction_config);
    }
    if (mesh_group_settings.get<bool>("relative_extrusion"))
    {
        gcode.writeExtrusionMode(true);
    }
    if (gcode.getFlavor() != EGCodeFlavor::GRIFFIN)
    {
        if (mesh_group_settings.get<bool>("retraction_enable"))
        {
            // ensure extruder is zeroed
            gcode.resetExtrusionValue();

            // retract before first travel move
            gcode.writeRetraction(storage.retraction_config_per_extruder[start_extruder_nr]);
        }
    }
    gcode.setExtruderFanNumber(start_extruder_nr);
}

void FffGcodeWriter::processNextMeshGroupCode(const SliceDataStorage& storage)
{
    gcode.writeFanCommand(0);
    gcode.setZ(max_object_height + 5000);

    Application::getInstance().communication->sendCurrentPosition(gcode.getPositionXY());
    gcode.writeTravel(gcode.getPositionXY(), Application::getInstance().current_slice->scene.extruders[gcode.getExtruderNr()].settings.get<Velocity>("speed_travel"));
    Point start_pos(storage.model_min.x, storage.model_min.y);
    gcode.writeTravel(start_pos, Application::getInstance().current_slice->scene.extruders[gcode.getExtruderNr()].settings.get<Velocity>("speed_travel"));

    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    if (mesh_group_settings.get<bool>("machine_heated_bed") && mesh_group_settings.get<Temperature>("material_bed_temperature_layer_0") != 0)
    {
        constexpr bool wait = true;
        gcode.writeBedTemperatureCommand(mesh_group_settings.get<Temperature>("material_bed_temperature_layer_0"), wait);
    }

    processInitialLayerTemperature(storage, gcode.getExtruderNr());
}
    
void FffGcodeWriter::processRaft(const SliceDataStorage& storage)
{
    Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    size_t extruder_nr = mesh_group_settings.get<ExtruderTrain&>("adhesion_extruder_nr").extruder_nr;
    const ExtruderTrain& train = Application::getInstance().current_slice->scene.extruders[extruder_nr];

    coord_t z = 0;
    const LayerIndex initial_raft_layer_nr = -Raft::getTotalExtraLayers();

    // some infill config for all lines infill generation below
    constexpr int offset_from_poly_outline = 0;
    constexpr double fill_overlap = 0; // raft line shouldn't be expanded - there is no boundary polygon printed
    constexpr int infill_multiplier = 1; // rafts use single lines
    constexpr int extra_infill_shift = 0;
    Polygons raft_polygons; // should remain empty, since we only have the lines pattern for the raft...
    std::optional<Point> last_planned_position = std::optional<Point>();

    unsigned int current_extruder_nr = extruder_nr;

    { // raft base layer
        LayerIndex layer_nr = initial_raft_layer_nr;
        const coord_t layer_height = train.settings.get<coord_t>("raft_base_thickness");
        z += layer_height;
        const coord_t comb_offset = train.settings.get<coord_t>("raft_base_line_spacing");

        std::vector<FanSpeedLayerTimeSettings> fan_speed_layer_time_settings_per_extruder_raft_base = fan_speed_layer_time_settings_per_extruder; // copy so that we change only the local copy
        for (FanSpeedLayerTimeSettings& fan_speed_layer_time_settings : fan_speed_layer_time_settings_per_extruder_raft_base)
        {
            double regular_fan_speed = train.settings.get<Ratio>("raft_base_fan_speed") * 100.0;
            fan_speed_layer_time_settings.cool_fan_speed_min = regular_fan_speed;
            fan_speed_layer_time_settings.cool_fan_speed_0 = regular_fan_speed; // ignore initial layer fan speed stuff
        }

        LayerPlan& gcode_layer = *new LayerPlan(storage, layer_nr, z, layer_height, extruder_nr, fan_speed_layer_time_settings_per_extruder_raft_base, comb_offset, train.settings.get<bool>("raft_base_line_width"), train.settings.get<coord_t>("travel_avoid_distance"));
        gcode_layer.setIsInside(true);

        gcode_layer.setExtruder(extruder_nr);

        Application::getInstance().communication->sendLayerComplete(layer_nr, z, layer_height);

        Polygons wall = storage.raftOutline.offset(-gcode_layer.configs_storage.raft_base_config.getLineWidth() / 2);
        wall.simplify(); //Simplify because of a micron-movement created in corners when insetting a polygon that was offset with round joint type.
        gcode_layer.addPolygonsByOptimizer(wall, gcode_layer.configs_storage.raft_base_config);

        Polygons raftLines;
        double fill_angle = 0;
        constexpr bool zig_zaggify_infill = false;
        constexpr bool connect_polygons = true; // causes less jerks, so better adhesion

        constexpr int wall_line_count = 0;
        const Point& infill_origin = Point();
        Polygons* perimeter_gaps = nullptr;
        constexpr bool connected_zigzags = false;
        constexpr bool use_endpieces = true;
        constexpr bool skip_some_zags = false;
        constexpr int zag_skip_count = 0;
        constexpr coord_t pocket_size = 0;
        const coord_t maximum_resolution = train.settings.get<coord_t>("meshfix_maximum_resolution");

        Infill infill_comp(
            EFillMethod::LINES, zig_zaggify_infill, connect_polygons, wall, offset_from_poly_outline, gcode_layer.configs_storage.raft_base_config.getLineWidth(), train.settings.get<coord_t>("raft_base_line_spacing"),
            fill_overlap, infill_multiplier, fill_angle, z, extra_infill_shift,
            wall_line_count, infill_origin, perimeter_gaps, connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count, pocket_size, maximum_resolution
            );
        infill_comp.generate(raft_polygons, raftLines);
        gcode_layer.addLinesByOptimizer(raftLines, gcode_layer.configs_storage.raft_base_config, SpaceFillType::Lines);

        // When we use raft, we need to make sure that all used extruders for this print will get primed on the first raft layer,
        // and then switch back to the original extruder.
        std::vector<size_t> extruder_order = getUsedExtrudersOnLayerExcludingStartingExtruder(storage, extruder_nr, layer_nr);
        for (const size_t to_be_primed_extruder_nr : extruder_order)
        {
            setExtruder_addPrime(storage, gcode_layer, to_be_primed_extruder_nr);
            current_extruder_nr = to_be_primed_extruder_nr;
        }

        layer_plan_buffer.handle(gcode_layer, gcode);
        last_planned_position = gcode_layer.getLastPlannedPositionOrStartingPosition();
    }

    { // raft interface layer
        const LayerIndex layer_nr = initial_raft_layer_nr + 1;
        const coord_t layer_height = train.settings.get<coord_t>("raft_interface_thickness");
        z += layer_height;
        const coord_t comb_offset = train.settings.get<coord_t>("raft_interface_line_spacing");

        std::vector<FanSpeedLayerTimeSettings> fan_speed_layer_time_settings_per_extruder_raft_interface = fan_speed_layer_time_settings_per_extruder; // copy so that we change only the local copy
        for (FanSpeedLayerTimeSettings& fan_speed_layer_time_settings : fan_speed_layer_time_settings_per_extruder_raft_interface)
        {
            double regular_fan_speed = train.settings.get<Ratio>("raft_interface_fan_speed") * 100.0;
            fan_speed_layer_time_settings.cool_fan_speed_min = regular_fan_speed;
            fan_speed_layer_time_settings.cool_fan_speed_0 = regular_fan_speed; // ignore initial layer fan speed stuff
        }

        LayerPlan& gcode_layer = *new LayerPlan(storage, layer_nr, z, layer_height, current_extruder_nr, fan_speed_layer_time_settings_per_extruder_raft_interface, comb_offset, train.settings.get<coord_t>("raft_interface_line_width"), train.settings.get<coord_t>("travel_avoid_distance"));
        gcode_layer.setIsInside(true);

        gcode_layer.setExtruder(extruder_nr); // reset to extruder number, because we might have primed in the last layer
        current_extruder_nr = extruder_nr;

        Application::getInstance().communication->sendLayerComplete(layer_nr, z, layer_height);

        Polygons raft_outline_path = storage.raftOutline.offset(-gcode_layer.configs_storage.raft_interface_config.getLineWidth() / 2); //Do this manually because of micron-movement created in corners when insetting a polygon that was offset with round joint type.
        raft_outline_path.simplify(); //Remove those micron-movements.
        constexpr coord_t infill_outline_width = 0;
        Polygons raftLines;
        int offset_from_poly_outline = 0;
        AngleDegrees fill_angle = train.settings.get<size_t>("raft_surface_layers") > 0 ? 45 : 90;
        constexpr bool zig_zaggify_infill = true;
        constexpr bool connect_polygons = true; // why not?

        constexpr int wall_line_count = 0;
        const Point& infill_origin = Point();
        Polygons* perimeter_gaps = nullptr;
        constexpr bool connected_zigzags = false;
        constexpr bool use_endpieces = true;
        constexpr bool skip_some_zags = false;
        constexpr int zag_skip_count = 0;
        constexpr coord_t pocket_size = 0;
        const coord_t maximum_resolution = train.settings.get<coord_t>("meshfix_maximum_resolution");

        Infill infill_comp(
            EFillMethod::ZIG_ZAG, zig_zaggify_infill, connect_polygons, raft_outline_path, offset_from_poly_outline, infill_outline_width, train.settings.get<coord_t>("raft_interface_line_spacing"),
            fill_overlap, infill_multiplier, fill_angle, z, extra_infill_shift,
            wall_line_count, infill_origin, perimeter_gaps, connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count, pocket_size, maximum_resolution
            );
        infill_comp.generate(raft_polygons, raftLines);
        gcode_layer.addLinesByOptimizer(raftLines, gcode_layer.configs_storage.raft_interface_config, SpaceFillType::Lines, false, 0, 1.0, last_planned_position);

        layer_plan_buffer.handle(gcode_layer, gcode);
        last_planned_position = gcode_layer.getLastPlannedPositionOrStartingPosition();
    }
    
    coord_t layer_height = train.settings.get<coord_t>("raft_surface_thickness");

    for (LayerIndex raft_surface_layer = 1; static_cast<size_t>(raft_surface_layer) <= train.settings.get<size_t>("raft_surface_layers"); raft_surface_layer++)
    { // raft surface layers
        const LayerIndex layer_nr = initial_raft_layer_nr + 2 + raft_surface_layer - 1; // 2: 1 base layer, 1 interface layer
        z += layer_height;
        const coord_t comb_offset = train.settings.get<coord_t>("raft_surface_line_spacing");

        std::vector<FanSpeedLayerTimeSettings> fan_speed_layer_time_settings_per_extruder_raft_surface = fan_speed_layer_time_settings_per_extruder; // copy so that we change only the local copy
        for (FanSpeedLayerTimeSettings& fan_speed_layer_time_settings : fan_speed_layer_time_settings_per_extruder_raft_surface)
        {
            double regular_fan_speed = train.settings.get<Ratio>("raft_surface_fan_speed") * 100.0;
            fan_speed_layer_time_settings.cool_fan_speed_min = regular_fan_speed;
            fan_speed_layer_time_settings.cool_fan_speed_0 = regular_fan_speed; // ignore initial layer fan speed stuff
        }

        LayerPlan& gcode_layer = *new LayerPlan(storage, layer_nr, z, layer_height, extruder_nr, fan_speed_layer_time_settings_per_extruder_raft_surface, comb_offset, train.settings.get<coord_t>("raft_surface_line_width"), train.settings.get<coord_t>("travel_avoid_distance"));
        gcode_layer.setIsInside(true);

        // make sure that we are using the correct extruder to print raft
        gcode_layer.setExtruder(extruder_nr);
        current_extruder_nr = extruder_nr;

        Application::getInstance().communication->sendLayerComplete(layer_nr, z, layer_height);

        const coord_t maximum_resolution = train.settings.get<coord_t>("meshfix_maximum_resolution");
        const coord_t maximum_deviation = train.settings.get<coord_t>("meshfix_maximum_deviation");
        Polygons raft_outline_path = storage.raftOutline.offset(-gcode_layer.configs_storage.raft_surface_config.getLineWidth() / 2); //Do this manually because of micron-movement created in corners when insetting a polygon that was offset with round joint type.
        raft_outline_path.simplify(maximum_resolution, maximum_deviation); //Remove those micron-movements.
        constexpr coord_t infill_outline_width = 0;
        Polygons raft_lines;
        int offset_from_poly_outline = 0;
        AngleDegrees fill_angle = 90 * raft_surface_layer;
        constexpr bool zig_zaggify_infill = true;

        constexpr size_t wall_line_count = 0;
        const Point& infill_origin = Point();
        Polygons* perimeter_gaps = nullptr;
        constexpr bool connected_zigzags = false;
        constexpr bool connect_polygons = false; // midway connections between polygons can make the surface less smooth
        constexpr bool use_endpieces = true;
        constexpr bool skip_some_zags = false;
        constexpr size_t zag_skip_count = 0;
        constexpr coord_t pocket_size = 0;

        Infill infill_comp(
            EFillMethod::ZIG_ZAG, zig_zaggify_infill, connect_polygons, raft_outline_path, offset_from_poly_outline, infill_outline_width, train.settings.get<coord_t>("raft_surface_line_spacing"),
            fill_overlap, infill_multiplier, fill_angle, z, extra_infill_shift,
            wall_line_count, infill_origin, perimeter_gaps, connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count, pocket_size, maximum_resolution
            );
        infill_comp.generate(raft_polygons, raft_lines);
        gcode_layer.addLinesByOptimizer(raft_lines, gcode_layer.configs_storage.raft_surface_config, SpaceFillType::Lines, false, 0, 1.0, last_planned_position);

        layer_plan_buffer.handle(gcode_layer, gcode);
    }
}

LayerPlan& FffGcodeWriter::processLayer(const SliceDataStorage& storage, LayerIndex layer_nr, const size_t total_layers) const
{
    logDebug("GcodeWriter processing layer %i of %i\n", layer_nr, total_layers);

    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    coord_t layer_thickness = mesh_group_settings.get<coord_t>("layer_height");
    coord_t z;
    bool include_helper_parts = true;
    if (layer_nr < 0)
    {
#ifdef DEBUG
        assert(mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::RAFT && "negative layer_number means post-raft, pre-model layer!");
#endif // DEBUG
        const int filler_layer_count = Raft::getFillerLayerCount();
        layer_thickness = Raft::getFillerLayerHeight();
        z = Raft::getTotalThickness() + (filler_layer_count + layer_nr + 1) * layer_thickness;
    }
    else
    {
        z = storage.meshes[0].layers[layer_nr].printZ; // stub default
        // find printZ of first actual printed mesh
        for (const SliceMeshStorage& mesh : storage.meshes)
        {
            if (layer_nr >= static_cast<int>(mesh.layers.size())
                || mesh.settings.get<bool>("support_mesh")
                || mesh.settings.get<bool>("anti_overhang_mesh")
                || mesh.settings.get<bool>("cutting_mesh")
                || mesh.settings.get<bool>("infill_mesh"))
            {
                continue;
            }
            z = mesh.layers[layer_nr].printZ;
            layer_thickness = mesh.layers[layer_nr].thickness;
            break;
        }

        if (layer_nr == 0)
        {
            if (mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::RAFT)
            {
                include_helper_parts = false;
            }
        }
    }

    const Scene& scene = Application::getInstance().current_slice->scene;
#pragma omp critical
    Application::getInstance().communication->sendLayerComplete(layer_nr, z, layer_thickness);

    coord_t avoid_distance = 0; // minimal avoid distance is zero
    const std::vector<bool> extruder_is_used = storage.getExtrudersUsed();
    for (size_t extruder_nr = 0; extruder_nr < scene.extruders.size(); extruder_nr++)
    {
        if (extruder_is_used[extruder_nr])
        {
            const ExtruderTrain& extruder = scene.extruders[extruder_nr];

            if (extruder.settings.get<bool>("travel_avoid_other_parts"))
            {
                avoid_distance = std::max(avoid_distance, extruder.settings.get<coord_t>("travel_avoid_distance"));
            }
        }
    }

    coord_t max_inner_wall_width = 0;
    for (const SliceMeshStorage& mesh : storage.meshes)
    {
        max_inner_wall_width = std::max(max_inner_wall_width, mesh.settings.get<coord_t>((mesh.settings.get<size_t>("wall_line_count") > 1) ? "wall_line_width_x" : "wall_line_width_0"));
        if (layer_nr == 0)
        {
            const ExtruderTrain& train = mesh.settings.get<ExtruderTrain&>((mesh.settings.get<size_t>("wall_line_count") > 1) ? "wall_0_extruder_nr" : "wall_x_extruder_nr");
            max_inner_wall_width *= train.settings.get<Ratio>("initial_layer_line_width_factor");
        }
    }
    const coord_t comb_offset_from_outlines = max_inner_wall_width * 2;

    assert(static_cast<LayerIndex>(extruder_order_per_layer_negative_layers.size()) + layer_nr >= 0 && "Layer numbers shouldn't get more negative than there are raft/filler layers");
    const std::vector<size_t>& extruder_order =
        (layer_nr < 0) ?
        extruder_order_per_layer_negative_layers[extruder_order_per_layer_negative_layers.size() + layer_nr]
        :
        extruder_order_per_layer[layer_nr];

    const coord_t first_outer_wall_line_width = scene.extruders[extruder_order.front()].settings.get<coord_t>("wall_line_width_0");
    LayerPlan& gcode_layer = *new LayerPlan(storage, layer_nr, z, layer_thickness, extruder_order.front(), fan_speed_layer_time_settings_per_extruder, comb_offset_from_outlines, first_outer_wall_line_width, avoid_distance);

    if (include_helper_parts && layer_nr == 0)
    { // process the skirt or the brim of the starting extruder.
        int extruder_nr = gcode_layer.getExtruder();
        if (storage.skirt_brim[extruder_nr].size() > 0)
        {
            processSkirtBrim(storage, gcode_layer, extruder_nr);
        }
    }
    if (include_helper_parts)
    { // handle shield(s) first in a layer so that chances are higher that the other nozzle is wiped (for the ooze shield)
        processOozeShield(storage, gcode_layer);

        processDraftShield(storage, gcode_layer);
    }

    const size_t support_roof_extruder_nr = mesh_group_settings.get<ExtruderTrain&>("support_roof_extruder_nr").extruder_nr;
    const size_t support_bottom_extruder_nr = mesh_group_settings.get<ExtruderTrain&>("support_bottom_extruder_nr").extruder_nr;
    const size_t support_infill_extruder_nr = (layer_nr <= 0) ? mesh_group_settings.get<ExtruderTrain&>("support_extruder_nr_layer_0").extruder_nr : mesh_group_settings.get<ExtruderTrain&>("support_infill_extruder_nr").extruder_nr;
    bool disable_path_optimisation = false;
    
    for (const size_t& extruder_nr : extruder_order)
    {
        if (include_helper_parts
            && (extruder_nr == support_infill_extruder_nr || extruder_nr == support_roof_extruder_nr || extruder_nr == support_bottom_extruder_nr))
        {
            addSupportToGCode(storage, gcode_layer, extruder_nr);
        }

        if (layer_nr >= 0)
        {
            const std::vector<size_t>& mesh_order = mesh_order_per_extruder[extruder_nr];
            for (size_t mesh_idx : mesh_order)
            {
                const SliceMeshStorage& mesh = storage.meshes[mesh_idx];
                const PathConfigStorage::MeshPathConfigs& mesh_config = gcode_layer.configs_storage.mesh_configs[mesh_idx];
                if (mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") == ESurfaceMode::SURFACE)
                {
                    assert(extruder_nr == mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr && "mesh surface mode should always only be printed with the outer wall extruder!");
                    addMeshLayerToGCode_meshSurfaceMode(storage, mesh, mesh_config, gcode_layer);
                }
                else
                {
                    addMeshLayerToGCode(storage, mesh, extruder_nr, mesh_config, gcode_layer);
                }
            }
        }
        // ensure we print the prime tower with this extruder, because the next layer begins with this extruder!
        // If this is not performed, the next layer might get two extruder switches...
        setExtruder_addPrime(storage, gcode_layer, extruder_nr);
    }

    if (include_helper_parts)
    { // add prime tower if it hasn't already been added
        int prev_extruder = gcode_layer.getExtruder(); // most likely the same extruder as we are extruding with now
        addPrimeTower(storage, gcode_layer, prev_extruder);
    }

    if (!disable_path_optimisation)
    {
        gcode_layer.optimizePaths(gcode.getPositionXY());
    }

    return gcode_layer;
}

bool FffGcodeWriter::getExtruderNeedPrimeBlobDuringFirstLayer(const SliceDataStorage& storage, const size_t extruder_nr) const
{
    bool need_prime_blob = false;
    switch (gcode.getFlavor())
    {
        case EGCodeFlavor::GRIFFIN:
            need_prime_blob = true;
            break;
        default:
            need_prime_blob = false; // TODO: change this once priming for other firmware types is implemented
            break;
    }

    // check the settings if the prime blob is disabled
    if (need_prime_blob)
    {
        const bool is_extruder_used_overall = storage.getExtrudersUsed()[extruder_nr];
        const bool extruder_prime_blob_enabled = storage.getExtruderPrimeBlobEnabled(extruder_nr);

        need_prime_blob = is_extruder_used_overall && extruder_prime_blob_enabled;
    }

    return need_prime_blob;
}

void FffGcodeWriter::processSkirtBrim(const SliceDataStorage& storage, LayerPlan& gcode_layer, unsigned int extruder_nr) const
{
    if (gcode_layer.getSkirtBrimIsPlanned(extruder_nr))
    {
        return;
    }
    const Polygons& skirt_brim = storage.skirt_brim[extruder_nr];
    gcode_layer.setSkirtBrimIsPlanned(extruder_nr);
    if (skirt_brim.size() == 0)
    {
        return;
    }
    // Start brim close to the prime location
    const ExtruderTrain& train = Application::getInstance().current_slice->scene.extruders[extruder_nr];
    Point start_close_to;
    if (train.settings.get<bool>("prime_blob_enable"))
    {
        const bool prime_pos_is_abs = train.settings.get<bool>("extruder_prime_pos_abs");
        const Point prime_pos(train.settings.get<coord_t>("extruder_prime_pos_x"), train.settings.get<coord_t>("extruder_prime_pos_y"));
        start_close_to = prime_pos_is_abs ? prime_pos : gcode_layer.getLastPlannedPositionOrStartingPosition() + prime_pos;
    }
    else
    {
        start_close_to = gcode_layer.getLastPlannedPositionOrStartingPosition();
    }
    
    if (train.settings.get<bool>("brim_outside_only"))
    {
        gcode_layer.addTravel(skirt_brim.back().closestPointTo(start_close_to));
        gcode_layer.addPolygonsByOptimizer(skirt_brim, gcode_layer.configs_storage.skirt_brim_config_per_extruder[extruder_nr]);
    }
    else
    {
        Polygons outer_brim, inner_brim;
        for(unsigned int index = 0; index < skirt_brim.size(); index++)
        {
            ConstPolygonRef polygon = skirt_brim[index];
            if(polygon.area() > 0)
            {
                outer_brim.add(polygon);
            }
            else
            {
                inner_brim.add(polygon);
            }
        }
        gcode_layer.addTravel(outer_brim.back().closestPointTo(start_close_to));
        gcode_layer.addPolygonsByOptimizer(outer_brim, gcode_layer.configs_storage.skirt_brim_config_per_extruder[extruder_nr]);
        
        //Add polygon in reverse order
        const coord_t wall_0_wipe_dist = 0;
        const bool spiralize = false;
        const float flow_ratio = 1.0;
        const bool always_retract = false;
        const bool reverse_order = true;
        gcode_layer.addPolygonsByOptimizer(inner_brim, gcode_layer.configs_storage.skirt_brim_config_per_extruder[extruder_nr], nullptr, ZSeamConfig(), wall_0_wipe_dist, spiralize, flow_ratio, always_retract, reverse_order);
    }
}

void FffGcodeWriter::processOozeShield(const SliceDataStorage& storage, LayerPlan& gcode_layer) const
{
    unsigned int layer_nr = std::max(0, gcode_layer.getLayerNr());
    if (layer_nr == 0 && Application::getInstance().current_slice->scene.current_mesh_group->settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::BRIM)
    {
        return; // ooze shield already generated by brim
    }
    if (storage.oozeShield.size() > 0 && layer_nr < storage.oozeShield.size())
    {
        gcode_layer.addPolygonsByOptimizer(storage.oozeShield[layer_nr], gcode_layer.configs_storage.skirt_brim_config_per_extruder[0]);
    }
}

void FffGcodeWriter::processDraftShield(const SliceDataStorage& storage, LayerPlan& gcode_layer) const
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const LayerIndex layer_nr = std::max(0, gcode_layer.getLayerNr());
    if (storage.draft_protection_shield.size() == 0)
    {
        return;
    }
    if (!mesh_group_settings.get<bool>("draft_shield_enabled"))
    {
        return;
    }
    if (layer_nr == 0 && Application::getInstance().current_slice->scene.current_mesh_group->settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::BRIM)
    {
        return; // ooze shield already generated by brim
    }

    if (mesh_group_settings.get<DraftShieldHeightLimitation>("draft_shield_height_limitation") == DraftShieldHeightLimitation::LIMITED)
    {
        const coord_t draft_shield_height = mesh_group_settings.get<coord_t>("draft_shield_height");
        const coord_t layer_height_0 = mesh_group_settings.get<coord_t>("layer_height_0");
        const coord_t layer_height = mesh_group_settings.get<coord_t>("layer_height");
        const LayerIndex max_screen_layer = (draft_shield_height - layer_height_0) / layer_height + 1;
        if (layer_nr > max_screen_layer)
        {
            return;
        }
    }

    gcode_layer.addPolygonsByOptimizer(storage.draft_protection_shield, gcode_layer.configs_storage.skirt_brim_config_per_extruder[0]);
}

void FffGcodeWriter::calculateExtruderOrderPerLayer(const SliceDataStorage& storage)
{
    size_t last_extruder;
    // set the initial extruder of this meshgroup
    Scene& scene = Application::getInstance().current_slice->scene;
    if (scene.current_mesh_group == scene.mesh_groups.begin())
    { // first meshgroup
        last_extruder = getStartExtruder(storage);
    }
    else
    {
        last_extruder = gcode.getExtruderNr();
    }
    for (LayerIndex layer_nr = -Raft::getTotalExtraLayers(); layer_nr < static_cast<LayerIndex>(storage.print_layer_count); layer_nr++)
    {
        std::vector<std::vector<size_t>>& extruder_order_per_layer_here = (layer_nr < 0) ? extruder_order_per_layer_negative_layers : extruder_order_per_layer;
        extruder_order_per_layer_here.push_back(getUsedExtrudersOnLayerExcludingStartingExtruder(storage, last_extruder, layer_nr));
        last_extruder = extruder_order_per_layer_here.back().back();
        extruder_prime_layer_nr[last_extruder] = std::min(extruder_prime_layer_nr[last_extruder], layer_nr);
    }
}

std::vector<size_t> FffGcodeWriter::getUsedExtrudersOnLayerExcludingStartingExtruder(const SliceDataStorage& storage, const size_t start_extruder, const LayerIndex& layer_nr) const
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    size_t extruder_count = Application::getInstance().current_slice->scene.extruders.size();
    assert(static_cast<int>(extruder_count) > 0);
    std::vector<size_t> ret;
    ret.push_back(start_extruder);
    std::vector<bool> extruder_is_used_on_this_layer = storage.getExtrudersUsed(layer_nr);

    //The outermost prime tower extruder is always used if there is a prime tower.
    if (mesh_group_settings.get<bool>("prime_tower_enable") && layer_nr <= storage.max_print_height_second_to_last_extruder)
    {
        extruder_is_used_on_this_layer[storage.primeTower.extruder_order[0]] = true;
    }

    // check if we are on the first layer
    if ((mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::RAFT && layer_nr == -static_cast<LayerIndex>(Raft::getTotalExtraLayers()))
        || (mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") != EPlatformAdhesion::RAFT && layer_nr == 0))
    {
        // check if we need prime blob on the first layer
        for (size_t used_idx = 0; used_idx < extruder_is_used_on_this_layer.size(); used_idx++)
        {
            if (getExtruderNeedPrimeBlobDuringFirstLayer(storage, used_idx))
            {
                extruder_is_used_on_this_layer[used_idx] = true;
            }
        }
    }

    for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        if (extruder_nr == start_extruder)
        { // skip the current extruder, it's the one we started out planning
            continue;
        }
        if (!extruder_is_used_on_this_layer[extruder_nr])
        {
            continue;
        }
        ret.push_back(extruder_nr);
    }
    assert(ret.size() <= (size_t)extruder_count && "Not more extruders may be planned in a layer than there are extruders!");
    return ret;
}

std::vector<size_t> FffGcodeWriter::calculateMeshOrder(const SliceDataStorage& storage, const size_t extruder_nr) const
{
    OrderOptimizer<size_t> mesh_idx_order_optimizer;

    std::vector<MeshGroup>::iterator mesh_group = Application::getInstance().current_slice->scene.current_mesh_group;
    for (unsigned int mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
    {
        const SliceMeshStorage& mesh = storage.meshes[mesh_idx];
        if (mesh.getExtruderIsUsed(extruder_nr))
        {
            const Mesh& mesh_data = mesh_group->meshes[mesh_idx];
            const Point3 middle = (mesh_data.getAABB().min + mesh_data.getAABB().max) / 2;
            mesh_idx_order_optimizer.addItem(Point(middle.x, middle.y), mesh_idx);
        }
    }
    const ExtruderTrain& train = Application::getInstance().current_slice->scene.extruders[extruder_nr];
    const Point layer_start_position(train.settings.get<coord_t>("layer_start_x"), train.settings.get<coord_t>("layer_start_y"));
    std::list<size_t> mesh_indices_order = mesh_idx_order_optimizer.optimize(layer_start_position);

    std::vector<size_t> ret;
    ret.reserve(mesh_indices_order.size());

    for(size_t i: mesh_indices_order)
    {
        const size_t mesh_idx = mesh_idx_order_optimizer.items[i].second;
        ret.push_back(mesh_idx);
    }
    return ret;
}

void FffGcodeWriter::addMeshLayerToGCode_meshSurfaceMode(const SliceDataStorage& storage, const SliceMeshStorage& mesh, const PathConfigStorage::MeshPathConfigs& mesh_config, LayerPlan& gcode_layer) const
{
    if (gcode_layer.getLayerNr() > mesh.layer_nr_max_filled_layer)
    {
        return;
    }

    if (mesh.settings.get<bool>("anti_overhang_mesh") || mesh.settings.get<bool>("support_mesh"))
    {
        return;
    }

    setExtruder_addPrime(storage, gcode_layer, mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr);

    const SliceLayer* layer = &mesh.layers[gcode_layer.getLayerNr()];


    Polygons polygons;
    for (const SliceLayerPart& part : layer->parts)
    {
        polygons.add(part.outline);
    }

    ZSeamConfig z_seam_config(mesh.settings.get<EZSeamType>("z_seam_type"), mesh.getZSeamHint(), mesh.settings.get<EZSeamCornerPrefType>("z_seam_corner"));
    const bool spiralize = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<bool>("magic_spiralize");
    gcode_layer.addPolygonsByOptimizer(polygons, mesh_config.inset0_config, nullptr, z_seam_config, mesh.settings.get<coord_t>("wall_0_wipe_dist"), spiralize);

    addMeshOpenPolyLinesToGCode(mesh, mesh_config, gcode_layer);
}

void FffGcodeWriter::addMeshOpenPolyLinesToGCode(const SliceMeshStorage& mesh, const PathConfigStorage::MeshPathConfigs& mesh_config, LayerPlan& gcode_layer) const
{
    const SliceLayer* layer = &mesh.layers[gcode_layer.getLayerNr()];
    
    Polygons lines;
    for(ConstPolygonRef polyline : layer->openPolyLines)
    {
        for(unsigned int point_idx = 1; point_idx<polyline.size(); point_idx++)
        {
            Polygon p;
            p.add(polyline[point_idx-1]);
            p.add(polyline[point_idx]);
            lines.add(p);
        }
    }
    gcode_layer.addLinesByOptimizer(lines, mesh_config.inset0_config, SpaceFillType::PolyLines);
}

void FffGcodeWriter::addMeshLayerToGCode(const SliceDataStorage& storage, const SliceMeshStorage& mesh, const size_t extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, LayerPlan& gcode_layer) const
{
    if (gcode_layer.getLayerNr() > mesh.layer_nr_max_filled_layer)
    {
        return;
    }

    if (mesh.settings.get<bool>("anti_overhang_mesh")
        || mesh.settings.get<bool>("support_mesh")
    )
    {
        return;
    }

    const SliceLayer& layer = mesh.layers[gcode_layer.getLayerNr()];

    if (layer.parts.size() == 0)
    {
        return;
    }

    const ExtruderTrain& train = Application::getInstance().current_slice->scene.extruders[extruder_nr];
    gcode_layer.setMesh(mesh.mesh_name);

    ZSeamConfig z_seam_config(mesh.settings.get<EZSeamType>("z_seam_type"), mesh.getZSeamHint(), mesh.settings.get<EZSeamCornerPrefType>("z_seam_corner"));
    const Point layer_start_position(train.settings.get<coord_t>("layer_start_x"), train.settings.get<coord_t>("layer_start_y"));
    PathOrderOptimizer part_order_optimizer(layer_start_position, z_seam_config);
    for (unsigned int part_idx = 0; part_idx < layer.parts.size(); part_idx++)
    {
        const SliceLayerPart& part = layer.parts[part_idx];
        ConstPolygonRef part_representative = (part.insets.size() > 0) ? part.insets[0][0] : part.outline[0];
        part_order_optimizer.addPolygon(part_representative);
    }
    part_order_optimizer.optimize();

    for (int part_idx : part_order_optimizer.polyOrder)
    {
        const SliceLayerPart& part = layer.parts[part_idx];
        addMeshPartToGCode(storage, mesh, extruder_nr, mesh_config, part, gcode_layer);
    }
    processIroning(mesh, layer, mesh_config.ironing_config, gcode_layer);
    if (mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::NORMAL && extruder_nr == mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr)
    {
        addMeshOpenPolyLinesToGCode(mesh, mesh_config, gcode_layer);
    }
    gcode_layer.setMesh("NONMESH");
}

void FffGcodeWriter::addMeshPartToGCode(const SliceDataStorage& storage, const SliceMeshStorage& mesh, const size_t extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part, LayerPlan& gcode_layer) const
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;

    bool added_something = false;

    if (mesh.settings.get<bool>("infill_before_walls"))
    {
        added_something = added_something | processInfill(storage, gcode_layer, mesh, extruder_nr, mesh_config, part);
    }

    added_something = added_something | processInsets(storage, gcode_layer, mesh, extruder_nr, mesh_config, part);

    processOutlineGaps(storage, gcode_layer, mesh, extruder_nr, mesh_config, part, added_something);

    if (!mesh.settings.get<bool>("infill_before_walls"))
    {
        added_something = added_something | processInfill(storage, gcode_layer, mesh, extruder_nr, mesh_config, part);
    }

    added_something = added_something | processSkinAndPerimeterGaps(storage, gcode_layer, mesh, extruder_nr, mesh_config, part);

    //After a layer part, make sure the nozzle is inside the comb boundary, so we do not retract on the perimeter.
    if (added_something && (!mesh_group_settings.get<bool>("magic_spiralize") || gcode_layer.getLayerNr() < static_cast<LayerIndex>(mesh.settings.get<size_t>("bottom_layers"))))
    {
        coord_t innermost_wall_line_width = mesh.settings.get<coord_t>((mesh.settings.get<size_t>("wall_line_count") > 1) ? "wall_line_width_x" : "wall_line_width_0");
        if (gcode_layer.getLayerNr() == 0)
        {
            innermost_wall_line_width *= mesh.settings.get<Ratio>("initial_layer_line_width_factor");
        }
        gcode_layer.moveInsideCombBoundary(innermost_wall_line_width);
    }

    gcode_layer.setIsInside(false);
}

bool FffGcodeWriter::processInfill(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const size_t extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part) const
{
    if (extruder_nr != mesh.settings.get<ExtruderTrain&>("infill_extruder_nr").extruder_nr)
    {
        return false;
    }
    if (mesh.settings.get<bool>("spaghetti_infill_enabled"))
    {
        return SpaghettiInfillPathGenerator::processSpaghettiInfill(storage, *this, gcode_layer, mesh, extruder_nr, mesh_config, part);
    }
    else
    {
        bool added_something = processMultiLayerInfill(storage, gcode_layer, mesh, extruder_nr, mesh_config, part);
        added_something = added_something | processSingleLayerInfill(storage, gcode_layer, mesh, extruder_nr, mesh_config, part);
        return added_something;
    }
}

bool FffGcodeWriter::processMultiLayerInfill(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const size_t extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part) const
{
    if (extruder_nr != mesh.settings.get<ExtruderTrain&>("infill_extruder_nr").extruder_nr)
    {
        return false;
    }
    const coord_t infill_line_distance = mesh.settings.get<coord_t>("infill_line_distance");
    if (infill_line_distance <= 0)
    {
        return false;
    }

    const coord_t infill_overlap = mesh.settings.get<coord_t>("infill_overlap_mm");
    AngleDegrees infill_angle = 45; //Original default. This will get updated to an element from mesh->infill_angles.
    if (!mesh.infill_angles.empty())
    {
        const size_t combined_infill_layers = std::max(unsigned(1), round_divide(mesh.settings.get<coord_t>("infill_sparse_thickness"), std::max(mesh.settings.get<coord_t>("layer_height"), coord_t(1))));
        infill_angle = mesh.infill_angles.at((gcode_layer.getLayerNr() / combined_infill_layers) % mesh.infill_angles.size());
    }
    const Point3 mesh_middle = mesh.bounding_box.getMiddle();
    const Point infill_origin(mesh_middle.x + mesh.settings.get<coord_t>("infill_offset_x"), mesh_middle.y + mesh.settings.get<coord_t>("infill_offset_y"));

    //Print the thicker infill lines first. (double or more layer thickness, infill combined with previous layers)
    bool added_something = false;
    for(unsigned int combine_idx = 1; combine_idx < part.infill_area_per_combine_per_density[0].size(); combine_idx++)
    {
        const coord_t infill_line_width = mesh_config.infill_config[combine_idx].getLineWidth();
        const EFillMethod infill_pattern = mesh.settings.get<EFillMethod>("infill_pattern");
        const bool zig_zaggify_infill = mesh.settings.get<bool>("zig_zaggify_infill") || infill_pattern == EFillMethod::ZIG_ZAG;
        const bool connect_polygons = mesh.settings.get<bool>("connect_infill_polygons");
        const size_t infill_multiplier = mesh.settings.get<size_t>("infill_multiplier");
        Polygons infill_polygons;
        Polygons infill_lines;
        for (size_t density_idx = part.infill_area_per_combine_per_density.size() - 1; (int)density_idx >= 0; density_idx--)
        { // combine different density infill areas (for gradual infill)
            size_t density_factor = 2 << density_idx; // == pow(2, density_idx + 1)
            coord_t infill_line_distance_here = infill_line_distance * density_factor; // the highest density infill combines with the next to create a grid with density_factor 1
            coord_t infill_shift = infill_line_distance_here / 2;
            if (density_idx == part.infill_area_per_combine_per_density.size() - 1 || infill_pattern == EFillMethod::CROSS || infill_pattern == EFillMethod::CROSS_3D)
            {
                infill_line_distance_here /= 2;
            }

            constexpr size_t wall_line_count = 0; // wall lines are always single layer
            Polygons* perimeter_gaps = nullptr;
            constexpr bool connected_zigzags = false;
            constexpr bool use_endpieces = true;
            constexpr bool skip_some_zags = false;
            constexpr size_t zag_skip_count = 0;
            const coord_t maximum_resolution = mesh.settings.get<coord_t>("meshfix_maximum_resolution");

            Infill infill_comp(infill_pattern, zig_zaggify_infill, connect_polygons, part.infill_area_per_combine_per_density[density_idx][combine_idx], /*outline_offset =*/ 0
                , infill_line_width, infill_line_distance_here, infill_overlap, infill_multiplier, infill_angle, gcode_layer.z, infill_shift, wall_line_count, infill_origin
                , perimeter_gaps, connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count
                , mesh.settings.get<coord_t>("cross_infill_pocket_size")
                , maximum_resolution);
            infill_comp.generate(infill_polygons, infill_lines, mesh.cross_fill_provider, &mesh);
        }
        if (!infill_lines.empty() || !infill_polygons.empty())
        {
            added_something = true;
            setExtruder_addPrime(storage, gcode_layer, extruder_nr);
            gcode_layer.setIsInside(true); // going to print stuff inside print object
            if (!infill_polygons.empty())
            {
                constexpr bool force_comb_retract = false;
                gcode_layer.addTravel(infill_polygons[0][0], force_comb_retract);
                gcode_layer.addPolygonsByOptimizer(infill_polygons, mesh_config.infill_config[combine_idx]);
            }
            const bool enable_travel_optimization = mesh.settings.get<bool>("infill_enable_travel_optimization");
            gcode_layer.addLinesByOptimizer(infill_lines, mesh_config.infill_config[combine_idx], zig_zaggify_infill ? SpaceFillType::PolyLines : SpaceFillType::Lines, enable_travel_optimization);
        }
    }
    return added_something;
}

bool FffGcodeWriter::processSingleLayerInfill(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const size_t extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part) const
{
    if (extruder_nr != mesh.settings.get<ExtruderTrain&>("infill_extruder_nr").extruder_nr)
    {
        return false;
    }
    const coord_t infill_line_distance = mesh.settings.get<coord_t>("infill_line_distance");
    if (infill_line_distance == 0 || part.infill_area_per_combine_per_density[0].size() == 0)
    {
        return false;
    }
    bool added_something = false;
    const coord_t infill_line_width = mesh_config.infill_config[0].getLineWidth();

    //Combine the 1 layer thick infill with the top/bottom skin and print that as one thing.
    Polygons infill_polygons;
    Polygons infill_lines;

    const EFillMethod pattern = mesh.settings.get<EFillMethod>("infill_pattern");
    const bool zig_zaggify_infill = mesh.settings.get<bool>("zig_zaggify_infill") || pattern == EFillMethod::ZIG_ZAG;
    const bool connect_polygons = mesh.settings.get<bool>("connect_infill_polygons");
    const coord_t infill_overlap = mesh.settings.get<coord_t>("infill_overlap_mm");
    const size_t infill_multiplier = mesh.settings.get<size_t>("infill_multiplier");
    const size_t wall_line_count = mesh.settings.get<size_t>("infill_wall_line_count");
    AngleDegrees infill_angle = 45; //Original default. This will get updated to an element from mesh->infill_angles.
    if (mesh.infill_angles.size() > 0)
    {
        const size_t combined_infill_layers = std::max(unsigned(1), round_divide(mesh.settings.get<coord_t>("infill_sparse_thickness"), std::max(mesh.settings.get<coord_t>("layer_height"), coord_t(1))));
        infill_angle = mesh.infill_angles.at((gcode_layer.getLayerNr() / combined_infill_layers) % mesh.infill_angles.size());
    }
    const Point3 mesh_middle = mesh.bounding_box.getMiddle();
    const Point infill_origin(mesh_middle.x + mesh.settings.get<coord_t>("infill_offset_x"), mesh_middle.y + mesh.settings.get<coord_t>("infill_offset_y"));
    for (unsigned int density_idx = part.infill_area_per_combine_per_density.size() - 1; (int)density_idx >= 0; density_idx--)
    {
        int infill_line_distance_here = infill_line_distance << (density_idx + 1); // the highest density infill combines with the next to create a grid with density_factor 1
        int infill_shift = infill_line_distance_here / 2;
        // infill shift explanation: [>]=shift ["]=line_dist
// :       |       :       |       :       |       :       |         > furthest from top
// :   |   |   |   :   |   |   |   :   |   |   |   :   |   |   |     > further from top
// : | | | | | | | : | | | | | | | : | | | | | | | : | | | | | | |   > near top
// >>"""""
// :       |       :       |       :       |       :       |         > furthest from top
// :   |   |   |   :   |   |   |   :   |   |   |   :   |   |   |     > further from top
// : | | | | | | | : | | | | | | | : | | | | | | | : | | | | | | |   > near top
// >>>>"""""""""
// :       |       :       |       :       |       :       |         > furthest from top
// :   |   |   |   :   |   |   |   :   |   |   |   :   |   |   |     > further from top
// : | | | | | | | : | | | | | | | : | | | | | | | : | | | | | | |   > near top
// >>>>>>>>"""""""""""""""""

        const coord_t maximum_resolution = mesh.settings.get<coord_t>("meshfix_maximum_resolution");

        if (density_idx == part.infill_area_per_combine_per_density.size() - 1 || pattern == EFillMethod::CROSS || pattern == EFillMethod::CROSS_3D)
        { // the least dense infill should fill up all remaining gaps
// :       |       :       |       :       |       :       |       :  > furthest from top
// :   |   |   |   :   |   |   |   :   |   |   |   :   |   |   |   :  > further from top
// : | | | | | | | : | | | | | | | : | | | | | | | : | | | | | | | :  > near top
//   .   .     .       .           .               .       .       .
//   :   :     :       :           :               :       :       :
//   `"""'     `"""""""'           `"""""""""""""""'       `"""""""'
//                                                             ^   new line distance for lowest density infill
//                                       ^ infill_line_distance_here for lowest density infill up till here
//                 ^ middle density line dist
//     ^   highest density line dist
            
            //All of that doesn't hold for the Cross patterns; they should just always be multiplied by 2 for every density index.
            infill_line_distance_here /= 2;
        }

        Polygons in_outline = part.infill_area_per_combine_per_density[density_idx][0];
        const coord_t circumference = in_outline.polygonLength();
        //Originally an area of 0.4*0.4*2 (2 line width squares) was found to be a good threshold for removal.
        //However we found that this doesn't scale well with polygons with larger circumference (https://github.com/Ultimaker/Cura/issues/3992).
        //Given that the original test worked for approximately 2x2cm models, this scaling by circumference should make it work for any size.
        const double minimum_small_area = 0.4 * 0.4 * circumference / 40000;
        
        // This is only for density infill, because after generating the infill might appear unnecessary infill on walls
        // especially on vertical surfaces
        in_outline.removeSmallAreas(minimum_small_area);
        
        Infill infill_comp(pattern, zig_zaggify_infill, connect_polygons, in_outline, /*outline_offset =*/ 0
            , infill_line_width, infill_line_distance_here, infill_overlap, infill_multiplier, infill_angle, gcode_layer.z, infill_shift, wall_line_count, infill_origin
            , /*Polygons* perimeter_gaps =*/ nullptr
            , /*bool connected_zigzags =*/ false
            , /*bool use_endpieces =*/ false
            , /*bool skip_some_zags =*/ false
            , /*int zag_skip_count =*/ 0
            , mesh.settings.get<coord_t>("cross_infill_pocket_size")
            , maximum_resolution);
        infill_comp.generate(infill_polygons, infill_lines, mesh.cross_fill_provider, &mesh);
    }
    if (infill_lines.size() > 0 || infill_polygons.size() > 0)
    {
        added_something = true;
        setExtruder_addPrime(storage, gcode_layer, extruder_nr);
        gcode_layer.setIsInside(true); // going to print stuff inside print object
        if (!infill_polygons.empty())
        {
            constexpr bool force_comb_retract = false;
            gcode_layer.addTravel(infill_polygons[0][0], force_comb_retract);
            gcode_layer.addPolygonsByOptimizer(infill_polygons, mesh_config.infill_config[0]);
        }
        const bool enable_travel_optimization = mesh.settings.get<bool>("infill_enable_travel_optimization");
        if (pattern == EFillMethod::GRID || pattern == EFillMethod::LINES || pattern == EFillMethod::TRIANGLES || pattern == EFillMethod::CUBIC || pattern == EFillMethod::TETRAHEDRAL || pattern == EFillMethod::QUARTER_CUBIC || pattern == EFillMethod::CUBICSUBDIV)
        {
            gcode_layer.addLinesByOptimizer(infill_lines, mesh_config.infill_config[0], SpaceFillType::Lines, enable_travel_optimization, mesh.settings.get<coord_t>("infill_wipe_dist"));
        }
        else
        {
            gcode_layer.addLinesByOptimizer(infill_lines, mesh_config.infill_config[0], (pattern == EFillMethod::ZIG_ZAG) ? SpaceFillType::PolyLines : SpaceFillType::Lines, enable_travel_optimization);
        }
    }
    return added_something;
}

void FffGcodeWriter::processSpiralizedWall(const SliceDataStorage& storage, LayerPlan& gcode_layer, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part) const
{
    if (part.insets.size() == 0 || part.insets[0].size() == 0)
    {
        // wall doesn't have usable outline
        return;
    }
    const ClipperLib::Path* last_wall_outline = &*part.insets[0][0]; // default to current wall outline
    int last_seam_vertex_idx = -1; // last layer seam vertex index
    int layer_nr = gcode_layer.getLayerNr();
    if (layer_nr > 0)
    {
        if (storage.spiralize_wall_outlines[layer_nr - 1] != nullptr)
        {
            // use the wall outline from the previous layer
            last_wall_outline = &*(*storage.spiralize_wall_outlines[layer_nr - 1])[0];
            // and the seam vertex index pre-computed for that layer
            last_seam_vertex_idx = storage.spiralize_seam_vertex_indices[layer_nr - 1];
        }
    }
    ConstPolygonRef wall_outline = part.insets[0][0]; // current layer outer wall outline
    const int seam_vertex_idx = storage.spiralize_seam_vertex_indices[layer_nr]; // use pre-computed seam vertex index for current layer
    // output a wall slice that is interpolated between the last and current walls
    gcode_layer.spiralizeWallSlice(mesh_config.inset0_config, wall_outline, ConstPolygonRef(*last_wall_outline), seam_vertex_idx, last_seam_vertex_idx);
}

bool FffGcodeWriter::processInsets(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const size_t extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part) const
{
    if (extruder_nr != mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr && extruder_nr != mesh.settings.get<ExtruderTrain&>("wall_x_extruder_nr").extruder_nr)
    {
        return false;
    }
    bool added_something = false;
    const bool compensate_overlap_0 = mesh.settings.get<bool>("travel_compensate_overlapping_walls_0_enabled");
    const bool compensate_overlap_x = mesh.settings.get<bool>("travel_compensate_overlapping_walls_x_enabled");
    const bool retract_before_outer_wall = mesh.settings.get<bool>("travel_retract_before_outer_wall");
    if (mesh.settings.get<size_t>("wall_line_count") > 0)
    {
        bool spiralize = false;
        if(Application::getInstance().current_slice->scene.current_mesh_group->settings.get<bool>("magic_spiralize"))
        {
            if (part.insets.size() == 0)
            {
                // nothing to do
                return false;
            }
            const size_t bottom_layers = mesh.settings.get<size_t>("bottom_layers");
            if (gcode_layer.getLayerNr() >= static_cast<LayerIndex>(bottom_layers))
            {
                spiralize = true;
            }
            if (spiralize && gcode_layer.getLayerNr() == static_cast<LayerIndex>(bottom_layers) && !part.insets.empty() && extruder_nr == mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr)
            { // on the last normal layer first make the outer wall normally and then start a second outer wall from the same hight, but gradually moving upward
                added_something = true;
                setExtruder_addPrime(storage, gcode_layer, extruder_nr);
                gcode_layer.setIsInside(true); // going to print stuff inside print object
                WallOverlapComputation* wall_overlap_computation(nullptr);
                int wall_0_wipe_dist(0);
                gcode_layer.addPolygonsByOptimizer(part.insets[0], mesh_config.inset0_config, wall_overlap_computation, ZSeamConfig(), wall_0_wipe_dist);
            }
        }
        // for non-spiralized layers, determine the shape of the unsupported areas below this part
        if (!spiralize && gcode_layer.getLayerNr() > 0)
        {
            // accumulate the outlines of all of the parts that are on the layer below

            Polygons outlines_below;
            AABB boundaryBox(part.outline);
            for (const SliceMeshStorage& m : storage.meshes)
            {
                if (m.isPrinted())
                {
                    for (const SliceLayerPart& prevLayerPart : m.layers[gcode_layer.getLayerNr() - 1].parts)
                    {
                        if (boundaryBox.hit(prevLayerPart.boundaryBox))
                        {
                            outlines_below.add(prevLayerPart.outline);
                        }
                    }
                }
            }

            const coord_t layer_height = mesh_config.inset0_config.getLayerThickness();

            // if support is enabled, add the support outlines also so we don't generate bridges over support

            const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
            if (mesh_group_settings.get<bool>("support_enable") || mesh_group_settings.get<bool>("support_tree_enable"))
            {
                const coord_t z_distance_top = mesh.settings.get<coord_t>("support_top_distance");
                const size_t z_distance_top_layers = round_up_divide(z_distance_top, layer_height) + 1;
                const int support_layer_nr = gcode_layer.getLayerNr() - z_distance_top_layers;

                if (support_layer_nr > 0)
                {
                    const SupportLayer& support_layer = storage.support.supportLayers[support_layer_nr];

                    if (!support_layer.support_roof.empty())
                    {
                        AABB support_roof_bb(support_layer.support_roof);
                        if (boundaryBox.hit(support_roof_bb))
                        {
                            outlines_below.add(support_layer.support_roof);
                        }
                    }
                    else
                    {
                        for (const SupportInfillPart& support_part : support_layer.support_infill_parts)
                        {
                            AABB support_part_bb(support_part.getInfillArea());
                            if (boundaryBox.hit(support_part_bb))
                            {
                                outlines_below.add(support_part.getInfillArea());
                            }
                        }
                    }
                }
            }

            const int half_outer_wall_width = mesh_config.inset0_config.getLineWidth() / 2;

            // remove those parts of the layer below that are narrower than a wall line width as they will not be printed

            outlines_below = outlines_below.offset(-half_outer_wall_width).offset(half_outer_wall_width);

            if (mesh.settings.get<bool>("bridge_settings_enabled"))
            {
                // max_air_gap is the max allowed width of the unsupported region below the wall line
                // if the unsupported region is wider than max_air_gap, the wall line will be printed using bridge settings

                const coord_t max_air_gap = half_outer_wall_width;

                // subtract the outlines of the parts below this part to give the shapes of the unsupported regions and then
                // shrink those shapes so that any that are narrower than two times max_air_gap will be removed

                Polygons compressed_air(part.outline.difference(outlines_below).offset(-max_air_gap));

                // now expand the air regions by the same amount as they were shrunk plus half the outer wall line width
                // which is required because when the walls are being generated, the vertices do not fall on the part's outline
                // but, instead, are 1/2 a line width inset from the outline

                gcode_layer.setBridgeWallMask(compressed_air.offset(max_air_gap + half_outer_wall_width));
            }
            else
            {
                // clear to disable use of bridging settings
                gcode_layer.setBridgeWallMask(Polygons());
            }

            const AngleDegrees overhang_angle = mesh.settings.get<AngleDegrees>("wall_overhang_angle");
            if (overhang_angle >= 90)
            {
                // clear to disable overhang detection
                gcode_layer.setOverhangMask(Polygons());
            }
            else
            {
                // the overhang mask is set to the area of the current part's outline minus the region that is considered to be supported
                // the supported region is made up of those areas that really are supported by either model or support on the layer below
                // expanded to take into account the overhang angle, the greater the overhang angle, the larger the supported area is
                // considered to be
                const coord_t overhang_width = layer_height * std::tan(overhang_angle / (180 / M_PI));
                Polygons overhang_region = part.outline.offset(-half_outer_wall_width).difference(outlines_below.offset(10 + overhang_width - half_outer_wall_width)).offset(10);
                gcode_layer.setOverhangMask(overhang_region);
            }
        }
        else
        {
            // clear to disable use of bridging settings
            gcode_layer.setBridgeWallMask(Polygons());
            // clear to disable overhang detection
            gcode_layer.setOverhangMask(Polygons());
        }

        // Only spiralize the first part in the mesh, any other parts will be printed using the normal, non-spiralize codepath.
        // This sounds weird but actually does the right thing when you have a model that has multiple parts at the bottom that merge into
        // one part higher up. Once all the parts have merged, layers above that level will be spiralized
        if (spiralize && &mesh.layers[gcode_layer.getLayerNr()].parts[0] == &part)
        {
            if (part.insets.size() > 0 && extruder_nr == mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr)
            {
                added_something = true;
                setExtruder_addPrime(storage, gcode_layer, extruder_nr);
                gcode_layer.setIsInside(true); // going to print stuff inside print object
                processSpiralizedWall(storage, gcode_layer, mesh_config, part);
            }
        }
        else if (InsetOrderOptimizer::optimizingInsetsIsWorthwhile(mesh, part))
        {
            InsetOrderOptimizer ioo(*this, storage, gcode_layer, mesh, extruder_nr, mesh_config, part, gcode_layer.getLayerNr());
            return ioo.processInsetsWithOptimizedOrdering();
        }
        else
        {
            const bool outer_inset_first = mesh.settings.get<bool>("outer_inset_first")
                || (gcode_layer.getLayerNr() == 0 && mesh.settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::BRIM);
            int processed_inset_number = -1;
            for (int inset_number = part.insets.size() - 1; inset_number > -1; inset_number--)
            {
                processed_inset_number = inset_number;
                if (outer_inset_first)
                {
                    processed_inset_number = part.insets.size() - 1 - inset_number;
                }
                // Outer wall is processed
                if (processed_inset_number == 0)
                {
                    constexpr float flow = 1.0;
                    if (part.insets[0].size() > 0 && extruder_nr == mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr)
                    {
                        added_something = true;
                        setExtruder_addPrime(storage, gcode_layer, extruder_nr);
                        gcode_layer.setIsInside(true); // going to print stuff inside print object
                        ZSeamConfig z_seam_config(mesh.settings.get<EZSeamType>("z_seam_type"), mesh.getZSeamHint(), mesh.settings.get<EZSeamCornerPrefType>("z_seam_corner"));
                        Polygons outer_wall = part.insets[0];
                        if (!compensate_overlap_0)
                        {
                            WallOverlapComputation* wall_overlap_computation(nullptr);
                            gcode_layer.addWalls(outer_wall, mesh, mesh_config.inset0_config, mesh_config.bridge_inset0_config, wall_overlap_computation, z_seam_config, mesh.settings.get<coord_t>("wall_0_wipe_dist"), flow, retract_before_outer_wall);
                        }
                        else
                        {
                            WallOverlapComputation wall_overlap_computation(outer_wall, mesh_config.inset0_config.getLineWidth());
                            gcode_layer.addWalls(outer_wall, mesh, mesh_config.inset0_config, mesh_config.bridge_inset0_config, &wall_overlap_computation, z_seam_config, mesh.settings.get<coord_t>("wall_0_wipe_dist"), flow, retract_before_outer_wall);
                        }
                    }
                }
                // Inner walls are processed
                else if (!part.insets[processed_inset_number].empty() && extruder_nr == mesh.settings.get<ExtruderTrain&>("wall_x_extruder_nr").extruder_nr)
                {
                    added_something = true;
                    setExtruder_addPrime(storage, gcode_layer, extruder_nr);
                    gcode_layer.setIsInside(true); // going to print stuff inside print object
                    ZSeamConfig z_seam_config(mesh.settings.get<EZSeamType>("z_seam_type"), mesh.getZSeamHint(), mesh.settings.get<EZSeamCornerPrefType>("z_seam_corner"));
                    Polygons inner_wall = part.insets[processed_inset_number];
                    if (!compensate_overlap_x)
                    {
                        WallOverlapComputation* wall_overlap_computation(nullptr);
                        gcode_layer.addWalls(part.insets[processed_inset_number], mesh, mesh_config.insetX_config, mesh_config.bridge_insetX_config, wall_overlap_computation, z_seam_config);
                    }
                    else
                    {
                        WallOverlapComputation wall_overlap_computation(inner_wall, mesh_config.insetX_config.getLineWidth());
                        gcode_layer.addWalls(inner_wall, mesh, mesh_config.insetX_config, mesh_config.bridge_insetX_config, &wall_overlap_computation, z_seam_config);
                    }
                }
            }
        }
    }
    return added_something;
}

std::optional<Point> FffGcodeWriter::getSeamAvoidingLocation(const Polygons& filling_part, int filling_angle, Point last_position) const
{
    if (filling_part.empty())
    {
        return std::optional<Point>();
    }
    // start with the BB of the outline
    AABB skin_part_bb(filling_part);
    PointMatrix rot((double)((-filling_angle + 90) % 360)); // create a matrix to rotate a vector so that it is normal to the skin angle
    const Point bb_middle = skin_part_bb.getMiddle();
    // create a vector from the middle of the BB whose length is such that it can be rotated
    // around the middle of the BB and the end will always be a long way outside of the part's outline
    // and rotate the vector so that it is normal to the skin angle
    const Point vec = rot.apply(Point(0, vSize(skin_part_bb.max - bb_middle) * 100));
    // find the vertex in the outline that is closest to the end of the rotated vector
    const PolygonsPointIndex pa = PolygonUtils::findNearestVert(bb_middle + vec, filling_part);
    // and find another outline vertex, this time using the vector + 180 deg
    const PolygonsPointIndex pb = PolygonUtils::findNearestVert(bb_middle - vec, filling_part);
    if (!pa.initialized() || !pb.initialized())
    {
        return std::optional<Point>();
    }
    // now go to whichever of those vertices that is closest to where we are now
    if (vSize2(pa.p() - last_position) < vSize2(pb.p() - last_position))
    {
        bool bs_arg = true;
        return std::optional<Point>(bs_arg, pa.p());
    }
    else
    {
        bool bs_arg = true;
        return std::optional<Point>(bs_arg, pb.p());
    }
}

void FffGcodeWriter::processOutlineGaps(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const size_t extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part, bool& added_something) const
{
    size_t wall_0_extruder_nr = mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr;
    if (extruder_nr != wall_0_extruder_nr || !mesh.settings.get<bool>("fill_outline_gaps"))
    {
        return;
    }
    const coord_t perimeter_gaps_line_width = mesh_config.perimeter_gap_config.getLineWidth();
    int skin_angle = 45;
    if (mesh.skin_angles.size() > 0)
    {
        skin_angle = mesh.skin_angles.at(gcode_layer.getLayerNr() % mesh.skin_angles.size());
    }
    Polygons gap_polygons; // unused
    Polygons gap_lines; // soon to be generated gap filler lines
    int offset = 0;
    int extra_infill_shift = 0;
    constexpr coord_t outline_gap_overlap = 0;
    constexpr int infill_multiplier = 1;
    constexpr bool zig_zaggify_infill = false;
    constexpr bool connect_polygons = false; // not applicable

    constexpr int wall_line_count = 0;
    const Point& infill_origin = Point();
    Polygons* perimeter_gaps = nullptr;
    constexpr bool connected_zigzags = false;
    constexpr bool use_endpieces = true;
    constexpr bool skip_some_zags = false;
    constexpr int zag_skip_count = 0;
    constexpr coord_t pocket_size = 0;
    const coord_t maximum_resolution = mesh.settings.get<coord_t>("meshfix_maximum_resolution");

    Infill infill_comp(
        EFillMethod::LINES, zig_zaggify_infill, connect_polygons, part.outline_gaps, offset, perimeter_gaps_line_width, perimeter_gaps_line_width, outline_gap_overlap, infill_multiplier, skin_angle, gcode_layer.z, extra_infill_shift,
        wall_line_count, infill_origin, perimeter_gaps, connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count, pocket_size, maximum_resolution
        );
    infill_comp.generate(gap_polygons, gap_lines);

    if (gap_lines.size() > 0)
    {
        assert(extruder_nr == wall_0_extruder_nr); // Should already be the case because of fill_perimeter_gaps check
        added_something = true;
        setExtruder_addPrime(storage, gcode_layer, extruder_nr);
        gcode_layer.setIsInside(false); // going to print stuff outside print object
        gcode_layer.addLinesByOptimizer(gap_lines, mesh_config.perimeter_gap_config, SpaceFillType::Lines);
    }
}

bool FffGcodeWriter::processSkinAndPerimeterGaps(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const size_t extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part) const
{
    const size_t top_bottom_extruder_nr = mesh.settings.get<ExtruderTrain&>("top_bottom_extruder_nr").extruder_nr;
    const size_t roofing_extruder_nr = mesh.settings.get<ExtruderTrain&>("roofing_extruder_nr").extruder_nr;
    const size_t wall_0_extruder_nr = mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr;
    if (extruder_nr != top_bottom_extruder_nr && extruder_nr != wall_0_extruder_nr
        && (extruder_nr != roofing_extruder_nr || mesh.settings.get<size_t>("roofing_layer_count") <= 0))
    {
        return false;
    }
    bool added_something = false;

    const bool fill_perimeter_gaps = mesh.settings.get<FillPerimeterGapMode>("fill_perimeter_gaps") != FillPerimeterGapMode::NOWHERE
                            && !Application::getInstance().current_slice->scene.current_mesh_group->settings.get<bool>("magic_spiralize")
                            && extruder_nr == wall_0_extruder_nr;

    PathOrderOptimizer part_order_optimizer(gcode_layer.getLastPlannedPositionOrStartingPosition());
    for (unsigned int skin_part_idx = 0; skin_part_idx < part.skin_parts.size(); skin_part_idx++)
    {
        const PolygonsPart& outline = part.skin_parts[skin_part_idx].outline;
        part_order_optimizer.addPolygon(outline.outerPolygon());
    }
    part_order_optimizer.optimize();

    for (int ordered_skin_part_idx : part_order_optimizer.polyOrder)
    {
        const SkinPart& skin_part = part.skin_parts[ordered_skin_part_idx];

        processSkinInsets(storage, gcode_layer, mesh, extruder_nr, mesh_config, skin_part, added_something);

        added_something = added_something |
            processSkinPart(storage, gcode_layer, mesh, extruder_nr, mesh_config, skin_part);
    }

    if (fill_perimeter_gaps)
    { // handle perimeter gaps of normal insets
        assert(extruder_nr == wall_0_extruder_nr); // Should already be the case because of fill_perimeter_gaps check
        processPerimeterGaps(storage, gcode_layer, mesh, extruder_nr, part.perimeter_gaps, mesh_config.perimeter_gap_config, added_something);
    }
    return added_something;
}

bool FffGcodeWriter::processSkinPart(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const size_t extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SkinPart& skin_part) const
{
    bool added_something = false;

    gcode_layer.mode_skip_agressive_merge = true;

    // add roofing
    Polygons roofing_concentric_perimeter_gaps; // the perimeter gaps of the insets of concentric skin pattern of this skin part
    processRoofing(storage, gcode_layer, mesh, extruder_nr, mesh_config, skin_part, roofing_concentric_perimeter_gaps, added_something);

    // add normal skinfill
    Polygons top_bottom_concentric_perimeter_gaps; // the perimeter gaps of the insets of concentric skin pattern of this skin part
    processTopBottom(storage, gcode_layer, mesh, extruder_nr, mesh_config, skin_part, top_bottom_concentric_perimeter_gaps, added_something);

    // handle perimeter_gaps of concentric skin
    {
        Polygons perimeter_gaps = top_bottom_concentric_perimeter_gaps;
        perimeter_gaps.add(roofing_concentric_perimeter_gaps);
        if (extruder_nr == mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr)
        {
            perimeter_gaps.add(skin_part.perimeter_gaps);
        }
        perimeter_gaps.unionPolygons();

        processPerimeterGaps(storage, gcode_layer, mesh, extruder_nr, perimeter_gaps, mesh_config.perimeter_gap_config, added_something);
    }

    gcode_layer.mode_skip_agressive_merge = false;
    return added_something;
}

void FffGcodeWriter::processSkinInsets(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const size_t extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SkinPart& skin_part, bool& added_something) const
{
    const size_t skin_extruder_nr = mesh.settings.get<ExtruderTrain&>("top_bottom_extruder_nr").extruder_nr;
    // add skin walls aka skin perimeters
    if (extruder_nr == skin_extruder_nr)
    {
        for (const Polygons& skin_perimeter : skin_part.insets)
        {
            if (skin_perimeter.size() > 0)
            {
                added_something = true;
                setExtruder_addPrime(storage, gcode_layer, extruder_nr);
                gcode_layer.setIsInside(true); // going to print stuff inside print object
                gcode_layer.addWalls(skin_perimeter, mesh, mesh_config.skin_config, mesh_config.bridge_skin_config, nullptr); // add polygons to gcode in inward order
            }
        }
    }
}

void FffGcodeWriter::processRoofing(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const size_t extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SkinPart& skin_part, Polygons& concentric_perimeter_gaps, bool& added_something) const
{
    const size_t roofing_extruder_nr = mesh.settings.get<ExtruderTrain&>("roofing_extruder_nr").extruder_nr;
    if (extruder_nr != roofing_extruder_nr)
    {
        return;
    }

    const bool fill_perimeter_gaps =
        mesh.settings.get<FillPerimeterGapMode>("fill_perimeter_gaps") != FillPerimeterGapMode::NOWHERE
        && !Application::getInstance().current_slice->scene.current_mesh_group->settings.get<bool>("magic_spiralize");

    const EFillMethod pattern = mesh.settings.get<EFillMethod>("roofing_pattern");

    AngleDegrees roofing_angle = 45;
    if (mesh.roofing_angles.size() > 0)
    {
        roofing_angle = mesh.roofing_angles.at(gcode_layer.getLayerNr() % mesh.roofing_angles.size());
    }

    const Ratio skin_density = 1.0;
    const coord_t skin_overlap = 0; // skinfill already expanded over the roofing areas; don't overlap with perimeters
    Polygons* perimeter_gaps_output = (fill_perimeter_gaps) ? &concentric_perimeter_gaps : nullptr;
    processSkinPrintFeature(storage, gcode_layer, mesh, extruder_nr, skin_part.roofing_fill, mesh_config.roofing_config, pattern, roofing_angle, skin_overlap, skin_density, perimeter_gaps_output, added_something);
}

void FffGcodeWriter::processTopBottom(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const size_t extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SkinPart& skin_part, Polygons& concentric_perimeter_gaps, bool& added_something) const
{
    const size_t top_bottom_extruder_nr = mesh.settings.get<ExtruderTrain&>("top_bottom_extruder_nr").extruder_nr;
    if (extruder_nr != top_bottom_extruder_nr)
    {
        return;
    }
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;

    const bool generate_perimeter_gaps =
        mesh.settings.get<FillPerimeterGapMode>("fill_perimeter_gaps") != FillPerimeterGapMode::NOWHERE
        && !mesh_group_settings.get<bool>("magic_spiralize");

    const size_t layer_nr = gcode_layer.getLayerNr();

    EFillMethod pattern = (layer_nr == 0) ?
        mesh.settings.get<EFillMethod>("top_bottom_pattern_0") :
        mesh.settings.get<EFillMethod>("top_bottom_pattern");

    AngleDegrees skin_angle = 45;
    const bool skin_alternate_rotation = mesh.settings.get<bool>("skin_alternate_rotation") && (mesh.settings.get<size_t>("top_layers") >= 4 || mesh.settings.get<size_t>("bottom_layers") >= 4 );
    if (mesh.skin_angles.size() > 0)
    {
        skin_angle = mesh.skin_angles.at(layer_nr % mesh.skin_angles.size());
    }
    if (skin_alternate_rotation && (layer_nr / 2 ) & 1)
    {
        skin_angle -= 45;
    }

    // generate skin_polygons and skin_lines (and concentric_perimeter_gaps if needed)
    const GCodePathConfig* skin_config = &mesh_config.skin_config;
    Ratio skin_density = 1.0;
    coord_t skin_overlap = mesh.settings.get<coord_t>("skin_overlap_mm");
    const coord_t more_skin_overlap = std::max(skin_overlap, (coord_t)(mesh_config.insetX_config.getLineWidth() / 2)); // force a minimum amount of skin_overlap
    const bool bridge_settings_enabled = mesh.settings.get<bool>("bridge_settings_enabled");
    const bool bridge_enable_more_layers = bridge_settings_enabled && mesh.settings.get<bool>("bridge_enable_more_layers");
    const Ratio support_threshold = bridge_settings_enabled ? mesh.settings.get<Ratio>("bridge_skin_support_threshold") : 0.0_r;
    const size_t bottom_layers = mesh.settings.get<size_t>("bottom_layers");

    // if support is enabled, consider the support outlines so we don't generate bridges over support

    int support_layer_nr = -1;
    const SupportLayer* support_layer = nullptr;

    if (mesh_group_settings.get<bool>("support_enable") || mesh_group_settings.get<bool>("support_tree_enable"))
    {
        const coord_t layer_height = mesh_config.inset0_config.getLayerThickness();
        const coord_t z_distance_top = mesh.settings.get<coord_t>("support_top_distance");
        const size_t z_distance_top_layers = round_up_divide(z_distance_top, layer_height) + 1;
        support_layer_nr = layer_nr - z_distance_top_layers;
    }

    // helper function that detects skin regions that have no support and modifies their print settings (config, line angle, density, etc.)

    auto handle_bridge_skin = [&](const int bridge_layer, const GCodePathConfig* config, const float density) // bridge_layer = 1, 2 or 3
    {
        if (support_layer_nr >= (bridge_layer - 1))
        {
            support_layer = &storage.support.supportLayers[support_layer_nr - (bridge_layer - 1)];
        }

        // for upper bridge skins, outline used is union of current skin part and those skin parts from the 1st bridge layer that overlap the curent skin part

        // this is done because if we only use skin_part.outline for this layer and that outline is different (i.e. smaller) than
        // the skin outline used to compute the bridge angle for the first skin, the angle computed for this (second) skin could
        // be different and we would prefer it to be the same as computed for the first bridge layer
        Polygons skin_outline(skin_part.outline);

        if (bridge_layer > 1)
        {
            for (const SliceLayerPart& layer_part : mesh.layers[layer_nr - (bridge_layer - 1)].parts)
            {
                for (const SkinPart& other_skin_part : layer_part.skin_parts)
                {
                    if (PolygonUtils::polygonsIntersect(skin_part.outline.outerPolygon(), other_skin_part.outline.outerPolygon()))
                    {
                        skin_outline = skin_outline.unionPolygons(other_skin_part.outline);
                    }
                }
            }
        }

        Polygons supported_skin_part_regions;

        const int angle = bridgeAngle(mesh.settings, skin_part.outline, storage, layer_nr - bridge_layer, support_layer, supported_skin_part_regions);

        if (angle > -1 || (supported_skin_part_regions.area() / (skin_part.outline.area() + 1) < support_threshold))
        {
            if (angle > -1)
            {
                switch (bridge_layer)
                {
                    default:
                    case 1:
                        skin_angle = angle;
                        break;

                    case 2:
                        if (bottom_layers > 2)
                        {
                            // orientate second bridge skin at +45 deg to first
                            skin_angle = angle + 45;
                        }
                        else
                        {
                            // orientate second bridge skin at 90 deg to first
                            skin_angle = angle + 90;
                        }
                        break;

                    case 3:
                        // orientate third bridge skin at 135 (same result as -45) deg to first
                        skin_angle = angle + 135;
                        break;
                }
            }
            pattern = EFillMethod::LINES; // force lines pattern when bridging
            if (bridge_settings_enabled)
            {
                skin_config = config;
                skin_overlap = more_skin_overlap;
                skin_density = density;
            }
            return true;
        }

        return false;
    };

    bool is_bridge_skin = false;
    if (layer_nr > 0)
    {
        is_bridge_skin = handle_bridge_skin(1, &mesh_config.bridge_skin_config, mesh.settings.get<Ratio>("bridge_skin_density"));
    }
    if (bridge_enable_more_layers && !is_bridge_skin && layer_nr > 1 && bottom_layers > 1)
    {
        is_bridge_skin = handle_bridge_skin(2, &mesh_config.bridge_skin_config2, mesh.settings.get<Ratio>("bridge_skin_density_2"));

        if (!is_bridge_skin && layer_nr > 2 && bottom_layers > 2)
        {
            is_bridge_skin = handle_bridge_skin(3, &mesh_config.bridge_skin_config3, mesh.settings.get<Ratio>("bridge_skin_density_3"));
        }
    }

    double fan_speed = GCodePathConfig::FAN_SPEED_DEFAULT;

    if (layer_nr > 0 && skin_config == &mesh_config.skin_config && support_layer_nr >= 0 && mesh.settings.get<bool>("support_fan_enable"))
    {
        // skin isn't a bridge but is it above support and we need to modify the fan speed?

        AABB skin_bb(skin_part.outline);

        support_layer = &storage.support.supportLayers[support_layer_nr];

        bool supported = false;

        if (!support_layer->support_roof.empty())
        {
            AABB support_roof_bb(support_layer->support_roof);
            if (skin_bb.hit(support_roof_bb))
            {
                supported = !skin_part.outline.intersection(support_layer->support_roof).empty();
            }
        }
        else
        {
            for (auto support_part : support_layer->support_infill_parts)
            {
                AABB support_part_bb(support_part.getInfillArea());
                if (skin_bb.hit(support_part_bb))
                {
                    supported = !skin_part.outline.intersection(support_part.getInfillArea()).empty();

                    if (supported)
                    {
                        break;
                    }
                }
            }
        }

        if (supported)
        {
            fan_speed = mesh.settings.get<Ratio>("support_supported_skin_fan_speed") * 100.0;
        }
    }

    // calculate polygons and lines
    Polygons* perimeter_gaps_output = (generate_perimeter_gaps) ? &concentric_perimeter_gaps : nullptr;

    processSkinPrintFeature(storage, gcode_layer, mesh, extruder_nr, skin_part.inner_infill, *skin_config, pattern, skin_angle, skin_overlap, skin_density, perimeter_gaps_output, added_something, fan_speed);
}

void FffGcodeWriter::processSkinPrintFeature(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const size_t extruder_nr, const Polygons& area, const GCodePathConfig& config, EFillMethod pattern, const AngleDegrees skin_angle, const coord_t skin_overlap, const Ratio skin_density, Polygons* perimeter_gaps_output, bool& added_something, double fan_speed) const
{
    Polygons skin_polygons;
    Polygons skin_lines;

    constexpr int infill_multiplier = 1;
    constexpr int extra_infill_shift = 0;
    constexpr int wall_line_count = 0;
    constexpr coord_t offset_from_inner_skin_infill = 0;
    const bool zig_zaggify_infill = pattern == EFillMethod::ZIG_ZAG;
    const bool connect_polygons = mesh.settings.get<bool>("connect_skin_polygons");
    const Point infill_origin;
    constexpr bool connected_zigzags = false;
    constexpr bool use_endpieces = true;
    constexpr bool skip_some_zags = false;
    constexpr int zag_skip_count = 0;
    constexpr coord_t pocket_size = 0;
    const coord_t maximum_resolution = mesh.settings.get<coord_t>("meshfix_maximum_resolution");

    Infill infill_comp(
        pattern, zig_zaggify_infill, connect_polygons, area, offset_from_inner_skin_infill, config.getLineWidth(), config.getLineWidth() / skin_density, skin_overlap, infill_multiplier, skin_angle, gcode_layer.z, extra_infill_shift, wall_line_count, infill_origin, perimeter_gaps_output,
        connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count, pocket_size, maximum_resolution
        );
    infill_comp.generate(skin_polygons, skin_lines);

    // add paths
    if (skin_polygons.size() > 0 || skin_lines.size() > 0)
    {
        added_something = true;
        setExtruder_addPrime(storage, gcode_layer, extruder_nr);
        gcode_layer.setIsInside(true); // going to print stuff inside print object
        if (!skin_polygons.empty())
        {
            constexpr bool force_comb_retract = false;
            gcode_layer.addTravel(skin_polygons[0][0], force_comb_retract);
            gcode_layer.addPolygonsByOptimizer(skin_polygons, config);
        }

        std::optional<Point> near_start_location;
        const EFillMethod pattern = (gcode_layer.getLayerNr() == 0) ?
            mesh.settings.get<EFillMethod>("top_bottom_pattern_0") :
            mesh.settings.get<EFillMethod>("top_bottom_pattern");
        if (pattern == EFillMethod::LINES || pattern == EFillMethod::ZIG_ZAG)
        { // update near_start_location to a location which tries to avoid seams in skin
            near_start_location = getSeamAvoidingLocation(area, skin_angle, gcode_layer.getLastPlannedPositionOrStartingPosition());
        }

        constexpr bool enable_travel_optimization = false;
        constexpr float flow = 1.0;
        if (pattern == EFillMethod::GRID || pattern == EFillMethod::LINES || pattern == EFillMethod::TRIANGLES || pattern == EFillMethod::CUBIC || pattern == EFillMethod::TETRAHEDRAL || pattern == EFillMethod::QUARTER_CUBIC || pattern == EFillMethod::CUBICSUBDIV)
        {
            gcode_layer.addLinesByOptimizer(skin_lines, config, SpaceFillType::Lines, enable_travel_optimization, mesh.settings.get<coord_t>("infill_wipe_dist"), flow, near_start_location, fan_speed);
        }
        else
        {
            SpaceFillType space_fill_type = (pattern == EFillMethod::ZIG_ZAG) ? SpaceFillType::PolyLines : SpaceFillType::Lines;
            constexpr coord_t wipe_dist = 0;
            gcode_layer.addLinesByOptimizer(skin_lines, config, space_fill_type, enable_travel_optimization, wipe_dist, flow, near_start_location, fan_speed);
        }
    }
}

void FffGcodeWriter::processPerimeterGaps(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const size_t extruder_nr, const Polygons& perimeter_gaps, const GCodePathConfig& perimeter_gap_config, bool& added_something) const
{
    const coord_t perimeter_gaps_line_width = perimeter_gap_config.getLineWidth();

    assert(mesh.roofing_angles.size() > 0);
    const bool zig_zaggify_infill = false;
    const bool connect_polygons = false; // not applicable
    int perimeter_gaps_angle = mesh.roofing_angles[gcode_layer.getLayerNr() % mesh.roofing_angles.size()]; // use roofing angles for perimeter gaps
    Polygons gap_polygons; // will remain empty
    Polygons gap_lines;
    constexpr int offset = 0;
    constexpr int infill_multiplier = 1;
    constexpr int extra_infill_shift = 0;
    const coord_t skin_overlap = mesh.settings.get<coord_t>("skin_overlap_mm");
    constexpr int wall_line_count = 0;
    const Point& infill_origin = Point();
    constexpr Polygons* perimeter_gaps_polyons = nullptr;
    constexpr bool connected_zigzags = false;
    constexpr bool use_endpieces = true;
    constexpr bool skip_some_zags = false;
    constexpr int zag_skip_count = 0;
    constexpr coord_t pocket_size = 0;
    const coord_t maximum_resolution = mesh.settings.get<coord_t>("meshfix_maximum_resolution");

    gcode_layer.mode_skip_agressive_merge = false;

    Infill infill_comp(
        EFillMethod::LINES, zig_zaggify_infill, connect_polygons, perimeter_gaps, offset, perimeter_gaps_line_width, perimeter_gaps_line_width, skin_overlap, infill_multiplier, perimeter_gaps_angle, gcode_layer.z, extra_infill_shift,
        wall_line_count, infill_origin, perimeter_gaps_polyons, connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count, pocket_size, maximum_resolution);
    infill_comp.generate(gap_polygons, gap_lines);
    if (gap_lines.size() > 0)
    {
        added_something = true;
        setExtruder_addPrime(storage, gcode_layer, extruder_nr);
        gcode_layer.setIsInside(true); // going to print stuff inside print object
        gcode_layer.addLinesByOptimizer(gap_lines, perimeter_gap_config, SpaceFillType::Lines);
    }

    gcode_layer.mode_skip_agressive_merge = true;
}

bool FffGcodeWriter::processIroning(const SliceMeshStorage& mesh, const SliceLayer& layer, const GCodePathConfig& line_config, LayerPlan& gcode_layer) const
{
    bool added_something = false;
    const bool ironing_enabled = mesh.settings.get<bool>("ironing_enabled");
    const bool ironing_only_highest_layer = mesh.settings.get<bool>("ironing_only_highest_layer");
    if (ironing_enabled && (!ironing_only_highest_layer || mesh.layer_nr_max_filled_layer == gcode_layer.getLayerNr()))
    {
        added_something |= layer.top_surface.ironing(mesh, line_config, gcode_layer);
    }
    return added_something;
}


bool FffGcodeWriter::addSupportToGCode(const SliceDataStorage& storage, LayerPlan& gcode_layer, const size_t extruder_nr) const
{
    bool support_added = false;
    if (!storage.support.generated || gcode_layer.getLayerNr() > storage.support.layer_nr_max_filled_layer)
    {
        return support_added;
    }

    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const size_t support_roof_extruder_nr = mesh_group_settings.get<ExtruderTrain&>("support_roof_extruder_nr").extruder_nr;
    const size_t support_bottom_extruder_nr = mesh_group_settings.get<ExtruderTrain&>("support_bottom_extruder_nr").extruder_nr;
    size_t support_infill_extruder_nr = (gcode_layer.getLayerNr() <= 0) ? mesh_group_settings.get<ExtruderTrain&>("support_extruder_nr_layer_0").extruder_nr : mesh_group_settings.get<ExtruderTrain&>("support_infill_extruder_nr").extruder_nr;

    const SupportLayer& support_layer = storage.support.supportLayers[std::max(0, gcode_layer.getLayerNr())];
    if (support_layer.support_bottom.empty() && support_layer.support_roof.empty() && support_layer.support_infill_parts.empty())
    {
        return support_added;
    }

    if (extruder_nr == support_infill_extruder_nr)
    {
        support_added |= processSupportInfill(storage, gcode_layer);
    }
    if (extruder_nr == support_roof_extruder_nr)
    {
        support_added |= addSupportRoofsToGCode(storage, gcode_layer);
    }
    if (extruder_nr == support_bottom_extruder_nr)
    {
        support_added |= addSupportBottomsToGCode(storage, gcode_layer);
    }
    return support_added;
}


bool FffGcodeWriter::processSupportInfill(const SliceDataStorage& storage, LayerPlan& gcode_layer) const
{
    bool added_something = false;
    const SupportLayer& support_layer = storage.support.supportLayers[std::max(0, gcode_layer.getLayerNr())]; // account for negative layer numbers for raft filler layers

    if (gcode_layer.getLayerNr() > storage.support.layer_nr_max_filled_layer || support_layer.support_infill_parts.empty())
    {
        return added_something;
    }

    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const size_t extruder_nr = (gcode_layer.getLayerNr() <= 0) ? mesh_group_settings.get<ExtruderTrain&>("support_extruder_nr_layer_0").extruder_nr : mesh_group_settings.get<ExtruderTrain&>("support_infill_extruder_nr").extruder_nr;
    const ExtruderTrain& infill_extruder = Application::getInstance().current_slice->scene.extruders[extruder_nr];

    coord_t default_support_line_distance = infill_extruder.settings.get<coord_t>("support_line_distance");
    
    // To improve adhesion for the "support initial layer" the first layer might have different properties
    if(gcode_layer.getLayerNr() == 0)
    {
        default_support_line_distance = infill_extruder.settings.get<coord_t>("support_initial_layer_line_distance"); 
    }

    const coord_t default_support_infill_overlap = infill_extruder.settings.get<coord_t>("infill_overlap_mm");
    const AngleDegrees support_infill_angle = infill_extruder.settings.get<AngleDegrees>("support_infill_angle");
    constexpr size_t infill_multiplier = 1; // there is no frontend setting for this (yet)
    constexpr size_t wall_line_count = 0;
    coord_t default_support_line_width = infill_extruder.settings.get<coord_t>("support_line_width");
    if (gcode_layer.getLayerNr() == 0 && mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") != EPlatformAdhesion::RAFT)
    {
        default_support_line_width *= infill_extruder.settings.get<Ratio>("initial_layer_line_width_factor");
    }

    EFillMethod support_pattern = infill_extruder.settings.get<EFillMethod>("support_pattern");
    if (gcode_layer.getLayerNr() <= 0 && (support_pattern == EFillMethod::LINES || support_pattern == EFillMethod::ZIG_ZAG))
    {
        support_pattern = EFillMethod::GRID;
    }
    const bool zig_zaggify_infill = infill_extruder.settings.get<bool>("zig_zaggify_support");
    constexpr bool connect_polygons = false; // polygons are too distant to connect for sparse support
    const bool skip_some_zags = infill_extruder.settings.get<bool>("support_skip_some_zags");
    const size_t zag_skip_count = infill_extruder.settings.get<size_t>("support_zag_skip_count");

    // create a list of outlines and use PathOrderOptimizer to optimize the travel move
    PathOrderOptimizer island_order_optimizer(gcode_layer.getLastPlannedPositionOrStartingPosition());
    for (size_t part_idx = 0; part_idx < support_layer.support_infill_parts.size(); ++part_idx)
    {
        island_order_optimizer.addPolygon(support_layer.support_infill_parts[part_idx].outline[0]);
    }
    island_order_optimizer.optimize();

    //Print the thicker infill lines first. (double or more layer thickness, infill combined with previous layers)
    const std::vector<SupportInfillPart>& part_list = support_layer.support_infill_parts;
    for (int part_idx : island_order_optimizer.polyOrder)
    {
        const SupportInfillPart& part = part_list[part_idx];

        // always process the wall overlap if walls are generated
        const int current_support_infill_overlap = (part.inset_count_to_generate > 0) ? default_support_infill_overlap : 0;

        // add outline (boundary) if any wall is generated
        if (!part.insets.empty())
        {
            Polygons all_insets;
            for (const Polygons& inset : part.insets)
            {
                all_insets.add(inset);
            }

            if (all_insets.size() > 0)
            {
                setExtruder_addPrime(storage, gcode_layer, extruder_nr); // only switch extruder if we're sure we're going to switch
                gcode_layer.setIsInside(false); // going to print stuff outside print object, i.e. support
                gcode_layer.addPolygonsByOptimizer(all_insets, gcode_layer.configs_storage.support_infill_config[0]);
            }
        }

        // process sub-areas in this support infill area with different densities
        if (default_support_line_distance <= 0
            || part.infill_area_per_combine_per_density.empty())
        {
            continue;
        }

        for (unsigned int combine_idx = 0; combine_idx < part.infill_area_per_combine_per_density[0].size(); ++combine_idx)
        {
            const coord_t support_line_width = default_support_line_width * (combine_idx + 1);

            Polygons support_polygons;
            Polygons support_lines;
            for (unsigned int density_idx = part.infill_area_per_combine_per_density.size() - 1; (int)density_idx >= 0; density_idx--)
            {
                if (combine_idx >= part.infill_area_per_combine_per_density[density_idx].size())
                {
                    continue;
                }
                const Polygons& support_area = part.infill_area_per_combine_per_density[density_idx][combine_idx];

                const unsigned int density_factor = 2 << density_idx; // == pow(2, density_idx + 1)
                int support_line_distance_here = default_support_line_distance * density_factor; // the highest density infill combines with the next to create a grid with density_factor 1
                const int support_shift = support_line_distance_here / 2;
                if (density_idx == part.infill_area_per_combine_per_density.size() - 1 || support_pattern == EFillMethod::CROSS || support_pattern == EFillMethod::CROSS_3D)
                {
                    support_line_distance_here /= 2;
                }

                constexpr coord_t offset_from_outline = 0;
                constexpr bool use_endpieces = true;
                constexpr Polygons* perimeter_gaps = nullptr;
                const Point infill_origin;

                constexpr coord_t pocket_size = 0;
                const coord_t maximum_resolution = infill_extruder.settings.get<coord_t>("meshfix_maximum_resolution");

                Infill infill_comp(support_pattern, zig_zaggify_infill, connect_polygons, support_area, offset_from_outline, support_line_width,
                                   support_line_distance_here, current_support_infill_overlap, infill_multiplier, support_infill_angle, gcode_layer.z, support_shift, wall_line_count, infill_origin,
                                   perimeter_gaps, infill_extruder.settings.get<bool>("support_connect_zigzags"), use_endpieces,
                                   skip_some_zags, zag_skip_count, pocket_size, maximum_resolution);
                infill_comp.generate(support_polygons, support_lines, storage.support.cross_fill_provider);
            }

            if (support_lines.size() > 0 || support_polygons.size() > 0)
            {
                setExtruder_addPrime(storage, gcode_layer, extruder_nr); // only switch extruder if we're sure we're going to switch
                gcode_layer.setIsInside(false); // going to print stuff outside print object, i.e. support
                if (!support_polygons.empty())
                {
                    constexpr bool force_comb_retract = false;
                    gcode_layer.addTravel(support_polygons[0][0], force_comb_retract);
                    gcode_layer.addPolygonsByOptimizer(support_polygons, gcode_layer.configs_storage.support_infill_config[combine_idx]);
                }
                gcode_layer.addLinesByOptimizer(support_lines, gcode_layer.configs_storage.support_infill_config[combine_idx],
                                                (support_pattern == EFillMethod::ZIG_ZAG) ? SpaceFillType::PolyLines : SpaceFillType::Lines);
                added_something = true;
            }
        }
    }

    return added_something;
}


bool FffGcodeWriter::addSupportRoofsToGCode(const SliceDataStorage& storage, LayerPlan& gcode_layer) const
{
    const SupportLayer& support_layer = storage.support.supportLayers[std::max(0, gcode_layer.getLayerNr())];

    if (!storage.support.generated 
        || gcode_layer.getLayerNr() > storage.support.layer_nr_max_filled_layer 
        || support_layer.support_roof.empty())
    {
        return false; //No need to generate support roof if there's no support.
    }

    const size_t roof_extruder_nr = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("support_roof_extruder_nr").extruder_nr;
    const ExtruderTrain& roof_extruder = Application::getInstance().current_slice->scene.extruders[roof_extruder_nr];

    const EFillMethod pattern = roof_extruder.settings.get<EFillMethod>("support_roof_pattern");
    const double fill_angle = supportInterfaceFillAngle(storage, pattern, "support_roof_height", gcode_layer.getLayerNr());
    const bool zig_zaggify_infill = pattern == EFillMethod::ZIG_ZAG;
    const bool connect_polygons = false; // connections might happen in mid air in between the infill lines
    constexpr coord_t support_roof_overlap = 0; // the roofs should never be expanded outwards
    constexpr size_t infill_multiplier = 1;
    constexpr coord_t outline_offset =  0;
    constexpr coord_t extra_infill_shift = 0;
    constexpr size_t wall_line_count = 0;
    const Point infill_origin;
    constexpr Polygons* perimeter_gaps = nullptr;
    constexpr bool use_endpieces = true;
    constexpr bool connected_zigzags = false;
    constexpr bool skip_some_zags = false;
    constexpr size_t zag_skip_count = 0;
    constexpr coord_t pocket_size = 0;
    const coord_t maximum_resolution = roof_extruder.settings.get<coord_t>("meshfix_maximum_resolution");

    coord_t support_roof_line_distance = roof_extruder.settings.get<coord_t>("support_roof_line_distance");
    const coord_t support_roof_line_width = roof_extruder.settings.get<coord_t>("support_roof_line_width");
    if (gcode_layer.getLayerNr() == 0 && support_roof_line_distance < 2 * support_roof_line_width)
    { // if roof is dense
        support_roof_line_distance *= roof_extruder.settings.get<Ratio>("initial_layer_line_width_factor");
    }

    Polygons infill_outline = support_layer.support_roof;
    Polygons wall;
    // make sure there is a wall if this is on the first layer
    if (gcode_layer.getLayerNr() == 0)
    {
        wall = support_layer.support_roof.offset(-support_roof_line_width / 2);
        infill_outline = wall.offset(-support_roof_line_width / 2);
    }

    Infill roof_computation(
        pattern, zig_zaggify_infill, connect_polygons, infill_outline, outline_offset, gcode_layer.configs_storage.support_roof_config.getLineWidth(),
        support_roof_line_distance, support_roof_overlap, infill_multiplier, fill_angle, gcode_layer.z, extra_infill_shift,
        wall_line_count, infill_origin, perimeter_gaps, connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count, pocket_size, maximum_resolution
        );
    Polygons roof_polygons;
    Polygons roof_lines;
    roof_computation.generate(roof_polygons, roof_lines);
    if ((gcode_layer.getLayerNr() == 0 && wall.empty()) || (gcode_layer.getLayerNr() > 0 && roof_polygons.empty() && roof_lines.empty()))
    {
        return false; //We didn't create any support roof.
    }
    setExtruder_addPrime(storage, gcode_layer, roof_extruder_nr);
    gcode_layer.setIsInside(false); // going to print stuff outside print object, i.e. support
    if (gcode_layer.getLayerNr() == 0)
    {
        gcode_layer.addPolygonsByOptimizer(wall, gcode_layer.configs_storage.support_roof_config);
    }
    if (!roof_polygons.empty())
    {
        constexpr bool force_comb_retract = false;
        gcode_layer.addTravel(roof_polygons[0][0], force_comb_retract);
        gcode_layer.addPolygonsByOptimizer(roof_polygons, gcode_layer.configs_storage.support_roof_config);
    }
    gcode_layer.addLinesByOptimizer(roof_lines, gcode_layer.configs_storage.support_roof_config, (pattern == EFillMethod::ZIG_ZAG) ? SpaceFillType::PolyLines : SpaceFillType::Lines);
    return true;
}

bool FffGcodeWriter::addSupportBottomsToGCode(const SliceDataStorage& storage, LayerPlan& gcode_layer) const
{
    const SupportLayer& support_layer = storage.support.supportLayers[std::max(0, gcode_layer.getLayerNr())];

    if (!storage.support.generated 
        || gcode_layer.getLayerNr() > storage.support.layer_nr_max_filled_layer 
        || support_layer.support_bottom.empty())
    {
        return false; //No need to generate support bottoms if there's no support.
    }

    const size_t bottom_extruder_nr = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("support_bottom_extruder_nr").extruder_nr;
    const ExtruderTrain& bottom_extruder = Application::getInstance().current_slice->scene.extruders[bottom_extruder_nr];

    const EFillMethod pattern = bottom_extruder.settings.get<EFillMethod>("support_bottom_pattern");
    const AngleDegrees fill_angle = supportInterfaceFillAngle(storage, pattern, "support_bottom_height", gcode_layer.getLayerNr());
    const bool zig_zaggify_infill = pattern == EFillMethod::ZIG_ZAG;
    const bool connect_polygons = true; // less retractions and less moves only make the bottoms easier to print
    constexpr coord_t support_bottom_overlap = 0; // the bottoms should never be expanded outwards
    constexpr size_t infill_multiplier = 1;
    constexpr coord_t outline_offset =  0;
    constexpr coord_t extra_infill_shift = 0;
    constexpr size_t wall_line_count = 0;
    const Point infill_origin;
    constexpr Polygons* perimeter_gaps = nullptr;
    constexpr bool use_endpieces = true;
    constexpr bool connected_zigzags = false;
    constexpr bool skip_some_zags = false;
    constexpr int zag_skip_count = 0;
    constexpr coord_t pocket_size = 0;
    const coord_t maximum_resolution = bottom_extruder.settings.get<coord_t>("meshfix_maximum_resolution");

    const coord_t support_bottom_line_distance = bottom_extruder.settings.get<coord_t>("support_bottom_line_distance"); // note: no need to apply initial line width factor; support bottoms cannot exist on the first layer
    Infill bottom_computation(
        pattern, zig_zaggify_infill, connect_polygons, support_layer.support_bottom, outline_offset, gcode_layer.configs_storage.support_bottom_config.getLineWidth(),
        support_bottom_line_distance, support_bottom_overlap, infill_multiplier, fill_angle, gcode_layer.z, extra_infill_shift,
        wall_line_count, infill_origin, perimeter_gaps, connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count, pocket_size, maximum_resolution
        );
    Polygons bottom_polygons;
    Polygons bottom_lines;
    bottom_computation.generate(bottom_polygons, bottom_lines);
    if (bottom_polygons.empty() && bottom_lines.empty())
    {
        return false;
    }
    setExtruder_addPrime(storage, gcode_layer, bottom_extruder_nr);
    gcode_layer.setIsInside(false); // going to print stuff outside print object, i.e. support
    if (!bottom_polygons.empty())
    {
        constexpr bool force_comb_retract = false;
        gcode_layer.addTravel(bottom_polygons[0][0], force_comb_retract);
        gcode_layer.addPolygonsByOptimizer(bottom_polygons, gcode_layer.configs_storage.support_bottom_config);
    }
    gcode_layer.addLinesByOptimizer(bottom_lines, gcode_layer.configs_storage.support_bottom_config, (pattern == EFillMethod::ZIG_ZAG) ? SpaceFillType::PolyLines : SpaceFillType::Lines);
    return true;
}

AngleDegrees FffGcodeWriter::supportInterfaceFillAngle(const SliceDataStorage& storage, const EFillMethod pattern, const std::string interface_height_setting, const LayerIndex layer_nr) const
{
    if (pattern == EFillMethod::CONCENTRIC)
    {
        return 0; //Concentric has no rotation.
    }
    if (pattern == EFillMethod::TRIANGLES)
    {
        return 90; //Triangular support interface shouldn't alternate every layer.
    }

    for (const SliceMeshStorage& mesh : storage.meshes)
    {
        if (mesh.settings.get<coord_t>(interface_height_setting) >= 2 * Application::getInstance().current_slice->scene.current_mesh_group->settings.get<coord_t>("layer_height"))
        {
            //Some roofs are quite thick.
            //Alternate between the two kinds of diagonal: / and \ .
            // + 2) % 2 is to handle negative layer numbers.
            return 45 + (((layer_nr % 2) + 2) % 2) * 90;
        }
    }

    return 90; //Perpendicular to support lines.
}

void FffGcodeWriter::setExtruder_addPrime(const SliceDataStorage& storage, LayerPlan& gcode_layer, const size_t extruder_nr) const
{
    const size_t outermost_prime_tower_extruder = storage.primeTower.extruder_order[0];

    const size_t previous_extruder = gcode_layer.getExtruder();
    if (previous_extruder == extruder_nr && !(extruder_nr == outermost_prime_tower_extruder
            && gcode_layer.getLayerNr() >= -static_cast<LayerIndex>(Raft::getFillerLayerCount()))) //No unnecessary switches, unless switching to extruder for the outer shell of the prime tower.
    {
        return;
    }
    const bool extruder_changed = gcode_layer.setExtruder(extruder_nr);

    if (extruder_changed)
    {
        if (extruder_prime_layer_nr[extruder_nr] == gcode_layer.getLayerNr())
        {
            const ExtruderTrain& train = Application::getInstance().current_slice->scene.extruders[extruder_nr];

            // We always prime an extruder, but whether it will be a prime blob/poop depends on if prime blob is enabled.
            // This is decided in GCodeExport::writePrimeTrain().
            if (train.settings.get<bool>("prime_blob_enable")) // Don't travel to the prime-blob position if not enabled though.
            {
                bool prime_pos_is_abs = train.settings.get<bool>("extruder_prime_pos_abs");
                Point prime_pos = Point(train.settings.get<coord_t>("extruder_prime_pos_x"), train.settings.get<coord_t>("extruder_prime_pos_y"));
                gcode_layer.addTravel(prime_pos_is_abs ? prime_pos : gcode_layer.getLastPlannedPositionOrStartingPosition() + prime_pos);
            }

            gcode_layer.planPrime();
        }

        if (gcode_layer.getLayerNr() == 0 && !gcode_layer.getSkirtBrimIsPlanned(extruder_nr))
        {
            processSkirtBrim(storage, gcode_layer, extruder_nr);
        }
    }

    // The first layer of the prime tower is printed with one material only, so do not prime another material on the
    // first layer again.
    if (((extruder_changed && gcode_layer.getLayerNr() > 0) || extruder_nr == outermost_prime_tower_extruder) && gcode_layer.getLayerNr() >= -static_cast<LayerIndex>(Raft::getFillerLayerCount())) //Always print a prime tower with outermost extruder.
    {
        addPrimeTower(storage, gcode_layer, previous_extruder);
    }
}

void FffGcodeWriter::addPrimeTower(const SliceDataStorage& storage, LayerPlan& gcode_layer, int prev_extruder) const
{
    if (!Application::getInstance().current_slice->scene.current_mesh_group->settings.get<bool>("prime_tower_enable"))
    {
        return;
    }

    const size_t outermost_prime_tower_extruder = storage.primeTower.extruder_order[0];
    if (gcode_layer.getLayerNr() == 0 && outermost_prime_tower_extruder != gcode_layer.getExtruder())
    {
        return;
    }

    storage.primeTower.addToGcode(storage, gcode_layer, prev_extruder, gcode_layer.getExtruder());
}

void FffGcodeWriter::finalize()
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    if (mesh_group_settings.get<bool>("machine_heated_bed"))
    {
        gcode.writeBedTemperatureCommand(0); //Cool down the bed (M140).
        //Nozzles are cooled down automatically after the last time they are used (which might be earlier than the end of the print).
    }

    const Duration print_time = gcode.getSumTotalPrintTimes();
    std::vector<double> filament_used;
    std::vector<std::string> material_ids;
    std::vector<bool> extruder_is_used;
    const Scene& scene = Application::getInstance().current_slice->scene;
    for (size_t extruder_nr = 0; extruder_nr < scene.extruders.size(); extruder_nr++)
    {
        filament_used.emplace_back(gcode.getTotalFilamentUsed(extruder_nr));
        material_ids.emplace_back(scene.extruders[extruder_nr].settings.get<std::string>("material_guid"));
        extruder_is_used.push_back(gcode.getExtruderIsUsed(extruder_nr));
    }
    std::string prefix = gcode.getFileHeader(extruder_is_used, &print_time, filament_used, material_ids);
    if (!Application::getInstance().communication->isSequential())
    {
        Application::getInstance().communication->sendGCodePrefix(prefix);
    }
    else
    {
        log("Gcode header after slicing:\n");
        log("%s", prefix.c_str());
        log("End of gcode header.\n");
    }
    if (mesh_group_settings.get<bool>("acceleration_enabled"))
    {
        gcode.writePrintAcceleration(mesh_group_settings.get<Acceleration>("machine_acceleration"));
        gcode.writeTravelAcceleration(mesh_group_settings.get<Acceleration>("machine_acceleration"));
    }
    if (mesh_group_settings.get<bool>("jerk_enabled"))
    {
        gcode.writeJerk(mesh_group_settings.get<Velocity>("machine_max_jerk_xy"));
    }
    if (gcode.getCurrentMaxZFeedrate() > 0)
    {
        gcode.writeMaxZFeedrate(mesh_group_settings.get<Velocity>("machine_max_feedrate_z"));
    }

    const std::string end_gcode = mesh_group_settings.get<std::string>("machine_end_gcode");

    if (end_gcode.length() > 0 && mesh_group_settings.get<bool>("relative_extrusion"))
    {
        gcode.writeExtrusionMode(false); // ensure absolute extrusion mode is set before the end gcode
    }
    gcode.finalize(end_gcode.c_str());

    // set extrusion mode back to "normal"
    const bool set_relative_extrusion_mode = (gcode.getFlavor() == EGCodeFlavor::REPRAP);
    gcode.writeExtrusionMode(set_relative_extrusion_mode);
    for (size_t e = 0; e < Application::getInstance().current_slice->scene.extruders.size(); e++)
    {
        gcode.writeTemperatureCommand(e, 0, false);
    }

    gcode.writeComment("End of Gcode");
    /*
    the profile string below can be executed since the M25 doesn't end the gcode on an UMO and when printing via USB.
    gcode.writeCode("M25 ;Stop reading from this point on.");
    gcode.writeComment("Cura profile string:");
    gcode.writeComment(FffProcessor::getInstance()->getAllLocalSettingsString() + FffProcessor::getInstance()->getProfileString());
    */
}


}//namespace cura

