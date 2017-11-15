//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <list>
#include <limits> // numeric_limits

#include "utils/math.h"
#include "FffGcodeWriter.h"
#include "FffProcessor.h"
#include "progress/Progress.h"
#include "wallOverlap.h"
#include "utils/orderOptimizer.h"
#include "GcodeLayerThreader.h"
#include "infill/SpaghettiInfillPathGenerator.h"
#include "InsetOrderOptimizer.h"

#define OMP_MAX_ACTIVE_LAYERS_PROCESSED 30 // TODO: hardcoded-value for the max number of layers being in the pipeline while writing away and destroying layers in a multi-threaded context

namespace cura
{

FffGcodeWriter::FffGcodeWriter(SettingsBase* settings_)
: SettingsMessenger(settings_)
, max_object_height(0)
, layer_plan_buffer(this, gcode)
{
    for (unsigned int extruder_nr = 0; extruder_nr < MAX_EXTRUDERS; extruder_nr++)
    { // initialize all as max layer_nr, so that they get updated to the lowest layer on which they are used.
        extruder_prime_layer_nr[extruder_nr] = std::numeric_limits<int>::max();
    }
}

void FffGcodeWriter::writeGCode(SliceDataStorage& storage, TimeKeeper& time_keeper)
{
    gcode.preSetup(storage.meshgroup);
    
    if (FffProcessor::getInstance()->getMeshgroupNr() == 0)
    { // first meshgroup
        gcode.resetTotalPrintTimeAndFilament();
        gcode.setInitialTemps(*storage.meshgroup, getStartExtruder(storage));
    }

    if (CommandSocket::isInstantiated())
    {
        CommandSocket::getInstance()->beginGCode();
    }

    setConfigFanSpeedLayerTime(storage);
    
    setConfigCoasting(storage);

    setConfigRetraction(storage);

    layer_plan_buffer.setPreheatConfig(*storage.meshgroup);
    
    if (FffProcessor::getInstance()->getMeshgroupNr() == 0)
    {
        unsigned int start_extruder_nr = getStartExtruder(storage);
        processStartingCode(storage, start_extruder_nr);
    }
    else
    {
        processNextMeshGroupCode(storage);
    }

    size_t total_layers = 0;
    for (SliceMeshStorage& mesh : storage.meshes)
    {
        total_layers = std::max(total_layers, mesh.layers.size());

        setInfillAndSkinAngles(mesh);
    }
    
    gcode.writeLayerCountComment(total_layers);

    { // calculate the mesh order for each extruder
        int extruder_count = storage.meshgroup->getExtruderCount();
        mesh_order_per_extruder.reserve(extruder_count);
        for (int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
        {
            mesh_order_per_extruder.push_back(calculateMeshOrder(storage, extruder_nr));
        }
    }
    calculateExtruderOrderPerLayer(storage);

    if (getSettingBoolean("magic_spiralize"))
    {
        findLayerSeamsForSpiralize(storage, total_layers);
    }

    int process_layer_starting_layer_nr = 0;
    bool has_raft = getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::RAFT;
    if (has_raft)
    {
        processRaft(storage);
        // process filler layers to fill the airgap with helper object (support etc) so that they stick better to the raft.
        // only process the filler layers if there is anything to print in them.
        for (bool extruder_is_used_in_filler_layers : storage.getExtrudersUsed(-1))
        {
            if (extruder_is_used_in_filler_layers)
            {
                process_layer_starting_layer_nr = -Raft::getFillerLayerCount(storage);
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
        if (mesh.getSettingAsZSeamType("z_seam_type") == EZSeamType::USER_SPECIFIED)
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
        const std::vector<unsigned int>& extruder_order = extruder_order_per_layer[layer_nr];
        for (unsigned int extruder_idx = 0; !done_this_layer && extruder_idx < extruder_order.size(); ++extruder_idx)
        {
            const unsigned int extruder_nr = extruder_order[extruder_idx];
            // iterate through this extruder's meshes until we find a part with insets
            const std::vector<unsigned int>& mesh_order = mesh_order_per_extruder[extruder_nr];
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

void FffGcodeWriter::setConfigFanSpeedLayerTime(SliceDataStorage& storage)
{
    for (int extr = 0; extr < storage.meshgroup->getExtruderCount(); extr++)
    {
        fan_speed_layer_time_settings_per_extruder.emplace_back();
        FanSpeedLayerTimeSettings& fan_speed_layer_time_settings = fan_speed_layer_time_settings_per_extruder.back();
        ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extr);
        fan_speed_layer_time_settings.cool_min_layer_time = train->getSettingInSeconds("cool_min_layer_time");
        fan_speed_layer_time_settings.cool_min_layer_time_fan_speed_max = train->getSettingInSeconds("cool_min_layer_time_fan_speed_max");
        fan_speed_layer_time_settings.cool_fan_speed_0 = train->getSettingInPercentage("cool_fan_speed_0");
        fan_speed_layer_time_settings.cool_fan_speed_min = train->getSettingInPercentage("cool_fan_speed_min");
        fan_speed_layer_time_settings.cool_fan_speed_max = train->getSettingInPercentage("cool_fan_speed_max");
        fan_speed_layer_time_settings.cool_min_speed = train->getSettingInMillimetersPerSecond("cool_min_speed");
        fan_speed_layer_time_settings.cool_fan_full_layer = train->getSettingAsLayerNumber("cool_fan_full_layer");
        if (!train->getSettingBoolean("cool_fan_enabled"))
        {
            fan_speed_layer_time_settings.cool_fan_speed_0 = 0;
            fan_speed_layer_time_settings.cool_fan_speed_min = 0;
            fan_speed_layer_time_settings.cool_fan_speed_max = 0;
        }
    }
}

void FffGcodeWriter::setConfigCoasting(SliceDataStorage& storage) 
{
    for (int extr = 0; extr < storage.meshgroup->getExtruderCount(); extr++)
    {
        storage.coasting_config.emplace_back();
        ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extr);
        CoastingConfig& coasting_config = storage.coasting_config.back();
        coasting_config.coasting_enable = train->getSettingBoolean("coasting_enable"); 
        coasting_config.coasting_volume = std::max(0.0, train->getSettingInCubicMillimeters("coasting_volume"));
        coasting_config.coasting_min_volume = std::max(0.0, train->getSettingInCubicMillimeters("coasting_min_volume"));
        coasting_config.coasting_speed = train->getSettingInPercentage("coasting_speed") / 100.0; 
    }
}

void FffGcodeWriter::setConfigRetraction(SliceDataStorage& storage) 
{
    int extruder_count = storage.meshgroup->getExtruderCount();
    for (int extruder = 0; extruder < extruder_count; extruder++)
    {
        ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder);
        RetractionConfig& retraction_config = storage.retraction_config_per_extruder[extruder];
        retraction_config.distance = (train->getSettingBoolean("retraction_enable"))? train->getSettingInMillimeters("retraction_amount") : 0;
        retraction_config.prime_volume = train->getSettingInCubicMillimeters("retraction_extra_prime_amount");
        retraction_config.speed = train->getSettingInMillimetersPerSecond("retraction_retract_speed");
        retraction_config.primeSpeed = train->getSettingInMillimetersPerSecond("retraction_prime_speed");
        retraction_config.zHop = train->getSettingInMicrons("retraction_hop");
        retraction_config.retraction_min_travel_distance = train->getSettingInMicrons("retraction_min_travel");
        retraction_config.retraction_extrusion_window = train->getSettingInMillimeters("retraction_extrusion_window");
        retraction_config.retraction_count_max = train->getSettingAsCount("retraction_count_max");

        RetractionConfig& switch_retraction_config = storage.extruder_switch_retraction_config_per_extruder[extruder];
        switch_retraction_config.distance = train->getSettingInMillimeters("switch_extruder_retraction_amount"); 
        switch_retraction_config.prime_volume = 0.0;
        switch_retraction_config.speed = train->getSettingInMillimetersPerSecond("switch_extruder_retraction_speed");
        switch_retraction_config.primeSpeed = train->getSettingInMillimetersPerSecond("switch_extruder_prime_speed");
        switch_retraction_config.zHop = retraction_config.zHop; // not used, because the last_retraction_config is used to govern how how high to zHop
        switch_retraction_config.retraction_min_travel_distance = 0; // no limitation on travel distance for an extruder switch retract
        switch_retraction_config.retraction_extrusion_window = 99999.9; // so that extruder switch retractions won't affect the retraction buffer (extruded_volume_at_previous_n_retractions)
        switch_retraction_config.retraction_count_max = 9999999; // extruder switch retraction is never limited
    }
}

unsigned int FffGcodeWriter::getStartExtruder(const SliceDataStorage& storage)
{
    int start_extruder_nr = getSettingAsIndex("adhesion_extruder_nr");
    if (getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::NONE)
    {
        std::vector<bool> extruder_is_used = storage.getExtrudersUsed();
        for (unsigned int extruder_nr = 0; extruder_nr < extruder_is_used.size(); extruder_nr++)
        {
            start_extruder_nr = extruder_nr;
            if (extruder_is_used[extruder_nr])
            {
                break;
            }
        }
    }
    assert(start_extruder_nr >= 0 && start_extruder_nr < storage.meshgroup->getExtruderCount() && "start_extruder_nr must be a valid extruder");
    return start_extruder_nr;
}

void FffGcodeWriter::setInfillAndSkinAngles(SliceMeshStorage& mesh)
{
    if (mesh.infill_angles.size() == 0)
    {
        mesh.infill_angles = mesh.getSettingAsIntegerList("infill_angles");
        if (mesh.infill_angles.size() == 0)
        {
            // user has not specified any infill angles so use defaults
            EFillMethod infill_pattern = mesh.getSettingAsFillMethod("infill_pattern");
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
        mesh.roofing_angles = mesh.getSettingAsIntegerList("roofing_angles");
        if (mesh.roofing_angles.size() == 0)
        {
            // user has not specified any infill angles so use defaults
            mesh.roofing_angles.push_back(45);
            mesh.roofing_angles.push_back(135);
        }
    }

    if (mesh.skin_angles.size() == 0)
    {
        mesh.skin_angles = mesh.getSettingAsIntegerList("skin_angles");
        if (mesh.skin_angles.size() == 0)
        {
            // user has not specified any infill angles so use defaults
            mesh.skin_angles.push_back(45);
            mesh.skin_angles.push_back(135);
        }
    }
}



void FffGcodeWriter::processStartingCode(const SliceDataStorage& storage, const unsigned int start_extruder_nr)
{
    if (!CommandSocket::isInstantiated())
    {
        std::vector<bool> extruder_is_used = storage.getExtrudersUsed();
        std::string prefix = gcode.getFileHeader(extruder_is_used);
        gcode.writeCode(prefix.c_str());
    }

    gcode.writeComment("Generated with Cura_SteamEngine " VERSION);

    if (gcode.getFlavor() == EGCodeFlavor::REPRAP || gcode.getFlavor() == EGCodeFlavor::GRIFFIN)
    {
        gcode.writeCode("T0"); // required before any temperature setting commands
    }

    if (gcode.getFlavor() != EGCodeFlavor::ULTIGCODE && gcode.getFlavor() != EGCodeFlavor::GRIFFIN)
    {
        if (getSettingBoolean("material_bed_temp_prepend")) 
        {
            if (getSettingBoolean("machine_heated_bed") && getSettingInDegreeCelsius("material_bed_temperature_layer_0") != 0)
            {
                gcode.writeBedTemperatureCommand(getSettingInDegreeCelsius("material_bed_temperature_layer_0"), getSettingBoolean("material_bed_temp_wait"));
            }
        }

        if (getSettingBoolean("material_print_temp_prepend")) 
        {
            for (int extruder_nr = 0; extruder_nr < storage.getSettingAsCount("machine_extruder_count"); extruder_nr++)
            {
                ExtruderTrain& train = *storage.meshgroup->getExtruderTrain(extruder_nr);
                double print_temp_0 = train.getSettingInDegreeCelsius("material_print_temperature_layer_0");
                double print_temp_here = (print_temp_0 != 0)? print_temp_0 : train.getSettingInDegreeCelsius("material_print_temperature");
                gcode.writeTemperatureCommand(extruder_nr, print_temp_here);
            }
            if (getSettingBoolean("material_print_temp_wait")) 
            {
                for (int extruder_nr = 0; extruder_nr < storage.getSettingAsCount("machine_extruder_count"); extruder_nr++)
                {
                    ExtruderTrain& train = *storage.meshgroup->getExtruderTrain(extruder_nr);
                    double print_temp_0 = train.getSettingInDegreeCelsius("material_print_temperature_layer_0");
                    double print_temp_here = (print_temp_0 != 0)? print_temp_0 : train.getSettingInDegreeCelsius("material_print_temperature");
                    gcode.writeTemperatureCommand(extruder_nr, print_temp_here, true);
                }
            }
        }
    }

    gcode.writeExtrusionMode(false); // ensure absolute extrusion mode is set before the start gcode
    gcode.writeCode(getSettingString("machine_start_gcode").c_str());

    if (gcode.getFlavor() == EGCodeFlavor::BFB)
    {
        gcode.writeComment("enable auto-retraction");
        std::ostringstream tmp;
        tmp << "M227 S" << (getSettingInMicrons("retraction_amount") * 2560 / 1000) << " P" << (getSettingInMicrons("retraction_amount") * 2560 / 1000);
        gcode.writeLine(tmp.str().c_str());
    }
    else if (gcode.getFlavor() == EGCodeFlavor::GRIFFIN)
    { // initialize extruder trains
        CommandSocket::setSendCurrentPosition(gcode.getPositionXY());
        gcode.startExtruder(start_extruder_nr);
        ExtruderTrain& train = *storage.meshgroup->getExtruderTrain(start_extruder_nr);
        constexpr bool wait = true;
        double print_temp_0 = train.getSettingInDegreeCelsius("material_print_temperature_layer_0");
        double print_temp_here = (print_temp_0 != 0)? print_temp_0 : train.getSettingInDegreeCelsius("material_print_temperature");
        gcode.writeTemperatureCommand(start_extruder_nr, print_temp_here, wait);
        gcode.writePrimeTrain(train.getSettingInMillimetersPerSecond("speed_travel"));
        extruder_prime_layer_nr[start_extruder_nr] = std::numeric_limits<int>::min(); // set to most negative number so that layer processing never primes this extruder any more.
        const RetractionConfig& retraction_config = storage.retraction_config_per_extruder[start_extruder_nr];
        gcode.writeRetraction(retraction_config);
    }
    if (getSettingBoolean("relative_extrusion"))
    {
        gcode.writeExtrusionMode(true);
    }
}

void FffGcodeWriter::processNextMeshGroupCode(const SliceDataStorage& storage)
{
    gcode.writeFanCommand(0);
    gcode.setZ(max_object_height + 5000);

    if (storage.getSettingBoolean("machine_heated_bed") && storage.getSettingInDegreeCelsius("material_bed_temperature_layer_0") != 0)
    {
        constexpr bool wait = true;
        gcode.writeBedTemperatureCommand(storage.getSettingInDegreeCelsius("material_bed_temperature_layer_0"), wait);
    }

    CommandSocket::setSendCurrentPosition(gcode.getPositionXY());
    gcode.writeTravel(gcode.getPositionXY(), storage.meshgroup->getExtruderTrain(gcode.getExtruderNr())->getSettingInMillimetersPerSecond("speed_travel"));
    Point start_pos(storage.model_min.x, storage.model_min.y);
    gcode.writeTravel(start_pos, storage.meshgroup->getExtruderTrain(gcode.getExtruderNr())->getSettingInMillimetersPerSecond("speed_travel"));
}
    
void FffGcodeWriter::processRaft(const SliceDataStorage& storage)
{
    int extruder_nr = getSettingAsIndex("adhesion_extruder_nr");
    ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder_nr);
    
    CombingMode combing_mode = storage.getSettingAsCombingMode("retraction_combing"); 

    int z = 0;
    


    const int initial_raft_layer_nr = -Raft::getTotalExtraLayers(storage);

    // some infill config for all lines infill generation below
    constexpr int offset_from_poly_outline = 0;
    constexpr double fill_overlap = 0; // raft line shouldn't be expanded - there is no boundary polygon printed
    constexpr int extra_infill_shift = 0;
    Polygons raft_polygons; // should remain empty, since we only have the lines pattern for the raft...

    unsigned int current_extruder_nr = extruder_nr;

    { // raft base layer
        int layer_nr = initial_raft_layer_nr;
        int layer_height = train->getSettingInMicrons("raft_base_thickness");
        z += layer_height;
        int64_t comb_offset = train->getSettingInMicrons("raft_base_line_spacing");

        std::vector<FanSpeedLayerTimeSettings> fan_speed_layer_time_settings_per_extruder_raft_base = fan_speed_layer_time_settings_per_extruder; // copy so that we change only the local copy
        for (FanSpeedLayerTimeSettings& fan_speed_layer_time_settings : fan_speed_layer_time_settings_per_extruder_raft_base)
        {
            double regular_fan_speed = train->getSettingInPercentage("raft_base_fan_speed");
            fan_speed_layer_time_settings.cool_fan_speed_min = regular_fan_speed;
            fan_speed_layer_time_settings.cool_fan_speed_0 = regular_fan_speed; // ignore initial layer fan speed stuff
        }

        LayerPlan& gcode_layer = *new LayerPlan(storage, layer_nr, z, layer_height, extruder_nr, fan_speed_layer_time_settings_per_extruder_raft_base, combing_mode, comb_offset, train->getSettingBoolean("travel_avoid_other_parts"), train->getSettingInMicrons("travel_avoid_distance"));
        gcode_layer.setIsInside(true);

        gcode_layer.setExtruder(extruder_nr);

        if (CommandSocket::isInstantiated())
        {
            CommandSocket::getInstance()->sendOptimizedLayerInfo(layer_nr, z, layer_height);
        }

        Polygons wall = storage.raftOutline.offset(-gcode_layer.configs_storage.raft_base_config.getLineWidth() / 2);
        wall.simplify(); //Simplify because of a micron-movement created in corners when insetting a polygon that was offset with round joint type.
        gcode_layer.addPolygonsByOptimizer(wall, gcode_layer.configs_storage.raft_base_config);

        Polygons raftLines;
        double fill_angle = 0;
        constexpr bool zig_zaggify_infill = false;
        Infill infill_comp(EFillMethod::LINES, zig_zaggify_infill, wall, offset_from_poly_outline, gcode_layer.configs_storage.raft_base_config.getLineWidth(), train->getSettingInMicrons("raft_base_line_spacing"), fill_overlap, fill_angle, z, extra_infill_shift);
        infill_comp.generate(raft_polygons, raftLines);
        gcode_layer.addLinesByOptimizer(raftLines, gcode_layer.configs_storage.raft_base_config, SpaceFillType::Lines);

        // When we use raft, we need to make sure that all used extruders for this print will get primed on the first raft layer,
        // and then switch back to the original extruder.
        std::vector<unsigned int> extruder_order = getUsedExtrudersOnLayerExcludingStartingExtruder(storage, extruder_nr, layer_nr);
        for (unsigned int to_be_primed_extruder_nr : extruder_order)
        {
            setExtruder_addPrime(storage, gcode_layer, to_be_primed_extruder_nr);
            current_extruder_nr = to_be_primed_extruder_nr;
        }

        layer_plan_buffer.handle(gcode_layer, gcode);
    }

    { // raft interface layer
        int layer_nr = initial_raft_layer_nr + 1;
        int layer_height = train->getSettingInMicrons("raft_interface_thickness");
        z += layer_height;
        int64_t comb_offset = train->getSettingInMicrons("raft_interface_line_spacing");

        std::vector<FanSpeedLayerTimeSettings> fan_speed_layer_time_settings_per_extruder_raft_interface = fan_speed_layer_time_settings_per_extruder; // copy so that we change only the local copy
        for (FanSpeedLayerTimeSettings& fan_speed_layer_time_settings : fan_speed_layer_time_settings_per_extruder_raft_interface)
        {
            double regular_fan_speed = train->getSettingInPercentage("raft_interface_fan_speed");
            fan_speed_layer_time_settings.cool_fan_speed_min = regular_fan_speed;
            fan_speed_layer_time_settings.cool_fan_speed_0 = regular_fan_speed; // ignore initial layer fan speed stuff
        }

        LayerPlan& gcode_layer = *new LayerPlan(storage, layer_nr, z, layer_height, current_extruder_nr, fan_speed_layer_time_settings_per_extruder_raft_interface, combing_mode, comb_offset, train->getSettingBoolean("travel_avoid_other_parts"), train->getSettingInMicrons("travel_avoid_distance"));
        gcode_layer.setIsInside(true);

        gcode_layer.setExtruder(extruder_nr); // reset to extruder number, because we might have primed in the last layer
        current_extruder_nr = extruder_nr;

        if (CommandSocket::isInstantiated())
        {
            CommandSocket::getInstance()->sendOptimizedLayerInfo(layer_nr, z, layer_height);
        }

        Polygons raft_outline_path = storage.raftOutline.offset(-gcode_layer.configs_storage.raft_interface_config.getLineWidth() / 2); //Do this manually because of micron-movement created in corners when insetting a polygon that was offset with round joint type.
        raft_outline_path.simplify(); //Remove those micron-movements.
        constexpr coord_t infill_outline_width = 0;
        Polygons raftLines;
        int offset_from_poly_outline = 0;
        double fill_angle = train->getSettingAsCount("raft_surface_layers") > 0 ? 45 : 90;
        constexpr bool zig_zaggify_infill = true;
        Infill infill_comp(EFillMethod::ZIG_ZAG, zig_zaggify_infill, raft_outline_path, offset_from_poly_outline, infill_outline_width, train->getSettingInMicrons("raft_interface_line_spacing"), fill_overlap, fill_angle, z, extra_infill_shift);
        infill_comp.generate(raft_polygons, raftLines);
        gcode_layer.addLinesByOptimizer(raftLines, gcode_layer.configs_storage.raft_interface_config, SpaceFillType::Lines);

        layer_plan_buffer.handle(gcode_layer, gcode);
    }
    
    int layer_height = train->getSettingInMicrons("raft_surface_thickness");

    for (int raftSurfaceLayer = 1; raftSurfaceLayer <= train->getSettingAsCount("raft_surface_layers"); raftSurfaceLayer++)
    { // raft surface layers
        const int layer_nr = initial_raft_layer_nr + 2 + raftSurfaceLayer - 1; // 2: 1 base layer, 1 interface layer
        z += layer_height;
        const int64_t comb_offset = train->getSettingInMicrons("raft_surface_line_spacing");

        std::vector<FanSpeedLayerTimeSettings> fan_speed_layer_time_settings_per_extruder_raft_surface = fan_speed_layer_time_settings_per_extruder; // copy so that we change only the local copy
        for (FanSpeedLayerTimeSettings& fan_speed_layer_time_settings : fan_speed_layer_time_settings_per_extruder_raft_surface)
        {
            double regular_fan_speed = train->getSettingInPercentage("raft_surface_fan_speed");
            fan_speed_layer_time_settings.cool_fan_speed_min = regular_fan_speed;
            fan_speed_layer_time_settings.cool_fan_speed_0 = regular_fan_speed; // ignore initial layer fan speed stuff
        }

        LayerPlan& gcode_layer = *new LayerPlan(storage, layer_nr, z, layer_height, extruder_nr, fan_speed_layer_time_settings_per_extruder_raft_surface, combing_mode, comb_offset, train->getSettingBoolean("travel_avoid_other_parts"), train->getSettingInMicrons("travel_avoid_distance"));
        gcode_layer.setIsInside(true);

        // make sure that we are using the correct extruder to print raft
        gcode_layer.setExtruder(extruder_nr);
        current_extruder_nr = extruder_nr;

        if (CommandSocket::isInstantiated())
        {
            CommandSocket::getInstance()->sendOptimizedLayerInfo(layer_nr, z, layer_height);
        }

        Polygons raft_outline_path = storage.raftOutline.offset(-gcode_layer.configs_storage.raft_surface_config.getLineWidth() / 2); //Do this manually because of micron-movement created in corners when insetting a polygon that was offset with round joint type.
        raft_outline_path.simplify(); //Remove those micron-movements.
        constexpr coord_t infill_outline_width = 0;
        Polygons raft_lines;
        int offset_from_poly_outline = 0;
        double fill_angle = 90 * raftSurfaceLayer;
        constexpr bool zig_zaggify_infill = true;
        Infill infill_comp(EFillMethod::ZIG_ZAG, zig_zaggify_infill, raft_outline_path, offset_from_poly_outline, infill_outline_width, train->getSettingInMicrons("raft_surface_line_spacing"), fill_overlap, fill_angle, z, extra_infill_shift);
        infill_comp.generate(raft_polygons, raft_lines);
        gcode_layer.addLinesByOptimizer(raft_lines, gcode_layer.configs_storage.raft_surface_config, SpaceFillType::Lines);

        layer_plan_buffer.handle(gcode_layer, gcode);
    }
}

LayerPlan& FffGcodeWriter::processLayer(const SliceDataStorage& storage, int layer_nr, unsigned int total_layers) const
{
    logDebug("GcodeWriter processing layer %i of %i\n", layer_nr, total_layers);

    int layer_thickness = getSettingInMicrons("layer_height");
    int64_t z;
    bool include_helper_parts = true;
    if (layer_nr < 0)
    {
#ifdef DEBUG
        assert(getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::RAFT && "negative layer_number means post-raft, pre-model layer!");
#endif // DEBUG
        const int filler_layer_count = Raft::getFillerLayerCount(storage);
        layer_thickness = Raft::getFillerLayerHeight(storage);
        z = Raft::getTotalThickness(storage) + (filler_layer_count + layer_nr + 1) * layer_thickness;
    }
    else
    {
        z = storage.meshes[0].layers[layer_nr].printZ; // stub default
        // find printZ of first actual printed mesh
        for (const SliceMeshStorage& mesh : storage.meshes)
        {
            if (mesh.getSettingBoolean("support_mesh")
                || mesh.getSettingBoolean("anti_overhang_mesh")
                || mesh.getSettingBoolean("cutting_mesh")
                || mesh.getSettingBoolean("infill_mesh"))
            {
                continue;
            }
            z = mesh.layers[layer_nr].printZ;
            break;
        }
        if (layer_nr == 0)
        {
            if (getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::RAFT)
            {
                include_helper_parts = false;
            }
            layer_thickness = getSettingInMicrons("layer_height_0");
        }
    }

    if (CommandSocket::isInstantiated())
    {
#pragma omp critical
        CommandSocket::getInstance()->sendOptimizedLayerInfo(layer_nr, z, layer_thickness);
    }

    bool avoid_other_parts = false;
    coord_t avoid_distance = 0; // minimal avoid distance is zero
    const std::vector<bool> extruder_is_used = storage.getExtrudersUsed();
    for (int extr_nr = 0; extr_nr < storage.meshgroup->getExtruderCount(); extr_nr++)
    {
        if (extruder_is_used[extr_nr])
        {
            ExtruderTrain* extr = storage.meshgroup->getExtruderTrain(extr_nr);

            if (extr->getSettingBoolean("travel_avoid_other_parts"))
            {
                avoid_other_parts = true;
                avoid_distance = std::max(avoid_distance, extr->getSettingInMicrons("travel_avoid_distance"));
            }
        }
    }

    coord_t max_inner_wall_width = 0;
    for (const SettingsBaseVirtual& mesh_settings : storage.meshes)
    {
        max_inner_wall_width = std::max(max_inner_wall_width, mesh_settings.getSettingInMicrons((mesh_settings.getSettingAsCount("wall_line_count") > 1) ? "wall_line_width_x" : "wall_line_width_0")); 
        if (layer_nr == 0)
        {
            const ExtruderTrain* train = storage.meshgroup->getExtruderTrain(mesh_settings.getSettingAsExtruderNr((mesh_settings.getSettingAsCount("wall_line_count") > 1) ? "wall_0_extruder_nr" : "wall_x_extruder_nr"));
            max_inner_wall_width *= train->getSettingAsRatio("initial_layer_line_width_factor");
        }
    }
    int64_t comb_offset_from_outlines = max_inner_wall_width * 2;

    assert(static_cast<int>(extruder_order_per_layer_negative_layers.size()) + layer_nr >= 0 && "Layer numbers shouldn't get more negative than there are raft/filler layers");
    const std::vector<unsigned int>& extruder_order =
        (layer_nr < 0)?
        extruder_order_per_layer_negative_layers[extruder_order_per_layer_negative_layers.size() + layer_nr]
        :
        extruder_order_per_layer[layer_nr];

    LayerPlan& gcode_layer = *new LayerPlan(storage, layer_nr, z, layer_thickness, extruder_order.front(), fan_speed_layer_time_settings_per_extruder, getSettingAsCombingMode("retraction_combing"), comb_offset_from_outlines, avoid_other_parts, avoid_distance);

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

    const unsigned int support_roof_extruder_nr = getSettingAsIndex("support_roof_extruder_nr");
    const unsigned int support_bottom_extruder_nr = getSettingAsIndex("support_bottom_extruder_nr");
    const unsigned int support_infill_extruder_nr = (layer_nr <= 0)? getSettingAsIndex("support_extruder_nr_layer_0") : getSettingAsIndex("support_infill_extruder_nr");

    for (unsigned int extruder_nr : extruder_order)
    {
        if (include_helper_parts
            && (extruder_nr == support_infill_extruder_nr || extruder_nr == support_roof_extruder_nr || extruder_nr == support_bottom_extruder_nr))
        {
            addSupportToGCode(storage, gcode_layer, extruder_nr);
        }

        if (layer_nr >= 0)
        {
            const std::vector<unsigned int>& mesh_order = mesh_order_per_extruder[extruder_nr];
            for (unsigned int mesh_idx : mesh_order)
            {
                const SliceMeshStorage& mesh = storage.meshes[mesh_idx];
                const PathConfigStorage::MeshPathConfigs& mesh_config = gcode_layer.configs_storage.mesh_configs[mesh_idx];
                if (mesh.getSettingAsSurfaceMode("magic_mesh_surface_mode") == ESurfaceMode::SURFACE)
                {
                    assert(static_cast<int>(extruder_nr) == mesh.getSettingAsExtruderNr("wall_0_extruder_nr") && "mesh surface mode should always only be printed with the outer wall extruder!");
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
        // print the prime tower if it hasn't been printed yet
        int prev_extruder = gcode_layer.getExtruder(); // most likely the same extruder as we are extruding with now
        addPrimeTower(storage, gcode_layer, prev_extruder);
    }

    return gcode_layer;
}

bool FffGcodeWriter::getExtruderNeedPrimeBlobDuringFirstLayer(const SliceDataStorage& storage, uint32_t extruder_nr) const
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
    ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder_nr);
    Point start_close_to;
    if (train->getSettingBoolean("prime_blob_enable"))
    {
        bool prime_pos_is_abs = train->getSettingBoolean("extruder_prime_pos_abs");
        Point prime_pos = Point(train->getSettingInMicrons("extruder_prime_pos_x"), train->getSettingInMicrons("extruder_prime_pos_y"));
        start_close_to = prime_pos_is_abs? prime_pos : gcode_layer.getLastPlannedPositionOrStartingPosition() + prime_pos;
    }
    else
    {
        start_close_to = gcode_layer.getLastPlannedPositionOrStartingPosition();
    }
    gcode_layer.addTravel(skirt_brim.back().closestPointTo(start_close_to));
    gcode_layer.addPolygonsByOptimizer(skirt_brim, gcode_layer.configs_storage.skirt_brim_config_per_extruder[extruder_nr]);
}

void FffGcodeWriter::processOozeShield(const SliceDataStorage& storage, LayerPlan& gcode_layer) const
{
    unsigned int layer_nr = std::max(0, gcode_layer.getLayerNr());
    if (layer_nr == 0 && storage.getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::BRIM)
    {
        return; // ooze shield already generated by brim
    }
    if (storage.oozeShield.size() > 0 && layer_nr < storage.oozeShield.size())
    {
        gcode_layer.addPolygonsByOptimizer(storage.oozeShield[layer_nr], gcode_layer.configs_storage.skirt_brim_config_per_extruder[0]); //TODO: Skirt and brim configuration index should correspond to draft shield extruder number.
    }
}

void FffGcodeWriter::processDraftShield(const SliceDataStorage& storage, LayerPlan& gcode_layer) const
{
    unsigned int layer_nr = std::max(0, gcode_layer.getLayerNr());
    if (storage.draft_protection_shield.size() == 0)
    {
        return;
    }
    if (!getSettingBoolean("draft_shield_enabled"))
    {
        return;
    }
    if (layer_nr == 0 && storage.getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::BRIM)
    {
        return; // ooze shield already generated by brim
    }

    if (getSettingAsDraftShieldHeightLimitation("draft_shield_height_limitation") == DraftShieldHeightLimitation::LIMITED)
    {
        const int draft_shield_height = getSettingInMicrons("draft_shield_height");
        const int layer_height_0 = getSettingInMicrons("layer_height_0");
        const int layer_height = getSettingInMicrons("layer_height");
        const unsigned int max_screen_layer = (unsigned int)((draft_shield_height - layer_height_0) / layer_height + 1);
        if (layer_nr > max_screen_layer)
        {
            return;
        }
    }

    gcode_layer.addPolygonsByOptimizer(storage.draft_protection_shield, gcode_layer.configs_storage.skirt_brim_config_per_extruder[0]); //TODO: Skirt and brim configuration index should correspond to draft shield extruder number.
}

void FffGcodeWriter::calculateExtruderOrderPerLayer(const SliceDataStorage& storage)
{
    unsigned int last_extruder;
    // set the initial extruder of this meshgroup
    if (FffProcessor::getInstance()->getMeshgroupNr() == 0)
    { // first meshgroup
        last_extruder = getStartExtruder(storage);
    }
    else
    {
        last_extruder = gcode.getExtruderNr();
    }
    for (int layer_nr = -Raft::getTotalExtraLayers(storage); layer_nr < static_cast<int>(storage.print_layer_count); layer_nr++)
    {
        std::vector<std::vector<unsigned int>>& extruder_order_per_layer_here = (layer_nr < 0)? extruder_order_per_layer_negative_layers : extruder_order_per_layer;
        extruder_order_per_layer_here.push_back(getUsedExtrudersOnLayerExcludingStartingExtruder(storage, last_extruder, layer_nr));
        last_extruder = extruder_order_per_layer_here.back().back();
        extruder_prime_layer_nr[last_extruder] = std::min(extruder_prime_layer_nr[last_extruder], layer_nr);
    }
}

std::vector<unsigned int> FffGcodeWriter::getUsedExtrudersOnLayerExcludingStartingExtruder(const SliceDataStorage& storage, const unsigned int start_extruder, const int layer_nr) const
{
    unsigned int extruder_count = storage.getSettingAsCount("machine_extruder_count");
    assert(static_cast<int>(extruder_count) > 0);
    std::vector<unsigned int> ret;
    ret.push_back(start_extruder);
    std::vector<bool> extruder_is_used_on_this_layer = storage.getExtrudersUsed(layer_nr);
    
    // check if we are on the first layer
    if ((getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::RAFT && layer_nr == -Raft::getTotalExtraLayers(storage))
        || (getSettingAsPlatformAdhesion("adhesion_type") != EPlatformAdhesion::RAFT && layer_nr == 0))
    {
        // check if we need prime blob on the first layer
        for (unsigned int used_idx = 0; used_idx < extruder_is_used_on_this_layer.size(); used_idx++)
        {
            if (this->getExtruderNeedPrimeBlobDuringFirstLayer(storage, used_idx))
            {
                extruder_is_used_on_this_layer[used_idx] = true;
            }
        }
    }

    for (unsigned int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
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

std::vector<unsigned int> FffGcodeWriter::calculateMeshOrder(const SliceDataStorage& storage, int extruder_nr) const
{
    OrderOptimizer<unsigned int> mesh_idx_order_optimizer;

    for (unsigned int mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
    {
        const SliceMeshStorage& mesh = storage.meshes[mesh_idx];
        if (mesh.getExtruderIsUsed(extruder_nr))
        {
            const Mesh& mesh_data = storage.meshgroup->meshes[mesh_idx];
            const Point3 middle = (mesh_data.getAABB().min + mesh_data.getAABB().max) / 2;
            mesh_idx_order_optimizer.addItem(Point(middle.x, middle.y), mesh_idx);
        }
    }
    std::list<unsigned int> mesh_indices_order = mesh_idx_order_optimizer.optimize();
    std::list<unsigned int>::iterator starting_mesh_idx_it = mesh_indices_order.end();
    { // calculate starting_mesh_it
        const ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder_nr);
        const Point layer_start_position = Point(train->getSettingInMicrons("layer_start_x"), train->getSettingInMicrons("layer_start_y"));
        coord_t best_dist2 = std::numeric_limits<coord_t>::max();
        for (std::list<unsigned int>::iterator mesh_it = mesh_indices_order.begin(); mesh_it != mesh_indices_order.end(); ++mesh_it)
        {
            const unsigned int mesh_idx = *mesh_it;
            const Mesh& mesh = storage.meshgroup->meshes[mesh_idx];
            const Point3 middle3 = mesh.getAABB().getMiddle();
            const Point middle(middle3.x, middle3.y);
            const coord_t dist2 = vSize2(middle - layer_start_position);
            if (dist2 < best_dist2)
            {
                best_dist2 = dist2;
                starting_mesh_idx_it = mesh_it;
            }
        }
    }
    std::vector<unsigned int> ret;
    ret.reserve(mesh_indices_order.size());
    for (unsigned int mesh_order_nr = 0; mesh_order_nr < mesh_indices_order.size(); mesh_order_nr++)
    {
        if (starting_mesh_idx_it == mesh_indices_order.end())
        {
            starting_mesh_idx_it = mesh_indices_order.begin();
        }
        unsigned int mesh_order_idx = *starting_mesh_idx_it;
        const unsigned int mesh_idx = mesh_idx_order_optimizer.items[mesh_order_idx].second;
        ret.push_back(mesh_idx);
        ++starting_mesh_idx_it;
    }
    return ret;
}

void FffGcodeWriter::addMeshLayerToGCode_meshSurfaceMode(const SliceDataStorage& storage, const SliceMeshStorage& mesh, const PathConfigStorage::MeshPathConfigs& mesh_config, LayerPlan& gcode_layer) const
{
    if (gcode_layer.getLayerNr() > mesh.layer_nr_max_filled_layer)
    {
        return;
    }

    if (mesh.getSettingBoolean("anti_overhang_mesh")
        || mesh.getSettingBoolean("support_mesh")
    )
    {
        return;
    }

    setExtruder_addPrime(storage, gcode_layer, mesh.getSettingAsExtruderNr("wall_0_extruder_nr"));

    const SliceLayer* layer = &mesh.layers[gcode_layer.getLayerNr()];


    Polygons polygons;
    for(unsigned int partNr=0; partNr<layer->parts.size(); partNr++)
    {
        polygons.add(layer->parts[partNr].outline);
    }

    ZSeamConfig z_seam_config(mesh.getSettingAsZSeamType("z_seam_type"), mesh.getZSeamHint(), mesh.getSettingAsZSeamCornerPrefType("z_seam_corner"));
    gcode_layer.addPolygonsByOptimizer(polygons, mesh_config.inset0_config, nullptr, z_seam_config, mesh.getSettingInMicrons("wall_0_wipe_dist"), getSettingBoolean("magic_spiralize"));

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

void FffGcodeWriter::addMeshLayerToGCode(const SliceDataStorage& storage, const SliceMeshStorage& mesh, const int extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, LayerPlan& gcode_layer) const
{
    if (gcode_layer.getLayerNr() > mesh.layer_nr_max_filled_layer)
    {
        return;
    }

    if (mesh.getSettingBoolean("anti_overhang_mesh")
        || mesh.getSettingBoolean("support_mesh")
    )
    {
        return;
    }

    const SliceLayer& layer = mesh.layers[gcode_layer.getLayerNr()];

    if (layer.parts.size() == 0)
    {
        return;
    }

    const ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder_nr);


    ZSeamConfig z_seam_config(mesh.getSettingAsZSeamType("z_seam_type"), mesh.getZSeamHint(), mesh.getSettingAsZSeamCornerPrefType("z_seam_corner"));
    Point layer_start_position = Point(train->getSettingInMicrons("layer_start_x"), train->getSettingInMicrons("layer_start_y"));
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
    if (mesh.getSettingAsSurfaceMode("magic_mesh_surface_mode") != ESurfaceMode::NORMAL && extruder_nr == mesh.getSettingAsExtruderNr("wall_0_extruder_nr"))
    {
        addMeshOpenPolyLinesToGCode(mesh, mesh_config, gcode_layer);
    }
}

void FffGcodeWriter::addMeshPartToGCode(const SliceDataStorage& storage, const SliceMeshStorage& mesh, const int extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part, LayerPlan& gcode_layer) const
{
    int infill_angle = 45; // original default, this will get updated to an element from mesh->infill_angles
    if (mesh.infill_angles.size() > 0)
    {
        unsigned int combined_infill_layers = std::max(1U, round_divide(mesh.getSettingInMicrons("infill_sparse_thickness"), std::max(getSettingInMicrons("layer_height"), (coord_t)1)));
        infill_angle = mesh.infill_angles.at((gcode_layer.getLayerNr() / combined_infill_layers) % mesh.infill_angles.size());
    }
    
    int infill_line_distance = mesh.getSettingInMicrons("infill_line_distance");
    int infill_overlap = mesh.getSettingInMicrons("infill_overlap_mm");

    bool added_something = false;

    if (mesh.getSettingBoolean("infill_before_walls"))
    {
        added_something = added_something | processInfill(storage, gcode_layer, mesh, extruder_nr, mesh_config, part, infill_line_distance, infill_overlap, infill_angle);
    }

    added_something = added_something | processInsets(storage, gcode_layer, mesh, extruder_nr, mesh_config, part, mesh.getZSeamHint());

    processOutlineGaps(storage, gcode_layer, mesh, extruder_nr, mesh_config, part, added_something);

    if (!mesh.getSettingBoolean("infill_before_walls"))
    {
        added_something = added_something | processInfill(storage, gcode_layer, mesh, extruder_nr, mesh_config, part, infill_line_distance, infill_overlap, infill_angle);
    }

    added_something = added_something | processSkinAndPerimeterGaps(storage, gcode_layer, mesh, extruder_nr, mesh_config, part);

    //After a layer part, make sure the nozzle is inside the comb boundary, so we do not retract on the perimeter.
    if (added_something && (!getSettingBoolean("magic_spiralize") || static_cast<int>(gcode_layer.getLayerNr()) < mesh.getSettingAsCount("bottom_layers")))
    {
        int innermost_wall_line_width = mesh.getSettingInMicrons((mesh.getSettingAsCount("wall_line_count") > 1) ? "wall_line_width_x" : "wall_line_width_0");
        if (gcode_layer.getLayerNr() == 0)
        {
            innermost_wall_line_width *= mesh.getSettingAsRatio("initial_layer_line_width_factor");
        }
        gcode_layer.moveInsideCombBoundary(innermost_wall_line_width);
    }

    gcode_layer.setIsInside(false);
}

bool FffGcodeWriter::processInfill(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const int extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part, int infill_line_distance, int infill_overlap, int infill_angle) const
{
    if (extruder_nr != mesh.getSettingAsExtruderNr("infill_extruder_nr"))
    {
        return false;
    }
    const Point3 mesh_middle = mesh.bounding_box.getMiddle();
    const Point infill_origin(mesh_middle.x + mesh.getSettingInMicrons("infill_offset_x"), mesh_middle.y + mesh.getSettingInMicrons("infill_offset_y"));
    if (mesh.getSettingBoolean("spaghetti_infill_enabled"))
    {
        return SpaghettiInfillPathGenerator::processSpaghettiInfill(storage, *this, gcode_layer, mesh, extruder_nr, mesh_config, part, infill_line_distance, infill_overlap, infill_angle, infill_origin);
    }
    else
    {
        bool added_something = processMultiLayerInfill(storage, gcode_layer, mesh, extruder_nr, mesh_config, part, infill_line_distance, infill_overlap, infill_angle, infill_origin);
        added_something = added_something | processSingleLayerInfill(storage, gcode_layer, mesh, extruder_nr, mesh_config, part, infill_line_distance, infill_overlap, infill_angle, infill_origin);
        return added_something;
    }
}

bool FffGcodeWriter::processMultiLayerInfill(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const int extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part, int infill_line_distance, int infill_overlap, int infill_angle, const Point& infill_origin) const
{
    if (extruder_nr != mesh.getSettingAsExtruderNr("infill_extruder_nr"))
    {
        return false;
    }
    bool added_something = false;
    if (infill_line_distance > 0)
    {
        //Print the thicker infill lines first. (double or more layer thickness, infill combined with previous layers)
        for(unsigned int combine_idx = 1; combine_idx < part.infill_area_per_combine_per_density[0].size(); combine_idx++)
        {
            const unsigned int infill_line_width = mesh_config.infill_config[combine_idx].getLineWidth();
            EFillMethod infill_pattern = mesh.getSettingAsFillMethod("infill_pattern");
            const bool zig_zaggify_infill = mesh.getSettingBoolean("zig_zaggify_infill");
            Polygons infill_polygons;
            Polygons infill_lines;
            for (unsigned int density_idx = part.infill_area_per_combine_per_density.size() - 1; (int)density_idx >= 0; density_idx--)
            { // combine different density infill areas (for gradual infill)
                unsigned int density_factor = 2 << density_idx; // == pow(2, density_idx + 1)
                int infill_line_distance_here = infill_line_distance * density_factor; // the highest density infill combines with the next to create a grid with density_factor 1
                int infill_shift = infill_line_distance_here / 2;
                if (density_idx == part.infill_area_per_combine_per_density.size() - 1 || infill_pattern == EFillMethod::CROSS || infill_pattern == EFillMethod::CROSS_3D)
                {
                    infill_line_distance_here /= 2;
                }
                
                Infill infill_comp(infill_pattern, zig_zaggify_infill, part.infill_area_per_combine_per_density[density_idx][combine_idx], 0
                    , infill_line_width, infill_line_distance_here, infill_overlap, infill_angle, gcode_layer.z, infill_shift, infill_origin
                    , /*Polygons* perimeter_gaps =*/ nullptr
                    , /*bool connected_zigzags =*/ false
                    , /*bool use_endpieces =*/ false
                    , /*bool skip_some_zags =*/ false
                    , /*int zag_skip_count =*/ 0
                    , mesh.getSettingBoolean("cross_infill_apply_pockets_alternatingly"), mesh.getSettingInMicrons("cross_infill_pocket_size"));
                const SpaceFillingTreeFill* cross_fill_pattern = (density_idx < mesh.cross_fill_patterns.size()) ? mesh.cross_fill_patterns[density_idx] : nullptr;
                infill_comp.generate(infill_polygons, infill_lines, cross_fill_pattern, &mesh);
            }
            if (infill_lines.size() > 0 || infill_polygons.size() > 0)
            {
                added_something = true;
                setExtruder_addPrime(storage, gcode_layer, extruder_nr);
                gcode_layer.setIsInside(true); // going to print stuff inside print object
                gcode_layer.addPolygonsByOptimizer(infill_polygons, mesh_config.infill_config[combine_idx]);
                gcode_layer.addLinesByOptimizer(infill_lines, mesh_config.infill_config[combine_idx], (infill_pattern == EFillMethod::ZIG_ZAG)? SpaceFillType::PolyLines : SpaceFillType::Lines);
            }
        }
    }
    return added_something;
}

bool FffGcodeWriter::processSingleLayerInfill(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const int extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part, int infill_line_distance, int infill_overlap, int infill_angle, const Point& infill_origin) const
{
    if (extruder_nr != mesh.getSettingAsExtruderNr("infill_extruder_nr"))
    {
        return false;
    }
    if (infill_line_distance == 0 || part.infill_area_per_combine_per_density[0].size() == 0)
    {
        return false;
    }
    bool added_something = false;
    const unsigned int infill_line_width = mesh_config.infill_config[0].getLineWidth();
        
    //Combine the 1 layer thick infill with the top/bottom skin and print that as one thing.
    Polygons infill_polygons;
    Polygons infill_lines;

    EFillMethod pattern = mesh.getSettingAsFillMethod("infill_pattern");
    const bool zig_zaggify_infill = mesh.getSettingBoolean("zig_zaggify_infill");
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
        Infill infill_comp(pattern, zig_zaggify_infill, part.infill_area_per_combine_per_density[density_idx][0], 0
            , infill_line_width, infill_line_distance_here, infill_overlap, infill_angle, gcode_layer.z, infill_shift, infill_origin
            , /*Polygons* perimeter_gaps =*/ nullptr
            , /*bool connected_zigzags =*/ false
            , /*bool use_endpieces =*/ false
            , /*bool skip_some_zags =*/ false
            , /*int zag_skip_count =*/ 0
            , mesh.getSettingBoolean("cross_infill_apply_pockets_alternatingly"), mesh.getSettingInMicrons("cross_infill_pocket_size"));
        const SpaceFillingTreeFill* cross_fill_pattern = (density_idx < mesh.cross_fill_patterns.size()) ? mesh.cross_fill_patterns[density_idx] : nullptr;
        infill_comp.generate(infill_polygons, infill_lines, cross_fill_pattern, &mesh);
    }
    if (infill_lines.size() > 0 || infill_polygons.size() > 0)
    {
        added_something = true;
        setExtruder_addPrime(storage, gcode_layer, extruder_nr);
        gcode_layer.setIsInside(true); // going to print stuff inside print object
        gcode_layer.addPolygonsByOptimizer(infill_polygons, mesh_config.infill_config[0]);
        if (pattern == EFillMethod::GRID || pattern == EFillMethod::LINES || pattern == EFillMethod::TRIANGLES || pattern == EFillMethod::CUBIC || pattern == EFillMethod::TETRAHEDRAL || pattern == EFillMethod::QUARTER_CUBIC || pattern == EFillMethod::CUBICSUBDIV)
        {
            gcode_layer.addLinesByOptimizer(infill_lines, mesh_config.infill_config[0], SpaceFillType::Lines, mesh.getSettingInMicrons("infill_wipe_dist"));
        }
        else
        {
            gcode_layer.addLinesByOptimizer(infill_lines, mesh_config.infill_config[0], (pattern == EFillMethod::ZIG_ZAG)? SpaceFillType::PolyLines : SpaceFillType::Lines);
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

bool FffGcodeWriter::processInsets(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const int extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part, Point z_seam_pos) const
{
    if (extruder_nr != mesh.getSettingAsExtruderNr("wall_0_extruder_nr") && extruder_nr != mesh.getSettingAsExtruderNr("wall_x_extruder_nr"))
    {
        return false;
    }
    bool added_something = false;
    bool compensate_overlap_0 = mesh.getSettingBoolean("travel_compensate_overlapping_walls_0_enabled");
    bool compensate_overlap_x = mesh.getSettingBoolean("travel_compensate_overlapping_walls_x_enabled");
    bool retract_before_outer_wall = mesh.getSettingBoolean("travel_retract_before_outer_wall");
    if (mesh.getSettingAsCount("wall_line_count") > 0)
    {
        bool spiralize = false;
        if (getSettingBoolean("magic_spiralize"))
        {
            if (part.insets.size() == 0)
            {
                // nothing to do
                return false;
            }
            const int bottom_layers = mesh.getSettingAsCount("bottom_layers");
            if (gcode_layer.getLayerNr() >= bottom_layers)
            {
                spiralize = true;
            }
            if (spiralize && gcode_layer.getLayerNr() == bottom_layers && !part.insets.empty() && extruder_nr == mesh.getSettingAsExtruderNr("wall_0_extruder_nr"))
            { // on the last normal layer first make the outer wall normally and then start a second outer wall from the same hight, but gradually moving upward
                added_something = true;
                setExtruder_addPrime(storage, gcode_layer, extruder_nr);
                gcode_layer.setIsInside(true); // going to print stuff inside print object
                WallOverlapComputation* wall_overlap_computation(nullptr);
                int wall_0_wipe_dist(0);
                gcode_layer.addPolygonsByOptimizer(part.insets[0], mesh_config.inset0_config, wall_overlap_computation, ZSeamConfig(), wall_0_wipe_dist);
            }
        }
        // Only spiralize the first part in the mesh, any other parts will be printed using the normal, non-spiralize codepath.
        // This sounds weird but actually does the right thing when you have a model that has multiple parts at the bottom that merge into
        // one part higher up. Once all the parts have merged, layers above that level will be spiralized
        if (spiralize && &mesh.layers[gcode_layer.getLayerNr()].parts[0] == &part)
        {
            if (part.insets.size() > 0 && extruder_nr == mesh.getSettingAsExtruderNr("wall_0_extruder_nr"))
            {
                added_something = true;
                setExtruder_addPrime(storage, gcode_layer, extruder_nr);
                gcode_layer.setIsInside(true); // going to print stuff inside print object
                processSpiralizedWall(storage, gcode_layer, mesh_config, part);
            }
        }
        else if (InsetOrderOptimizer::optimizingInsetsIsWorthwhile(mesh, part))
        {
            InsetOrderOptimizer ioo(*this, storage, gcode_layer, mesh, extruder_nr, mesh_config, part, gcode_layer.getLayerNr(), z_seam_pos);
            return ioo.processInsetsWithOptimizedOrdering();
        }
        else
        {
            const bool outer_inset_first = mesh.getSettingBoolean("outer_inset_first")
                || (gcode_layer.getLayerNr() == 0 && mesh.getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::BRIM);
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
                    constexpr bool spiralize = false;
                    constexpr float flow = 1.0;
                    if (part.insets[0].size() > 0 && extruder_nr == mesh.getSettingAsExtruderNr("wall_0_extruder_nr"))
                    {
                        added_something = true;
                        setExtruder_addPrime(storage, gcode_layer, extruder_nr);
                        gcode_layer.setIsInside(true); // going to print stuff inside print object
                        ZSeamConfig z_seam_config(mesh.getSettingAsZSeamType("z_seam_type"), z_seam_pos, mesh.getSettingAsZSeamCornerPrefType("z_seam_corner"));
                        Polygons outer_wall = part.insets[0];
                        if (!compensate_overlap_0)
                        {
                            WallOverlapComputation* wall_overlap_computation(nullptr);
                            gcode_layer.addPolygonsByOptimizer(outer_wall, mesh_config.inset0_config, wall_overlap_computation, z_seam_config, mesh.getSettingInMicrons("wall_0_wipe_dist"), spiralize, flow, retract_before_outer_wall);
                        }
                        else
                        {
                            WallOverlapComputation wall_overlap_computation(outer_wall, mesh_config.inset0_config.getLineWidth());
                            gcode_layer.addPolygonsByOptimizer(outer_wall, mesh_config.inset0_config, &wall_overlap_computation, z_seam_config, mesh.getSettingInMicrons("wall_0_wipe_dist"), spiralize, flow, retract_before_outer_wall);
                        }
                    }
                }
                // Inner walls are processed
                else
                {
                    if (part.insets[processed_inset_number].size() > 0 && extruder_nr == mesh.getSettingAsExtruderNr("wall_x_extruder_nr"))
                    {
                        added_something = true;
                        setExtruder_addPrime(storage, gcode_layer, extruder_nr);
                        gcode_layer.setIsInside(true); // going to print stuff inside print object
                        ZSeamConfig z_seam_config(mesh.getSettingAsZSeamType("z_seam_type"), z_seam_pos, mesh.getSettingAsZSeamCornerPrefType("z_seam_corner"));
                        Polygons inner_wall = part.insets[processed_inset_number];
                        if (!compensate_overlap_x)
                        {
                            WallOverlapComputation* wall_overlap_computation(nullptr);
                            gcode_layer.addPolygonsByOptimizer(part.insets[processed_inset_number], mesh_config.insetX_config, wall_overlap_computation, z_seam_config);
                        }
                        else
                        {
                            WallOverlapComputation wall_overlap_computation(inner_wall, mesh_config.insetX_config.getLineWidth());
                            gcode_layer.addPolygonsByOptimizer(inner_wall, mesh_config.insetX_config, &wall_overlap_computation, z_seam_config);
                        }
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

void FffGcodeWriter::processOutlineGaps(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const int extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part, bool& added_something) const
{
    int wall_0_extruder_nr = mesh.getSettingAsExtruderNr("wall_0_extruder_nr");
    if (extruder_nr != wall_0_extruder_nr || !mesh.getSettingBoolean("fill_outline_gaps"))
    {
        return;
    }
    const unsigned int perimeter_gaps_line_width = mesh_config.perimeter_gap_config.getLineWidth();
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
    constexpr bool zig_zaggify_infill = false;
    Infill infill_comp(EFillMethod::LINES, zig_zaggify_infill, part.outline_gaps, offset, perimeter_gaps_line_width, perimeter_gaps_line_width, outline_gap_overlap, skin_angle, gcode_layer.z, extra_infill_shift);
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

bool FffGcodeWriter::processSkinAndPerimeterGaps(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const int extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part) const
{
    int top_bottom_extruder_nr = mesh.getSettingAsExtruderNr("top_bottom_extruder_nr");
    int roofing_extruder_nr = mesh.getSettingAsExtruderNr("roofing_extruder_nr");
    int wall_0_extruder_nr = mesh.getSettingAsExtruderNr("wall_0_extruder_nr");
    if (extruder_nr != top_bottom_extruder_nr && extruder_nr != wall_0_extruder_nr
        && (extruder_nr != roofing_extruder_nr || mesh.getSettingAsCount("roofing_layer_count") <= 0))
    {
        return false;
    }
    bool added_something = false;

    bool fill_perimeter_gaps = mesh.getSettingAsFillPerimeterGapMode("fill_perimeter_gaps") != FillPerimeterGapMode::NOWHERE
                            && !getSettingBoolean("magic_spiralize")
                            && extruder_nr == wall_0_extruder_nr;

    Point z_seam_pos(0, 0); // not used
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

bool FffGcodeWriter::processSkinPart(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const int extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SkinPart& skin_part) const
{
    bool added_something = false;

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
        if (extruder_nr == mesh.getSettingAsExtruderNr("wall_0_extruder_nr"))
        {
            perimeter_gaps.add(skin_part.perimeter_gaps);
        }
        perimeter_gaps.unionPolygons();

        processPerimeterGaps(storage, gcode_layer, mesh, extruder_nr, perimeter_gaps, mesh_config.perimeter_gap_config, added_something);
    }
    return added_something;
}

void FffGcodeWriter::processSkinInsets(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const int extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SkinPart& skin_part, bool& added_something) const
{
    const int wall_0_extruder_nr = mesh.getSettingAsExtruderNr("wall_0_extruder_nr");
    // add skin walls aka skin perimeters
    if (extruder_nr == wall_0_extruder_nr)
    {
        for (const Polygons& skin_perimeter : skin_part.insets)
        {
            if (skin_perimeter.size() > 0)
            {
                added_something = true;
                setExtruder_addPrime(storage, gcode_layer, extruder_nr);
                gcode_layer.setIsInside(true); // going to print stuff inside print object
                gcode_layer.addPolygonsByOptimizer(skin_perimeter, mesh_config.insetX_config); // add polygons to gcode in inward order
            }
        }
    }
}

void FffGcodeWriter::processRoofing(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const int extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SkinPart& skin_part, Polygons& concentric_perimeter_gaps, bool& added_something) const
{
    const int roofing_extruder_nr = mesh.getSettingAsExtruderNr("roofing_extruder_nr");
    if (extruder_nr != roofing_extruder_nr)
    {
        return;
    }

    const bool generate_perimeter_gaps =
        mesh.getSettingAsFillPerimeterGapMode("fill_perimeter_gaps") != FillPerimeterGapMode::NOWHERE
        && !getSettingBoolean("magic_spiralize");

    EFillMethod pattern = mesh.getSettingAsFillMethod("roofing_pattern");

    int roofing_angle = 45;
    if (mesh.roofing_angles.size() > 0)
    {
        roofing_angle = mesh.roofing_angles.at(gcode_layer.getLayerNr() % mesh.roofing_angles.size());
    }

    const coord_t skin_overlap = 0; // skinfill already expanded over the roofing areas; don't overlap with perimeters
    Polygons* perimeter_gaps_output = (generate_perimeter_gaps)? &concentric_perimeter_gaps : nullptr;
    processSkinPrintFeature(storage, gcode_layer, mesh, extruder_nr, skin_part.roofing_fill, mesh_config.roofing_config, pattern, roofing_angle, skin_overlap, perimeter_gaps_output, added_something);
}

void FffGcodeWriter::processTopBottom(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const int extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SkinPart& skin_part, Polygons& concentric_perimeter_gaps, bool& added_something) const
{
    const int top_bottom_extruder_nr = mesh.getSettingAsExtruderNr("top_bottom_extruder_nr");
    if (extruder_nr != top_bottom_extruder_nr)
    {
        return;
    }

    const bool generate_perimeter_gaps =
        mesh.getSettingAsFillPerimeterGapMode("fill_perimeter_gaps") != FillPerimeterGapMode::NOWHERE
        && !getSettingBoolean("magic_spiralize");

    EFillMethod pattern = (gcode_layer.getLayerNr() == 0)?
        mesh.getSettingAsFillMethod("top_bottom_pattern_0") :
        mesh.getSettingAsFillMethod("top_bottom_pattern");

    int skin_angle = 45;
    bool skin_alternate_rotation = mesh.getSettingBoolean("skin_alternate_rotation") && (mesh.getSettingAsCount("top_layers") >= 4 || mesh.getSettingAsCount("bottom_layers") >= 4 );
    if (mesh.skin_angles.size() > 0)
    {
        skin_angle = mesh.skin_angles.at(gcode_layer.getLayerNr() % mesh.skin_angles.size());
    }
    if (skin_alternate_rotation && ( gcode_layer.getLayerNr() / 2 ) & 1)
    {
        skin_angle -= 45;
    }

    // generate skin_polygons and skin_lines (and concentric_perimeter_gaps if needed)
    int bridge = -1;
    {
        // calculate bridging angle
        if (gcode_layer.getLayerNr() > 0)
        {
            bridge = bridgeAngle(skin_part.outline, &mesh.layers[gcode_layer.getLayerNr() - 1]);
        }
        if (bridge > -1)
        {
            pattern = EFillMethod::LINES; // force lines pattern when bridging
            skin_angle = bridge;
        }
    }

    // calculate polygons and lines
    const coord_t skin_overlap = mesh.getSettingInMicrons("skin_overlap_mm");
    Polygons* perimeter_gaps_output = (generate_perimeter_gaps)? &concentric_perimeter_gaps : nullptr;
    processSkinPrintFeature(storage, gcode_layer, mesh, extruder_nr, skin_part.inner_infill, mesh_config.skin_config, pattern, skin_angle, skin_overlap, perimeter_gaps_output, added_something);
}

void FffGcodeWriter::processSkinPrintFeature(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const int extruder_nr, const Polygons& area, const GCodePathConfig& config, EFillMethod pattern, int skin_angle, const coord_t skin_overlap, Polygons* perimeter_gaps_output, bool& added_something) const
{
    Polygons skin_polygons;
    Polygons skin_lines;

    constexpr int extra_infill_shift = 0;
    constexpr coord_t offset_from_inner_skin_infill = 0;
    const bool zig_zaggify_infill = pattern == EFillMethod::ZIG_ZAG;
    const Point infill_origin;
    Infill infill_comp(pattern, zig_zaggify_infill, area, offset_from_inner_skin_infill, config.getLineWidth(), config.getLineWidth(), skin_overlap, skin_angle, gcode_layer.z, extra_infill_shift, infill_origin, perimeter_gaps_output);
    infill_comp.generate(skin_polygons, skin_lines);

    // add paths
    if (skin_polygons.size() > 0 || skin_lines.size() > 0)
    {
        added_something = true;
        setExtruder_addPrime(storage, gcode_layer, extruder_nr);
        gcode_layer.setIsInside(true); // going to print stuff inside print object
        gcode_layer.addPolygonsByOptimizer(skin_polygons, config);

        std::optional<Point> near_start_location;
        const EFillMethod pattern = (gcode_layer.getLayerNr() == 0)?
            mesh.getSettingAsFillMethod("top_bottom_pattern_0") :
            mesh.getSettingAsFillMethod("top_bottom_pattern");
        if (pattern == EFillMethod::LINES || pattern == EFillMethod::ZIG_ZAG)
        { // update near_start_location to a location which tries to avoid seams in skin
            near_start_location = getSeamAvoidingLocation(area, skin_angle, gcode_layer.getLastPlannedPositionOrStartingPosition());
        }

        constexpr float flow = 1.0;
        if (pattern == EFillMethod::GRID || pattern == EFillMethod::LINES || pattern == EFillMethod::TRIANGLES || pattern == EFillMethod::CUBIC || pattern == EFillMethod::TETRAHEDRAL || pattern == EFillMethod::QUARTER_CUBIC || pattern == EFillMethod::CUBICSUBDIV)
        {
            gcode_layer.addLinesByOptimizer(skin_lines, config, SpaceFillType::Lines, mesh.getSettingInMicrons("infill_wipe_dist"), flow, near_start_location);
        }
        else
        {
            SpaceFillType space_fill_type = (pattern == EFillMethod::ZIG_ZAG)? SpaceFillType::PolyLines : SpaceFillType::Lines;
            constexpr coord_t wipe_dist = 0;
            gcode_layer.addLinesByOptimizer(skin_lines, config, space_fill_type, wipe_dist, flow, near_start_location);
        }
    }
}

void FffGcodeWriter::processPerimeterGaps(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const int extruder_nr, const Polygons& perimeter_gaps, const GCodePathConfig& perimeter_gap_config, bool& added_something) const
{
    const coord_t perimeter_gaps_line_width = perimeter_gap_config.getLineWidth();
    
    assert(mesh.roofing_angles.size() > 0);
    const bool zig_zaggify_infill = false;
    int perimeter_gaps_angle = mesh.roofing_angles[gcode_layer.getLayerNr() % mesh.roofing_angles.size()]; // use roofing angles for perimeter gaps
    Polygons gap_polygons; // will remain empty
    Polygons gap_lines;
    constexpr int offset = 0;
    constexpr int extra_infill_shift = 0;
    const coord_t skin_overlap = mesh.getSettingInMicrons("skin_overlap_mm");
    Infill infill_comp(EFillMethod::LINES, zig_zaggify_infill, perimeter_gaps, offset, perimeter_gaps_line_width, perimeter_gaps_line_width, skin_overlap, perimeter_gaps_angle, gcode_layer.z, extra_infill_shift);
    infill_comp.generate(gap_polygons, gap_lines);
    if (gap_lines.size() > 0)
    {
        added_something = true;
        setExtruder_addPrime(storage, gcode_layer, extruder_nr);
        gcode_layer.setIsInside(true); // going to print stuff inside print object
        gcode_layer.addLinesByOptimizer(gap_lines, perimeter_gap_config, SpaceFillType::Lines);
    }
}

bool FffGcodeWriter::processIroning(const SliceMeshStorage& mesh, const SliceLayer& layer, const GCodePathConfig& line_config, LayerPlan& gcode_layer) const
{
    bool added_something = false;
    const bool ironing_enabled = mesh.getSettingBoolean("ironing_enabled");
    const bool ironing_only_highest_layer = mesh.getSettingBoolean("ironing_only_highest_layer");
    if (ironing_enabled && (!ironing_only_highest_layer || mesh.layer_nr_max_filled_layer == gcode_layer.getLayerNr()) && layer.top_surface)
    {
        added_something |= layer.top_surface->ironing(mesh, line_config, gcode_layer);
    }
    return added_something;
}


bool FffGcodeWriter::addSupportToGCode(const SliceDataStorage& storage, LayerPlan& gcode_layer, int extruder_nr) const
{
    bool support_added = false;
    if (!storage.support.generated || gcode_layer.getLayerNr() > storage.support.layer_nr_max_filled_layer)
    {
        return support_added;
    }

    const int support_roof_extruder_nr = getSettingAsIndex("support_roof_extruder_nr");
    const int support_bottom_extruder_nr = getSettingAsIndex("support_bottom_extruder_nr");
    int support_infill_extruder_nr = (gcode_layer.getLayerNr() <= 0)? getSettingAsIndex("support_extruder_nr_layer_0") : getSettingAsIndex("support_infill_extruder_nr");

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

    const int extruder_nr = (gcode_layer.getLayerNr() <= 0) ? getSettingAsIndex("support_extruder_nr_layer_0") : getSettingAsIndex("support_infill_extruder_nr");
    const ExtruderTrain& infill_extruder = *storage.meshgroup->getExtruderTrain(extruder_nr);

    const coord_t default_support_line_distance = infill_extruder.getSettingInMicrons("support_line_distance");
    const int default_support_infill_overlap = infill_extruder.getSettingInMicrons("infill_overlap_mm");
    const double support_infill_angle = 0;
    coord_t default_support_line_width = infill_extruder.getSettingInMicrons("support_line_width");
    if (gcode_layer.getLayerNr() == 0 && storage.getSettingAsPlatformAdhesion("adhesion_type") != EPlatformAdhesion::RAFT)
    {
        default_support_line_width *= infill_extruder.getSettingAsRatio("initial_layer_line_width_factor");
    }

    EFillMethod support_pattern = infill_extruder.getSettingAsFillMethod("support_pattern");
    if (gcode_layer.getLayerNr() <= 0 && (support_pattern == EFillMethod::LINES || support_pattern == EFillMethod::ZIG_ZAG))
    {
        support_pattern = EFillMethod::GRID;
    }
    const bool zig_zaggify_infill = support_pattern == EFillMethod::ZIG_ZAG || support_pattern == EFillMethod::CROSS || support_pattern == EFillMethod::CROSS_3D;
    const bool skip_some_zags = infill_extruder.getSettingBoolean("support_skip_some_zags");
    const int zag_skip_count = infill_extruder.getSettingAsCount("support_zag_skip_count");

    // create a list of outlines and use PathOrderOptimizer to optimize the travel move
    PathOrderOptimizer island_order_optimizer(gcode_layer.getLastPlannedPositionOrStartingPosition());
    for (unsigned int part_idx = 0; part_idx < support_layer.support_infill_parts.size(); ++part_idx)
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
            assert(!part.insets.empty() && "No empty support infill parts may exist.");
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

                const int offset_from_outline = 0;

                bool use_endpieces = true;
                Polygons* perimeter_gaps = nullptr;
                const Point infill_origin;
                Infill infill_comp(support_pattern, zig_zaggify_infill, support_area, offset_from_outline, support_line_width,
                                   support_line_distance_here, current_support_infill_overlap, support_infill_angle, gcode_layer.z, support_shift, infill_origin,
                                   perimeter_gaps, infill_extruder.getSettingBoolean("support_connect_zigzags"), use_endpieces,
                                   skip_some_zags, zag_skip_count);
                const SpaceFillingTreeFill* cross_fill_pattern = (density_idx < storage.support.cross_fill_patterns.size()) ? storage.support.cross_fill_patterns[density_idx] : nullptr;
                infill_comp.generate(support_polygons, support_lines, cross_fill_pattern);
            }

            if (support_lines.size() > 0 || support_polygons.size() > 0)
            {
                setExtruder_addPrime(storage, gcode_layer, extruder_nr); // only switch extruder if we're sure we're going to switch
                gcode_layer.setIsInside(false); // going to print stuff outside print object, i.e. support
                gcode_layer.addPolygonsByOptimizer(support_polygons, gcode_layer.configs_storage.support_infill_config[combine_idx]);
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

    const int roof_extruder_nr = getSettingAsIndex("support_roof_extruder_nr");
    const ExtruderTrain& roof_extr = *storage.meshgroup->getExtruderTrain(roof_extruder_nr);

    const EFillMethod pattern = roof_extr.getSettingAsFillMethod("support_roof_pattern");
    const double fill_angle = supportInterfaceFillAngle(storage, pattern, "support_roof_height", gcode_layer.getLayerNr());
    const bool zig_zaggify_infill = pattern == EFillMethod::ZIG_ZAG;
    constexpr int support_roof_overlap = 0; // the roofs should never be expanded outwards
    constexpr int outline_offset =  0;
    constexpr int extra_infill_shift = 0;
    const Point infill_origin;
    constexpr Polygons* perimeter_gaps = nullptr;
    constexpr bool use_endpieces = true;
    constexpr bool connected_zigzags = false;

    coord_t support_roof_line_distance = roof_extr.getSettingInMicrons("support_roof_line_distance");
    const coord_t support_roof_line_width = roof_extr.getSettingInMicrons("support_roof_line_width");
    if (gcode_layer.getLayerNr() == 0 && support_roof_line_distance < 2 * support_roof_line_width)
    { // if roof is dense
        support_roof_line_distance *= roof_extr.getSettingAsRatio("initial_layer_line_width_factor");
    }

    Polygons infill_outline = support_layer.support_roof;
    Polygons wall;
    // make sure there is a wall if this is on the first layer
    if (gcode_layer.getLayerNr() == 0)
    {
        wall = support_layer.support_roof.offset(-support_roof_line_width / 2);
        infill_outline = wall.offset(-support_roof_line_width / 2);
    }

    Infill roof_computation(pattern, zig_zaggify_infill, infill_outline, outline_offset, gcode_layer.configs_storage.support_roof_config.getLineWidth(), support_roof_line_distance, support_roof_overlap, fill_angle, gcode_layer.z, extra_infill_shift, infill_origin, perimeter_gaps, connected_zigzags, use_endpieces);
    Polygons roof_polygons;
    Polygons roof_lines;
    roof_computation.generate(roof_polygons, roof_lines);
    if ((gcode_layer.getLayerNr() == 0 && wall.empty()) || (roof_polygons.empty() && roof_lines.empty()))
    {
        return false; //We didn't create any support roof.
    }
    setExtruder_addPrime(storage, gcode_layer, roof_extruder_nr);
    gcode_layer.setIsInside(false); // going to print stuff outside print object, i.e. support
    if (gcode_layer.getLayerNr() == 0)
    {
        gcode_layer.addPolygonsByOptimizer(wall, gcode_layer.configs_storage.support_roof_config);
    }
    gcode_layer.addPolygonsByOptimizer(roof_polygons, gcode_layer.configs_storage.support_roof_config);
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

    const int bottom_extruder_nr = getSettingAsIndex("support_bottom_extruder_nr");
    const ExtruderTrain& bottom_extr = *storage.meshgroup->getExtruderTrain(bottom_extruder_nr);

    const EFillMethod pattern = bottom_extr.getSettingAsFillMethod("support_bottom_pattern");
    const double fill_angle = supportInterfaceFillAngle(storage, pattern, "support_bottom_height", gcode_layer.getLayerNr());
    const bool zig_zaggify_infill = pattern == EFillMethod::ZIG_ZAG;
    constexpr int support_bottom_overlap = 0; // the bottoms should never be expanded outwards
    constexpr int outline_offset =  0;
    constexpr int extra_infill_shift = 0;
    const Point infill_origin;
    constexpr Polygons* perimeter_gaps = nullptr;
    constexpr bool use_endpieces = true;
    constexpr bool connected_zigzags = false;

    const coord_t support_bottom_line_distance = bottom_extr.getSettingInMicrons("support_bottom_line_distance"); // note: no need to apply initial line width factor; support bottoms cannot exist on the first layer
    Infill bottom_computation(pattern, zig_zaggify_infill, support_layer.support_bottom, outline_offset, gcode_layer.configs_storage.support_bottom_config.getLineWidth(), support_bottom_line_distance, support_bottom_overlap, fill_angle, gcode_layer.z, extra_infill_shift, infill_origin, perimeter_gaps, connected_zigzags, use_endpieces);
    Polygons bottom_polygons;
    Polygons bottom_lines;
    bottom_computation.generate(bottom_polygons, bottom_lines);
    if (bottom_polygons.empty() && bottom_lines.empty())
    {
        return false;
    }
    setExtruder_addPrime(storage, gcode_layer, bottom_extruder_nr);
    gcode_layer.setIsInside(false); // going to print stuff outside print object, i.e. support
    gcode_layer.addPolygonsByOptimizer(bottom_polygons, gcode_layer.configs_storage.support_bottom_config);
    gcode_layer.addLinesByOptimizer(bottom_lines, gcode_layer.configs_storage.support_bottom_config, (pattern == EFillMethod::ZIG_ZAG) ? SpaceFillType::PolyLines : SpaceFillType::Lines);
    return true;
}

double FffGcodeWriter::supportInterfaceFillAngle(const SliceDataStorage& storage, const EFillMethod pattern, const std::string interface_height_setting, const int layer_nr) const
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
        if (mesh.getSettingInMicrons(interface_height_setting) >= 2 * getSettingInMicrons("layer_height"))
        {
            //Some roofs are quite thick.
            //Alternate between the two kinds of diagonal: / and \ .
            // + 2) % 2 is to handle negative layer numbers.
            return 45 + (((layer_nr % 2) + 2) % 2) * 90;
        }
    }

    return 90; //Perpendicular to support lines.
}

void FffGcodeWriter::setExtruder_addPrime(const SliceDataStorage& storage, LayerPlan& gcode_layer, int extruder_nr) const
{
    if (extruder_nr == -1) // an object with extruder_nr==-1 means it will be printed with any current nozzle
        return;
    
    int previous_extruder = gcode_layer.getExtruder();
    if (previous_extruder == extruder_nr) { return; }
    bool extruder_changed = gcode_layer.setExtruder(extruder_nr);
    
    if (extruder_changed)
    {
        if (extruder_prime_layer_nr[extruder_nr] == gcode_layer.getLayerNr())
        {
            ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder_nr);

            if (train->getSettingBoolean("prime_blob_enable"))
            { // only move to prime position if we do a blob/poop
                // ideally the prime position would be respected whether we do a blob or not,
                // but the frontend currently doesn't support a value function of an extruder setting depending on an fdmprinter setting,
                // which is needed to automatically ignore the prime position for the UM3 machine when blob is disabled
                bool prime_pos_is_abs = train->getSettingBoolean("extruder_prime_pos_abs");
                Point prime_pos = Point(train->getSettingInMicrons("extruder_prime_pos_x"), train->getSettingInMicrons("extruder_prime_pos_y"));
                gcode_layer.addTravel(prime_pos_is_abs? prime_pos : gcode_layer.getLastPlannedPositionOrStartingPosition() + prime_pos);

                gcode_layer.planPrime();
            }
        }

        if (gcode_layer.getLayerNr() == 0 && !gcode_layer.getSkirtBrimIsPlanned(extruder_nr))
        {
            processSkirtBrim(storage, gcode_layer, extruder_nr);
        }
        if (gcode_layer.getLayerNr() >= -Raft::getFillerLayerCount(storage))
        {
            addPrimeTower(storage, gcode_layer, previous_extruder);
        }
    }
}

void FffGcodeWriter::addPrimeTower(const SliceDataStorage& storage, LayerPlan& gcode_layer, int prev_extruder) const
{
    if (!getSettingBoolean("prime_tower_enable"))
    {
        return;
    }

    storage.primeTower.addToGcode(storage, gcode_layer, gcode, prev_extruder, gcode_layer.getExtruder());
}

void FffGcodeWriter::finalize()
{
    double print_time = gcode.getSumTotalPrintTimes();
    std::vector<double> filament_used;
    std::vector<std::string> material_ids;
    std::vector<bool> extruder_is_used;
    for (int extr_nr = 0; extr_nr < getSettingAsCount("machine_extruder_count"); extr_nr++)
    {
        filament_used.emplace_back(gcode.getTotalFilamentUsed(extr_nr));
        material_ids.emplace_back(gcode.getMaterialGUID(extr_nr));
        extruder_is_used.push_back(gcode.getExtruderIsUsed(extr_nr));
    }
    std::string prefix = gcode.getFileHeader(extruder_is_used, &print_time, filament_used, material_ids);
    if (CommandSocket::isInstantiated())
    {
        CommandSocket::getInstance()->sendGCodePrefix(prefix);
    }
    else
    {
        log("Gcode header after slicing:\n");
        log(prefix.c_str());
        log("End of gcode header.\n");
    }
    if (getSettingBoolean("acceleration_enabled"))
    {
        gcode.writePrintAcceleration(getSettingInMillimetersPerSecond("machine_acceleration"));
        gcode.writeTravelAcceleration(getSettingInMillimetersPerSecond("machine_acceleration"));
    }
    if (getSettingBoolean("jerk_enabled"))
    {
        gcode.writeJerk(getSettingInMillimetersPerSecond("machine_max_jerk_xy"));
    }
    if (gcode.getCurrentMaxZFeedrate() > 0)
    {
        gcode.writeMaxZFeedrate(getSettingInMillimetersPerSecond("machine_max_feedrate_z"));
    }

    std::string end_gcode = getSettingString("machine_end_gcode");

    if (end_gcode.length() > 0 && getSettingBoolean("relative_extrusion"))
    {
        gcode.writeExtrusionMode(false); // ensure absolute extrusion mode is set before the end gcode
    }
    gcode.finalize(end_gcode.c_str());

    // set extrusion mode back to "normal"
    const bool set_relative_extrusion_mode = (gcode.getFlavor() == EGCodeFlavor::REPRAP);
    gcode.writeExtrusionMode(set_relative_extrusion_mode);
    for (int e = 0; e < getSettingAsCount("machine_extruder_count"); e++)
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

