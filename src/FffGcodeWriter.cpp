
#include <list>

#include "utils/math.h"
#include "FffGcodeWriter.h"
#include "FffProcessor.h"
#include "progress/Progress.h"
#include "wallOverlap.h"
#include "utils/orderOptimizer.h"

namespace cura
{


void FffGcodeWriter::writeGCode(SliceDataStorage& storage, TimeKeeper& time_keeper)
{
    gcode.preSetup(storage.meshgroup);
    
    if (FffProcessor::getInstance()->getMeshgroupNr() == 0)
    { // first meshgroup
        gcode.resetTotalPrintTimeAndFilament();
        gcode.setInitialTemps(*storage.meshgroup, getStartExtruder(storage));
    }

    // set the initial extruder of this meshgroup
    if (FffProcessor::getInstance()->getMeshgroupNr() == 0)
    { // first meshgroup
        current_extruder_planned = getStartExtruder(storage);
    }
    else
    {
        current_extruder_planned = gcode.getExtruderNr();
    }

    if (CommandSocket::isInstantiated())
    {
        CommandSocket::getInstance()->beginGCode();
    }

    setConfigFanSpeedLayerTime(storage);
    
    setConfigCoasting(storage);

    setConfigRetraction(storage);
    
    initConfigs(storage);
    
    for (int extruder = 0; extruder < storage.meshgroup->getExtruderCount(); extruder++)
    { //Skirt and brim.
        storage.skirt_brim_config[extruder].setLayerHeight(getSettingInMicrons("layer_height_0"));
        skirt_brim_is_processed[extruder] = false;
    }

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
    }
    
    gcode.writeLayerCountComment(total_layers);

    bool has_raft = getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::RAFT;
    if (has_raft)
    {
        processRaft(storage, total_layers);
        // process filler layers to fill the airgap with helper object (support etc) so that they stick better to the raft.
        for (int layer_nr = -Raft::getFillerLayerCount(storage); layer_nr < 0; layer_nr++)
        {
            processLayer(storage, layer_nr, total_layers);
        }
    }

    { // calculate the mesh order for each extruder
        int extruder_count = storage.meshgroup->getExtruderCount();
        mesh_order_per_extruder.reserve(extruder_count);
        for (int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
        {
            mesh_order_per_extruder.push_back(calculateMeshOrder(storage, extruder_nr));
        }
    }

    for(unsigned int layer_nr=0; layer_nr<total_layers; layer_nr++)
    {
        processLayer(storage, layer_nr, total_layers);
    }
    
    Progress::messageProgressStage(Progress::Stage::FINISH, &time_keeper);

    //Store the object height for when we are printing multiple objects, as we need to clear every one of them when moving to the next position.
    max_object_height = std::max(max_object_height, storage.model_max.z);

    layer_plan_buffer.flush();

    constexpr bool force = true;
    gcode.writeRetraction(storage.retraction_config_per_extruder[gcode.getExtruderNr()], force); // retract after finishing each meshgroup
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
        coasting_config.coasting_volume = train->getSettingInCubicMillimeters("coasting_volume"); 
        coasting_config.coasting_min_volume = train->getSettingInCubicMillimeters("coasting_min_volume"); 
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

void FffGcodeWriter::initConfigs(SliceDataStorage& storage)
{
    for (int extruder = 0; extruder < storage.meshgroup->getExtruderCount(); extruder++)
    { //Skirt and brim.
        SettingsBase* train = storage.meshgroup->getExtruderTrain(extruder);
        storage.skirt_brim_config[extruder].init(train->getSettingInMillimetersPerSecond("skirt_brim_speed"), train->getSettingInMillimetersPerSecond("acceleration_skirt_brim"), train->getSettingInMillimetersPerSecond("jerk_skirt_brim"), train->getSettingInMicrons("skirt_brim_line_width"), train->getSettingInPercentage("material_flow"));
        storage.travel_config_per_extruder[extruder].init(train->getSettingInMillimetersPerSecond("speed_travel"), train->getSettingInMillimetersPerSecond("acceleration_travel"), train->getSettingInMillimetersPerSecond("jerk_travel"), 0, 0);
    }

    { // support 
        SettingsBase* infill_train = storage.meshgroup->getExtruderTrain(getSettingAsIndex("support_infill_extruder_nr"));
        storage.support_config.init(infill_train->getSettingInMillimetersPerSecond("speed_support_infill"), infill_train->getSettingInMillimetersPerSecond("acceleration_support_infill"), infill_train->getSettingInMillimetersPerSecond("jerk_support_infill"), infill_train->getSettingInMicrons("support_line_width"), infill_train->getSettingInPercentage("material_flow"));
    
        SettingsBase* interface_train = storage.meshgroup->getExtruderTrain(getSettingAsIndex("support_interface_extruder_nr"));
        storage.support_skin_config.init(interface_train->getSettingInMillimetersPerSecond("speed_support_interface"), interface_train->getSettingInMillimetersPerSecond("acceleration_support_interface"), interface_train->getSettingInMillimetersPerSecond("jerk_support_interface"), interface_train->getSettingInMicrons("support_interface_line_width"), interface_train->getSettingInPercentage("material_flow"));
    }
    
    for (SliceMeshStorage& mesh : storage.meshes)
    {
        mesh.inset0_config.init(mesh.getSettingInMillimetersPerSecond("speed_wall_0"), mesh.getSettingInMillimetersPerSecond("acceleration_wall_0"), mesh.getSettingInMillimetersPerSecond("jerk_wall_0"), mesh.getSettingInMicrons("wall_line_width_0"), mesh.getSettingInPercentage("material_flow"));
        mesh.insetX_config.init(mesh.getSettingInMillimetersPerSecond("speed_wall_x"), mesh.getSettingInMillimetersPerSecond("acceleration_wall_x"), mesh.getSettingInMillimetersPerSecond("jerk_wall_x"), mesh.getSettingInMicrons("wall_line_width_x"), mesh.getSettingInPercentage("material_flow"));
        mesh.skin_config.init(mesh.getSettingInMillimetersPerSecond("speed_topbottom"), mesh.getSettingInMillimetersPerSecond("acceleration_topbottom"), mesh.getSettingInMillimetersPerSecond("jerk_topbottom"), mesh.getSettingInMicrons("skin_line_width"), mesh.getSettingInPercentage("material_flow"));

        // The perimeter gap config follows the skin config, but has a different line width:
        // wall_line_width_x divided by two because the gaps are between 0 and 1 times the wall line width
        const int perimeter_gaps_line_width = mesh.getSettingInMicrons("wall_line_width_x") / 2;
        double perimeter_gaps_speed = mesh.getSettingInMillimetersPerSecond("speed_topbottom");
        if (mesh.getSettingBoolean("speed_equalize_flow_enabled"))
        {
            perimeter_gaps_speed = perimeter_gaps_speed * mesh.getSettingInMicrons("skin_line_width") / perimeter_gaps_line_width;
        }
        mesh.perimeter_gap_config.init(perimeter_gaps_speed, mesh.getSettingInMillimetersPerSecond("acceleration_topbottom"), mesh.getSettingInMillimetersPerSecond("jerk_topbottom"), perimeter_gaps_line_width, mesh.getSettingInPercentage("material_flow"));

        for (unsigned int idx = 0; idx < MAX_INFILL_COMBINE; idx++)
        {
            mesh.infill_config[idx].init(mesh.getSettingInMillimetersPerSecond("speed_infill"), mesh.getSettingInMillimetersPerSecond("acceleration_infill"), mesh.getSettingInMillimetersPerSecond("jerk_infill"), mesh.getSettingInMicrons("infill_line_width") * (idx + 1), mesh.getSettingInPercentage("material_flow"));
        }
    }
    
    storage.primeTower.initConfigs(storage.meshgroup);
}

unsigned int FffGcodeWriter::getStartExtruder(SliceDataStorage& storage)
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

void FffGcodeWriter::processStartingCode(SliceDataStorage& storage, const unsigned int start_extruder_nr)
{
    if (!CommandSocket::isInstantiated())
    {
        std::string prefix = gcode.getFileHeader();
        gcode.writeCode(prefix.c_str());
    }

    gcode.writeComment("Generated with Cura_SteamEngine " VERSION);

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
        gcode.writeCode("T0"); // Toolhead already assumed to be at T0, but writing it just to be safe...
        CommandSocket::setSendCurrentPosition(gcode.getPositionXY());
        gcode.startExtruder(start_extruder_nr);
        ExtruderTrain& train = *storage.meshgroup->getExtruderTrain(start_extruder_nr);
        constexpr bool wait = true;
        double print_temp_0 = train.getSettingInDegreeCelsius("material_print_temperature_layer_0");
        double print_temp_here = (print_temp_0 != 0)? print_temp_0 : train.getSettingInDegreeCelsius("material_print_temperature");
        gcode.writeTemperatureCommand(start_extruder_nr, print_temp_here, wait);
        gcode.writePrimeTrain(train.getSettingInMillimetersPerSecond("speed_travel"));
        extruder_prime_is_planned[start_extruder_nr] = true;
        RetractionConfig& retraction_config = storage.retraction_config_per_extruder[start_extruder_nr];
        gcode.writeRetraction(retraction_config);
    }
}

void FffGcodeWriter::processNextMeshGroupCode(SliceDataStorage& storage)
{
    gcode.writeFanCommand(0);

    bool wait = true;
    if (storage.getSettingBoolean("machine_heated_bed") && storage.getSettingInDegreeCelsius("material_bed_temperature_layer_0") != 0)
    {
        gcode.writeBedTemperatureCommand(storage.getSettingInDegreeCelsius("material_bed_temperature_layer_0"), wait);
    }

    gcode.resetExtrusionValue();
    CommandSocket::setSendCurrentPosition(gcode.getPositionXY());

    gcode.setZ(max_object_height + 5000);
    gcode.writeMove(gcode.getPositionXY(), storage.meshgroup->getExtruderTrain(gcode.getExtruderNr())->getSettingInMillimetersPerSecond("speed_travel"), 0);
    last_position_planned = Point(storage.model_min.x, storage.model_min.y);
    gcode.writeMove(last_position_planned, storage.meshgroup->getExtruderTrain(gcode.getExtruderNr())->getSettingInMillimetersPerSecond("speed_travel"), 0);
}
    
void FffGcodeWriter::processRaft(SliceDataStorage& storage, unsigned int total_layers)
{
    int extruder_nr = getSettingAsIndex("adhesion_extruder_nr");
    ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder_nr);
    
    CombingMode combing_mode = storage.getSettingAsCombingMode("retraction_combing"); 

    int z = 0;
    
    { // set configs 
        storage.raft_base_config.init(train->getSettingInMillimetersPerSecond("raft_base_speed"), train->getSettingInMillimetersPerSecond("raft_base_acceleration"), train->getSettingInMillimetersPerSecond("raft_base_jerk"), train->getSettingInMicrons("raft_base_line_width"), train->getSettingInPercentage("material_flow"));
        storage.raft_base_config.setLayerHeight(train->getSettingInMicrons("raft_base_thickness"));
        
        storage.raft_interface_config.init(train->getSettingInMillimetersPerSecond("raft_interface_speed"), train->getSettingInMillimetersPerSecond("raft_interface_acceleration"), train->getSettingInMillimetersPerSecond("raft_interface_jerk"), train->getSettingInMicrons("raft_interface_line_width"), train->getSettingInPercentage("material_flow"));
        storage.raft_interface_config.setLayerHeight(train->getSettingInMicrons("raft_interface_thickness"));

        storage.raft_surface_config.init(train->getSettingInMillimetersPerSecond("raft_surface_speed"), train->getSettingInMillimetersPerSecond("raft_surface_acceleration"), train->getSettingInMillimetersPerSecond("raft_surface_jerk"), train->getSettingInMicrons("raft_surface_line_width"), train->getSettingInPercentage("material_flow"));
        storage.raft_surface_config.setLayerHeight(train->getSettingInMicrons("raft_surface_thickness"));
    }

    const int initial_raft_layer_nr = -Raft::getTotalExtraLayers(storage);

    // some infill config for all lines infill generation below
    int offset_from_poly_outline = 0;
    double fill_overlap = 0; // raft line shouldn't be expanded - there is no boundary polygon printed
    int extra_infill_shift = 0;
    Polygons raft_polygons; // should remain empty, since we only have the lines pattern for the raft...
    
    { // raft base layer
        
        int layer_nr = initial_raft_layer_nr;
        int layer_height = train->getSettingInMicrons("raft_base_thickness");
        z += layer_height;
        int64_t comb_offset = train->getSettingInMicrons("raft_base_line_spacing");
        GCodePlanner& gcode_layer = layer_plan_buffer.emplace_back(storage, layer_nr, z, layer_height, last_position_planned, current_extruder_planned, is_inside_mesh_layer_part, fan_speed_layer_time_settings_per_extruder, combing_mode, comb_offset, train->getSettingBoolean("travel_avoid_other_parts"), train->getSettingInMicrons("travel_avoid_distance"));
        gcode_layer.setIsInside(true);

        gcode_layer.setExtruder(extruder_nr);

        if (CommandSocket::isInstantiated())
        {
            CommandSocket::getInstance()->sendOptimizedLayerInfo(layer_nr, z, layer_height);
        }

        Polygons wall = storage.raftOutline.offset(-storage.raft_base_config.getLineWidth() / 2);
        gcode_layer.addPolygonsByOptimizer(wall, &storage.raft_base_config);

        Polygons raftLines;
        double fill_angle = 0;
        Infill infill_comp(EFillMethod::LINES, wall, offset_from_poly_outline, storage.raft_base_config.getLineWidth(), train->getSettingInMicrons("raft_base_line_spacing"), fill_overlap, fill_angle, z, extra_infill_shift);
        infill_comp.generate(raft_polygons, raftLines);
        gcode_layer.addLinesByOptimizer(raftLines, &storage.raft_base_config, SpaceFillType::Lines);

        if (getExtrudersNeedPrimeDuringFirstLayer())
        {
            ensureAllExtrudersArePrimed(storage, gcode_layer, layer_nr);
        }

        last_position_planned = gcode_layer.getLastPosition();
        current_extruder_planned = gcode_layer.getExtruder();
        is_inside_mesh_layer_part = gcode_layer.getIsInsideMesh();

        gcode_layer.processFanSpeedAndMinimalLayerTime();
        gcode_layer.overrideFanSpeeds(train->getSettingInPercentage("raft_base_fan_speed"));
    }

    { // raft interface layer
        int layer_nr = initial_raft_layer_nr + 1;
        int layer_height = train->getSettingInMicrons("raft_interface_thickness");
        z += layer_height;
        int64_t comb_offset = train->getSettingInMicrons("raft_interface_line_spacing");
        GCodePlanner& gcode_layer = layer_plan_buffer.emplace_back(storage, layer_nr, z, layer_height, last_position_planned, current_extruder_planned, is_inside_mesh_layer_part, fan_speed_layer_time_settings_per_extruder, combing_mode, comb_offset, train->getSettingBoolean("travel_avoid_other_parts"), train->getSettingInMicrons("travel_avoid_distance"));
        gcode_layer.setIsInside(true);

        gcode_layer.setExtruder(extruder_nr); // reset to extruder number, because we might have primed in the last layer

        if (CommandSocket::isInstantiated())
        {
            CommandSocket::getInstance()->sendOptimizedLayerInfo(layer_nr, z, layer_height);
        }

        Polygons raftLines;
        int offset_from_poly_outline = 0;
        double fill_angle = train->getSettingAsCount("raft_surface_layers") > 0 ? 45 : 90;
        Infill infill_comp(EFillMethod::ZIG_ZAG, storage.raftOutline, offset_from_poly_outline, storage.raft_interface_config.getLineWidth(), train->getSettingInMicrons("raft_interface_line_spacing"), fill_overlap, fill_angle, z, extra_infill_shift);
        infill_comp.generate(raft_polygons, raftLines);
        gcode_layer.addLinesByOptimizer(raftLines, &storage.raft_interface_config, SpaceFillType::Lines);
        
        last_position_planned = gcode_layer.getLastPosition();
        current_extruder_planned = gcode_layer.getExtruder();
        is_inside_mesh_layer_part = gcode_layer.getIsInsideMesh();

        gcode_layer.processFanSpeedAndMinimalLayerTime();
        gcode_layer.overrideFanSpeeds(train->getSettingInPercentage("raft_interface_fan_speed"));
    }
    
    int layer_height = train->getSettingInMicrons("raft_surface_thickness");

    for (int raftSurfaceLayer = 1; raftSurfaceLayer <= train->getSettingAsCount("raft_surface_layers"); raftSurfaceLayer++)
    { // raft surface layers
        const int layer_nr = initial_raft_layer_nr + 2 + raftSurfaceLayer - 1; // 2: 1 base layer, 1 interface layer
        z += layer_height;
        const int64_t comb_offset = train->getSettingInMicrons("raft_surface_line_spacing");
        GCodePlanner& gcode_layer = layer_plan_buffer.emplace_back(storage, layer_nr, z, layer_height, last_position_planned, current_extruder_planned, is_inside_mesh_layer_part, fan_speed_layer_time_settings_per_extruder, combing_mode, comb_offset, train->getSettingBoolean("travel_avoid_other_parts"), train->getSettingInMicrons("travel_avoid_distance"));
        gcode_layer.setIsInside(true);

        if (CommandSocket::isInstantiated())
        {
            CommandSocket::getInstance()->sendOptimizedLayerInfo(layer_nr, z, layer_height);
        }
        
        Polygons raft_lines;
        int offset_from_poly_outline = 0;
        double fill_angle = 90 * raftSurfaceLayer;
        Infill infill_comp(EFillMethod::ZIG_ZAG, storage.raftOutline, offset_from_poly_outline, storage.raft_surface_config.getLineWidth(), train->getSettingInMicrons("raft_surface_line_spacing"), fill_overlap, fill_angle, z, extra_infill_shift);
        infill_comp.generate(raft_polygons, raft_lines);
        gcode_layer.addLinesByOptimizer(raft_lines, &storage.raft_surface_config, SpaceFillType::Lines);

        last_position_planned = gcode_layer.getLastPosition();
        current_extruder_planned = gcode_layer.getExtruder();
        is_inside_mesh_layer_part = gcode_layer.getIsInsideMesh();
        
        gcode_layer.processFanSpeedAndMinimalLayerTime();
        gcode_layer.overrideFanSpeeds(train->getSettingInPercentage("raft_surface_fan_speed"));
    }
}

void FffGcodeWriter::processLayer(SliceDataStorage& storage, int layer_nr, unsigned int total_layers)
{
    Progress::messageProgress(Progress::Stage::EXPORT, std::max(0, layer_nr) + 1, total_layers);
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

        if (CommandSocket::isInstantiated())
        {
            CommandSocket::getInstance()->sendOptimizedLayerInfo(layer_nr, z, layer_thickness);
        }
    }
    else
    {
        z = storage.meshes[0].layers[layer_nr].printZ;
        if (layer_nr == 0)
        {
            if (getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::RAFT)
            {
                include_helper_parts = false;
            }
            layer_thickness = getSettingInMicrons("layer_height_0");
        }
    }

    bool avoid_other_parts = false;
    coord_t avoid_distance = 0; // minimal avoid distance is zero
    for (int extr_nr = 0; extr_nr < storage.meshgroup->getExtruderCount(); extr_nr++)
    {
        if (gcode.getExtruderIsUsed(extr_nr))
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
    for (SettingsBaseVirtual& mesh_settings : storage.meshes)
    {
        max_inner_wall_width = std::max(max_inner_wall_width, mesh_settings.getSettingInMicrons((mesh_settings.getSettingAsCount("wall_line_count") > 1) ? "wall_line_width_x" : "wall_line_width_0")); 
    }
    int64_t comb_offset_from_outlines = max_inner_wall_width * 2;



    GCodePlanner& gcode_layer = layer_plan_buffer.emplace_back(storage, layer_nr, z, layer_thickness, last_position_planned, current_extruder_planned, is_inside_mesh_layer_part, fan_speed_layer_time_settings_per_extruder, getSettingAsCombingMode("retraction_combing"), comb_offset_from_outlines, avoid_other_parts, avoid_distance);

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
        processOozeShield(storage, gcode_layer, std::max(0, layer_nr));

        processDraftShield(storage, gcode_layer, std::max(0, layer_nr));
    }

    int support_skin_extruder_nr = getSettingAsIndex("support_interface_extruder_nr");
    int support_infill_extruder_nr = (layer_nr <= 0)? getSettingAsIndex("support_extruder_nr_layer_0") : getSettingAsIndex("support_infill_extruder_nr");

    std::vector<int> extruder_order = calculateExtruderOrder(storage, gcode_layer.getExtruder());
    for (int extruder_nr : extruder_order)
    {
        if (include_helper_parts
            && (extruder_nr == support_infill_extruder_nr || extruder_nr == support_skin_extruder_nr))
        {
            addSupportToGCode(storage, gcode_layer, layer_nr, extruder_nr);
        }

        if (layer_nr >= 0)
        {
            std::vector<unsigned int>& mesh_order = mesh_order_per_extruder[extruder_nr];
            unsigned int mesh_order_idx_starting_mesh = 0;
            { // calculate mesh_order_idx_starting_mesh
                Point layer_start_position = last_position_planned;
                if (storage.getSettingBoolean("start_layers_at_same_position"))
                {
                    layer_start_position = Point(storage.getSettingInMicrons("layer_start_x"), storage.getSettingInMicrons("layer_start_y"));
                }
                coord_t best_dist2 = std::numeric_limits<coord_t>::max();
                for (unsigned int mesh_order_idx = 0; mesh_order_idx < mesh_order.size(); mesh_order_idx++)
                {
                    unsigned int mesh_idx = mesh_order[mesh_order_idx];
                    SliceMeshStorage& mesh = storage.meshes[mesh_idx];
                    for (SliceLayerPart& part : mesh.layers[layer_nr].parts)
                    {
                        Point middle = (part.boundaryBox.min + part.boundaryBox.max) / 2;
                        coord_t dist2 = vSize2(middle - layer_start_position);
                        if (dist2 < best_dist2)
                        {
                            best_dist2 = dist2;
                            mesh_order_idx_starting_mesh = mesh_order_idx;
                        }
                    }
                }
            }

            for (unsigned int mesh_iterator_idx = 0; mesh_iterator_idx < mesh_order.size(); mesh_iterator_idx++)
            {
                unsigned int mesh_order_idx = (mesh_iterator_idx + mesh_order_idx_starting_mesh) % mesh_order.size();
                unsigned int mesh_idx = mesh_order[mesh_order_idx];
                SliceMeshStorage* mesh = &storage.meshes[mesh_idx];
                if (mesh->getSettingAsSurfaceMode("magic_mesh_surface_mode") == ESurfaceMode::SURFACE)
                {
                    addMeshLayerToGCode_meshSurfaceMode(storage, mesh, gcode_layer, layer_nr);
                }
                else
                {
                    addMeshLayerToGCode(storage, mesh, gcode_layer, layer_nr);
                }
            }
        }
    }

    if (layer_nr == 0 && getExtrudersNeedPrimeDuringFirstLayer())
    {
        ensureAllExtrudersArePrimed(storage, gcode_layer, layer_nr);
    }

    if (include_helper_parts)
    { // add prime tower if it hasn't already been added
        // print the prime tower if it hasn't been printed yet
        int prev_extruder = gcode_layer.getExtruder(); // most likely the same extruder as we are extruding with now
        addPrimeTower(storage, gcode_layer, layer_nr, prev_extruder);
    }
    
    last_position_planned = gcode_layer.getLastPosition();
    current_extruder_planned = gcode_layer.getExtruder();
    is_inside_mesh_layer_part = gcode_layer.getIsInsideMesh();
    
    gcode_layer.processFanSpeedAndMinimalLayerTime();
}

bool FffGcodeWriter::getExtrudersNeedPrimeDuringFirstLayer()
{
    switch(gcode.getFlavor())
    {
        case EGCodeFlavor::GRIFFIN:
            return true;
        default:
            return false; // TODO: change this once priming for other firmware types is implemented
    }
}

void FffGcodeWriter::ensureAllExtrudersArePrimed(SliceDataStorage& storage, GCodePlanner& gcode_layer, const int layer_nr)
{
    // Add prime for all extruders which haven't primed yet.

    std::vector<bool> extruder_is_used = storage.getExtrudersUsed();
    for (int extruder_nr = 0; extruder_nr < storage.meshgroup->getExtruderCount(); extruder_nr++)
    {
        if (extruder_is_used[extruder_nr] && !extruder_prime_is_planned[extruder_nr])
        { // prime before the current gcode layer plan is written to gcode
            setExtruder_addPrime(storage, gcode_layer, layer_nr, extruder_nr);
        }
    }
}

void FffGcodeWriter::processSkirtBrim(SliceDataStorage& storage, GCodePlanner& gcode_layer, unsigned int extruder_nr)
{
    if (skirt_brim_is_processed[extruder_nr])
    {
        return;
    }
    Polygons& skirt_brim = storage.skirt_brim[extruder_nr];
    skirt_brim_is_processed[extruder_nr] = true;
    if (skirt_brim.size() == 0)
    {
        return;
    }
    gcode_layer.addTravel(skirt_brim.back().closestPointTo(gcode_layer.getLastPosition()));
    gcode_layer.addPolygonsByOptimizer(skirt_brim, &storage.skirt_brim_config[extruder_nr]);
}

void FffGcodeWriter::processOozeShield(SliceDataStorage& storage, GCodePlanner& gcode_layer, unsigned int layer_nr)
{
    if (layer_nr == 0 && storage.getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::BRIM)
    {
        return; // ooze shield already generated by brim
    }
    if (storage.oozeShield.size() > 0 && layer_nr < storage.oozeShield.size())
    {
        gcode_layer.addPolygonsByOptimizer(storage.oozeShield[layer_nr], &storage.skirt_brim_config[0]); //TODO: Skirt and brim configuration index should correspond to draft shield extruder number.
    }
}

void FffGcodeWriter::processDraftShield(SliceDataStorage& storage, GCodePlanner& gcode_layer, unsigned int layer_nr)
{
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

    gcode_layer.addPolygonsByOptimizer(storage.draft_protection_shield, &storage.skirt_brim_config[0]); //TODO: Skirt and brim configuration index should correspond to draft shield extruder number.
}

std::vector<int> FffGcodeWriter::calculateExtruderOrder(SliceDataStorage& storage, int current_extruder)
{
    int extruder_count = storage.getSettingAsCount("machine_extruder_count");
    std::vector<int> ret;
    ret.push_back(current_extruder);
    for (int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        if (extruder_nr == current_extruder)
        { // skip the current extruder, it's the one we started out planning
            continue;
        }
        ret.push_back(extruder_nr);
    }
    assert(ret.size() == (size_t)extruder_count && "All extruders must be planned, even if later it appears one wasn't used.");
    return ret;
}

std::vector<unsigned int> FffGcodeWriter::calculateMeshOrder(SliceDataStorage& storage, int extruder_nr)
{
    OrderOptimizer<unsigned int> mesh_idx_order_optimizer;

    for (unsigned int mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
    {
        SliceMeshStorage& mesh = storage.meshes[mesh_idx];
        if (mesh.getSettingAsIndex("extruder_nr") == extruder_nr)
        {
            Mesh& mesh_data = storage.meshgroup->meshes[mesh_idx];
            Point3 middle = (mesh_data.getAABB().min + mesh_data.getAABB().max) / 2;
            mesh_idx_order_optimizer.addItem(Point(middle.x, middle.y), mesh_idx);
        }
    }
    std::list<unsigned int> mesh_indices_order = mesh_idx_order_optimizer.optimize();
    std::vector<unsigned int> ret;
    ret.reserve(mesh_indices_order.size());
    for (unsigned int mesh_order_idx : mesh_indices_order)
    {
        const unsigned int mesh_idx = mesh_idx_order_optimizer.items[mesh_order_idx].second;
        ret.push_back(mesh_idx);
    }
    return ret;
}

void FffGcodeWriter::addMeshLayerToGCode_meshSurfaceMode(SliceDataStorage& storage, SliceMeshStorage* mesh, GCodePlanner& gcode_layer, int layer_nr)
{
    if (layer_nr > mesh->layer_nr_max_filled_layer)
    {
        return;
    }

    if (mesh->getSettingBoolean("anti_overhang_mesh")
        || mesh->getSettingBoolean("support_mesh")
    )
    {
        return;
    }

    setExtruder_addPrime(storage, gcode_layer, layer_nr, mesh->getSettingAsIndex("extruder_nr"));

    SliceLayer* layer = &mesh->layers[layer_nr];


    Polygons polygons;
    for(unsigned int partNr=0; partNr<layer->parts.size(); partNr++)
    {
        polygons.add(layer->parts[partNr].outline);
    }

    EZSeamType z_seam_type = mesh->getSettingAsZSeamType("z_seam_type");
    Point z_seam_pos(mesh->getSettingInMicrons("z_seam_x"), mesh->getSettingInMicrons("z_seam_y"));
    gcode_layer.addPolygonsByOptimizer(polygons, &mesh->inset0_config, nullptr, z_seam_type, z_seam_pos, mesh->getSettingInMicrons("wall_0_wipe_dist"), mesh->getSettingBoolean("magic_spiralize"));

    addMeshOpenPolyLinesToGCode(storage, mesh, gcode_layer, layer_nr);
}

void FffGcodeWriter::addMeshOpenPolyLinesToGCode(SliceDataStorage& storage, SliceMeshStorage* mesh, GCodePlanner& gcode_layer, int layer_nr)
{
    SliceLayer* layer = &mesh->layers[layer_nr];
    
    Polygons lines;
    for(PolygonRef polyline : layer->openPolyLines)
    {
        for(unsigned int point_idx = 1; point_idx<polyline.size(); point_idx++)
        {
            Polygon p;
            p.add(polyline[point_idx-1]);
            p.add(polyline[point_idx]);
            lines.add(p);
        }
    }
    gcode_layer.addLinesByOptimizer(lines, &mesh->inset0_config, SpaceFillType::PolyLines);
    
}

void FffGcodeWriter::addMeshLayerToGCode(SliceDataStorage& storage, SliceMeshStorage* mesh, GCodePlanner& gcode_layer, int layer_nr)
{
    if (layer_nr > mesh->layer_nr_max_filled_layer)
    {
        return;
    }

    if (mesh->getSettingBoolean("anti_overhang_mesh")
        || mesh->getSettingBoolean("support_mesh")
    )
    {
        return;
    }

    SliceLayer* layer = &mesh->layers[layer_nr];

    if (layer->parts.size() == 0)
    {
        return;
    }

    if (mesh->getSettingAsCount("wall_line_count") > 0)
    { // don't switch extruder if there's nothing to print
        bool empty = true;
        for (SliceLayerPart& part : layer->parts)
        {
            if (part.insets.size() > 0)
            {
                empty = false;
                break;
            }
        }
        if (empty)
        {
            return;
        }
    }

    setExtruder_addPrime(storage, gcode_layer, layer_nr, mesh->getSettingAsIndex("extruder_nr"));

    EZSeamType z_seam_type = mesh->getSettingAsZSeamType("z_seam_type");
    Point z_seam_pos(mesh->getSettingInMicrons("z_seam_x"), mesh->getSettingInMicrons("z_seam_y"));
    Point layer_start_position = last_position_planned;
    if (storage.getSettingBoolean("start_layers_at_same_position"))
    {
        layer_start_position = Point(storage.getSettingInMicrons("layer_start_x"), storage.getSettingInMicrons("layer_start_y"));
    }
    PathOrderOptimizer part_order_optimizer(layer_start_position, z_seam_pos, z_seam_type);
    for(unsigned int partNr=0; partNr<layer->parts.size(); partNr++)
    {
        part_order_optimizer.addPolygon(layer->parts[partNr].insets[0][0]);
    }
    part_order_optimizer.optimize();

    for (int part_idx : part_order_optimizer.polyOrder)
    {
        SliceLayerPart& part = layer->parts[part_idx];
        addMeshPartToGCode(storage, mesh, part, gcode_layer, layer_nr);
    }
    if (mesh->getSettingAsSurfaceMode("magic_mesh_surface_mode") != ESurfaceMode::NORMAL)
    {
        addMeshOpenPolyLinesToGCode(storage, mesh, gcode_layer, layer_nr);
    }
}

void FffGcodeWriter::addMeshPartToGCode(SliceDataStorage& storage, SliceMeshStorage* mesh, SliceLayerPart& part, GCodePlanner& gcode_layer, int layer_nr)
{
    bool skin_alternate_rotation = mesh->getSettingBoolean("skin_alternate_rotation") && ( mesh->getSettingAsCount("top_layers") >= 4 || mesh->getSettingAsCount("bottom_layers") >= 4 );

    EFillMethod infill_pattern = mesh->getSettingAsFillMethod("infill_pattern");
    int infill_angle = 45;
    if ((infill_pattern == EFillMethod::LINES || infill_pattern == EFillMethod::ZIG_ZAG))
    {
        unsigned int combined_infill_layers = std::max(1U, round_divide(mesh->getSettingInMicrons("infill_sparse_thickness"), std::max(getSettingInMicrons("layer_height"), (coord_t)1)));
        if ((layer_nr / combined_infill_layers) & 1)
        { // switch every [combined_infill_layers] layers
            infill_angle += 90;
        }
    }
    
    int infill_line_distance = mesh->getSettingInMicrons("infill_line_distance");
    int infill_overlap = mesh->getSettingInMicrons("infill_overlap_mm");
    
    gcode_layer.setIsInside(true); // going to print inside stuff below
    
    if (mesh->getSettingBoolean("infill_before_walls"))
    {
        processMultiLayerInfill(gcode_layer, mesh, part, layer_nr, infill_line_distance, infill_overlap, infill_angle);
        processSingleLayerInfill(gcode_layer, mesh, part, layer_nr, infill_line_distance, infill_overlap, infill_angle);
    }

    EZSeamType z_seam_type = mesh->getSettingAsZSeamType("z_seam_type");
    Point z_seam_pos(mesh->getSettingInMicrons("z_seam_x"), mesh->getSettingInMicrons("z_seam_y"));
    processInsets(gcode_layer, mesh, part, layer_nr, z_seam_type, z_seam_pos);

    if (!mesh->getSettingBoolean("infill_before_walls"))
    {
        processMultiLayerInfill(gcode_layer, mesh, part, layer_nr, infill_line_distance, infill_overlap, infill_angle);
        processSingleLayerInfill(gcode_layer, mesh, part, layer_nr, infill_line_distance, infill_overlap, infill_angle);
    }

    EFillMethod skin_pattern = (layer_nr == 0)?
        mesh->getSettingAsFillMethod("top_bottom_pattern_0") :
        mesh->getSettingAsFillMethod("top_bottom_pattern");
    int skin_angle = 45;
    if ((skin_pattern == EFillMethod::LINES || skin_pattern == EFillMethod::ZIG_ZAG) && layer_nr & 1)
    {
        skin_angle += 90; // should coincide with infill_angle (if both skin and infill are lines) so that the first top layer is orthogonal to the last infill layer
    }
    if (skin_alternate_rotation && ( layer_nr / 2 ) & 1)
        skin_angle -= 45;

    int64_t skin_overlap = mesh->getSettingInMicrons("skin_overlap_mm");
    processSkinAndPerimeterGaps(gcode_layer, mesh, part, layer_nr, skin_overlap, skin_angle);

    //After a layer part, make sure the nozzle is inside the comb boundary, so we do not retract on the perimeter.
    if (!mesh->getSettingBoolean("magic_spiralize") || static_cast<int>(layer_nr) < mesh->getSettingAsCount("bottom_layers"))
    {
        gcode_layer.moveInsideCombBoundary(mesh->getSettingInMicrons((mesh->getSettingAsCount("wall_line_count") > 1) ? "wall_line_width_x" : "wall_line_width_0") * 1);
    }

    gcode_layer.setIsInside(false);
}

            


void FffGcodeWriter::processMultiLayerInfill(GCodePlanner& gcode_layer, SliceMeshStorage* mesh, SliceLayerPart& part, unsigned int layer_nr, int infill_line_distance, int infill_overlap, int infill_angle)
{
    int64_t z = layer_nr * getSettingInMicrons("layer_height");
    if (infill_line_distance > 0)
    {
        //Print the thicker infill lines first. (double or more layer thickness, infill combined with previous layers)
        for(unsigned int combine_idx = 1; combine_idx < part.infill_area_per_combine_per_density[0].size(); combine_idx++)
        {
            const unsigned int infill_line_width = mesh->infill_config[combine_idx].getLineWidth();
            EFillMethod infill_pattern = mesh->getSettingAsFillMethod("infill_pattern");
            Polygons infill_polygons;
            Polygons infill_lines;
            for (unsigned int density_idx = 0; density_idx < part.infill_area_per_combine_per_density.size(); density_idx++)
            { // combine different density infill areas (for gradual infill)
                unsigned int density_factor = 2 << density_idx; // == pow(2, density_idx + 1)
                int infill_line_distance_here = infill_line_distance * density_factor; // the highest density infill combines with the next to create a grid with density_factor 1
                int infill_shift = infill_line_distance_here / 2;
                if (density_idx == part.infill_area_per_combine_per_density.size() - 1)
                {
                    infill_line_distance_here /= 2;
                }
                
                Infill infill_comp(infill_pattern, part.infill_area_per_combine_per_density[density_idx][combine_idx], 0, infill_line_width, infill_line_distance_here, infill_overlap, infill_angle, z, infill_shift);
                infill_comp.generate(infill_polygons, infill_lines, mesh);
            }
            gcode_layer.addPolygonsByOptimizer(infill_polygons, &mesh->infill_config[combine_idx]);
            gcode_layer.addLinesByOptimizer(infill_lines, &mesh->infill_config[combine_idx], (infill_pattern == EFillMethod::ZIG_ZAG)? SpaceFillType::PolyLines : SpaceFillType::Lines);
        }
    }
}

void FffGcodeWriter::processSingleLayerInfill(GCodePlanner& gcode_layer, SliceMeshStorage* mesh, SliceLayerPart& part, unsigned int layer_nr, int infill_line_distance, int infill_overlap, int infill_angle)
{
    if (infill_line_distance == 0 || part.infill_area_per_combine_per_density[0].size() == 0)
    {
        return;
    }
    const unsigned int infill_line_width = mesh->infill_config[0].getLineWidth();
        
    //Combine the 1 layer thick infill with the top/bottom skin and print that as one thing.
    Polygons infill_polygons;
    Polygons infill_lines;

    int64_t z = layer_nr * getSettingInMicrons("layer_height");

    EFillMethod pattern = mesh->getSettingAsFillMethod("infill_pattern");
    for (unsigned int density_idx = 0; density_idx < part.infill_area_per_combine_per_density.size(); density_idx++)
    {
        unsigned int density_factor = 2 << density_idx; // == pow(2, density_idx + 1)
        int infill_line_distance_here = infill_line_distance * density_factor; // the highest density infill combines with the next to create a grid with density_factor 1
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
        if (density_idx == part.infill_area_per_combine_per_density.size() - 1)
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
            infill_line_distance_here /= 2;
        }
        Infill infill_comp(pattern, part.infill_area_per_combine_per_density[density_idx][0], 0, infill_line_width, infill_line_distance_here, infill_overlap, infill_angle, z, infill_shift);
        infill_comp.generate(infill_polygons, infill_lines, mesh);
    }
    gcode_layer.addPolygonsByOptimizer(infill_polygons, &mesh->infill_config[0]);
    if (pattern == EFillMethod::GRID || pattern == EFillMethod::LINES || pattern == EFillMethod::TRIANGLES)
    {
        gcode_layer.addLinesByOptimizer(infill_lines, &mesh->infill_config[0], SpaceFillType::Lines, mesh->getSettingInMicrons("infill_wipe_dist")); 
    }
    else 
    {
        gcode_layer.addLinesByOptimizer(infill_lines, &mesh->infill_config[0], (pattern == EFillMethod::ZIG_ZAG)? SpaceFillType::PolyLines : SpaceFillType::Lines); 
    }
}

void FffGcodeWriter::processInsets(GCodePlanner& gcode_layer, SliceMeshStorage* mesh, SliceLayerPart& part, unsigned int layer_nr, EZSeamType z_seam_type, Point z_seam_pos)
{
    bool compensate_overlap_0 = mesh->getSettingBoolean("travel_compensate_overlapping_walls_0_enabled");
    bool compensate_overlap_x = mesh->getSettingBoolean("travel_compensate_overlapping_walls_x_enabled");
    bool retract_before_outer_wall = mesh->getSettingBoolean("combing_retract_before_outer_wall");
    if (mesh->getSettingAsCount("wall_line_count") > 0)
    {
        bool spiralize = false;
        if (mesh->getSettingBoolean("magic_spiralize"))
        {
            if (static_cast<int>(layer_nr) >= mesh->getSettingAsCount("bottom_layers"))
            {
                spiralize = true;
            }
            if (static_cast<int>(layer_nr) == mesh->getSettingAsCount("bottom_layers") && part.insets.size() > 0)
            { // on the last normal layer first make the outer wall normally and then start a second outer wall from the same hight, but gradually moving upward
                WallOverlapComputation* wall_overlap_computation(nullptr);
                int wall_0_wipe_dist(0);
                gcode_layer.addPolygonsByOptimizer(part.insets[0], &mesh->insetX_config, wall_overlap_computation, EZSeamType::SHORTEST, z_seam_pos, wall_0_wipe_dist, spiralize);
            }
        }
        int processed_inset_number = -1;
        for(int inset_number=part.insets.size()-1; inset_number>-1; inset_number--)
        {
            processed_inset_number = inset_number;
            if (mesh->getSettingBoolean("outer_inset_first"))
            {
                processed_inset_number = part.insets.size() - 1 - inset_number;
            }
            if (processed_inset_number == 0)
            {
                if (!compensate_overlap_0)
                {
                    WallOverlapComputation* wall_overlap_computation(nullptr);
                    gcode_layer.addPolygonsByOptimizer(part.insets[0], &mesh->inset0_config, wall_overlap_computation, z_seam_type, z_seam_pos, mesh->getSettingInMicrons("wall_0_wipe_dist"), spiralize, retract_before_outer_wall);
                }
                else
                {
                    Polygons& outer_wall = part.insets[0];
                    WallOverlapComputation wall_overlap_computation(outer_wall, mesh->getSettingInMicrons("wall_line_width_0"));
                    gcode_layer.addPolygonsByOptimizer(outer_wall, &mesh->inset0_config, &wall_overlap_computation, z_seam_type, z_seam_pos, mesh->getSettingInMicrons("wall_0_wipe_dist"), spiralize, retract_before_outer_wall);
                }
            }
            else
            {
                if (!compensate_overlap_x)
                {
                    gcode_layer.addPolygonsByOptimizer(part.insets[processed_inset_number], &mesh->insetX_config);
                }
                else
                {
                    Polygons& outer_wall = part.insets[processed_inset_number];
                    WallOverlapComputation wall_overlap_computation(outer_wall, mesh->getSettingInMicrons("wall_line_width_x"));
                    gcode_layer.addPolygonsByOptimizer(outer_wall, &mesh->insetX_config, &wall_overlap_computation);
                }
            }
        }
    }
}


void FffGcodeWriter::processSkinAndPerimeterGaps(GCodePlanner& gcode_layer, SliceMeshStorage* mesh, SliceLayerPart& part, unsigned int layer_nr, int skin_overlap, int skin_angle)
{
    int64_t z = layer_nr * getSettingInMicrons("layer_height");
    const unsigned int skin_line_width = mesh->skin_config.getLineWidth();
    const unsigned int perimeter_gaps_line_width = mesh->perimeter_gap_config.getLineWidth();

    constexpr int perimeter_gaps_extra_offset = 15; // extra offset so that the perimeter gaps aren't created everywhere due to rounding errors
    bool fill_perimeter_gaps = mesh->getSettingAsFillPerimeterGapMode("fill_perimeter_gaps") != FillPerimeterGapMode::NOWHERE
                            && !mesh->getSettingBoolean("magic_spiralize");

    Point z_seam_pos(0, 0); // not used
    PathOrderOptimizer part_order_optimizer(gcode_layer.getLastPosition(), z_seam_pos, EZSeamType::SHORTEST);
    for (unsigned int skin_part_idx = 0; skin_part_idx < part.skin_parts.size(); skin_part_idx++)
    {
        PolygonsPart& outline = part.skin_parts[skin_part_idx].outline;
        part_order_optimizer.addPolygon(outline.outerPolygon());
    }
    part_order_optimizer.optimize();

    for (int ordered_skin_part_idx : part_order_optimizer.polyOrder)
    {
        SkinPart& skin_part = part.skin_parts[ordered_skin_part_idx];

        Polygons skin_polygons;
        Polygons skin_lines;
        
        EFillMethod pattern = (layer_nr == 0)?
            mesh->getSettingAsFillMethod("top_bottom_pattern_0") :
            mesh->getSettingAsFillMethod("top_bottom_pattern");
        int bridge = -1;
        if (layer_nr > 0)
            bridge = bridgeAngle(skin_part.outline, &mesh->layers[layer_nr-1]);
        if (bridge > -1)
        {
            pattern = EFillMethod::LINES;
            skin_angle = bridge;
        }

        Polygons perimeter_gaps; // the perimeter gaps of the insets of this skin part

        Polygons* inner_skin_outline = nullptr;
        int offset_from_inner_skin_outline = 0;
        if (pattern != EFillMethod::CONCENTRIC)
        {
            for (Polygons& skin_perimeter : skin_part.insets)
            {
                gcode_layer.addPolygonsByOptimizer(skin_perimeter, &mesh->insetX_config); // add polygons to gcode in inward order
            }
            if (skin_part.insets.size() > 0)
            {
                inner_skin_outline = &skin_part.insets.back();
                offset_from_inner_skin_outline = -mesh->insetX_config.getLineWidth() / 2;

                if (fill_perimeter_gaps)
                {
                    // add perimeter gaps between the outer skin inset and the innermost wall
                    const Polygons outer = skin_part.outline;
                    const Polygons inner = skin_part.insets[0].offset(mesh->insetX_config.getLineWidth() / 2 + perimeter_gaps_extra_offset);
                    perimeter_gaps.add(outer.difference(inner));

                    for (unsigned int inset_idx = 1; inset_idx < skin_part.insets.size(); inset_idx++)
                    { // add perimeter gaps between consecutive skin walls
                        const Polygons outer = skin_part.insets[inset_idx - 1].offset(-1 * mesh->insetX_config.getLineWidth() / 2 - perimeter_gaps_extra_offset);
                        const Polygons inner = skin_part.insets[inset_idx].offset(mesh->insetX_config.getLineWidth() / 2);
                        perimeter_gaps.add(outer.difference(inner));
                    }
                }
            }
        }

        if (inner_skin_outline == nullptr)
        {
            inner_skin_outline = &skin_part.outline;
        }

        int extra_infill_shift = 0;
        Polygons* perimeter_gaps_output = (fill_perimeter_gaps)? &perimeter_gaps : nullptr;
        Infill infill_comp(pattern, *inner_skin_outline, offset_from_inner_skin_outline, skin_line_width, skin_line_width, skin_overlap, skin_angle, z, extra_infill_shift, perimeter_gaps_output);
        infill_comp.generate(skin_polygons, skin_lines);

        gcode_layer.addPolygonsByOptimizer(skin_polygons, &mesh->skin_config);

        if (pattern == EFillMethod::GRID || pattern == EFillMethod::LINES || pattern == EFillMethod::TRIANGLES)
        {
            gcode_layer.addLinesByOptimizer(skin_lines, &mesh->skin_config, SpaceFillType::Lines, mesh->getSettingInMicrons("infill_wipe_dist"));
        }
        else
        {
            gcode_layer.addLinesByOptimizer(skin_lines, &mesh->skin_config, (pattern == EFillMethod::ZIG_ZAG)? SpaceFillType::PolyLines : SpaceFillType::Lines);
        }

        if (fill_perimeter_gaps)
        { // handle perimeter_gaps of skin insets
            Polygons gap_polygons; // will remain empty
            Polygons gap_lines;
            int offset = 0;
            Infill infill_comp(EFillMethod::LINES, perimeter_gaps, offset, perimeter_gaps_line_width, perimeter_gaps_line_width, skin_overlap, skin_angle, z, extra_infill_shift);
            infill_comp.generate(gap_polygons, gap_lines);
            gcode_layer.addLinesByOptimizer(gap_lines, &mesh->perimeter_gap_config, SpaceFillType::Lines);
        }
    }

    if (fill_perimeter_gaps)
    { // handle perimeter gaps of normal insets
        Polygons perimeter_gaps;
        int line_width = mesh->inset0_config.getLineWidth();
        for (unsigned int inset_idx = 0; inset_idx < part.insets.size() - 1; inset_idx++)
        {
            const Polygons outer = part.insets[inset_idx].offset(-1 * line_width / 2 - perimeter_gaps_extra_offset);
            line_width = mesh->insetX_config.getLineWidth();

            Polygons inner = part.insets[inset_idx + 1].offset(line_width / 2);
            perimeter_gaps.add(outer.difference(inner));
        }
        { // gap between inner wall and skin/infill
            if (mesh->getSettingInMicrons("infill_line_distance") > 0
                && !mesh->getSettingBoolean("infill_hollow")
                && mesh->getSettingInMicrons("infill_overlap_mm") >= 0
            )
            {
                const Polygons outer = part.insets.back().offset(-1 * line_width / 2 - perimeter_gaps_extra_offset);

                Polygons inner = part.infill_area;
                for (SkinPart& skin_part : part.skin_parts)
                {
                    inner.add(skin_part.outline);
                }
                inner = inner.unionPolygons();
                perimeter_gaps.add(outer.difference(inner));
            }
        }

        Polygons gap_polygons; // unused
        Polygons gap_lines; // soon to be generated gap filler lines
        int offset = 0;
        int extra_infill_shift = 0;
        Infill infill_comp(EFillMethod::LINES, perimeter_gaps, offset, perimeter_gaps_line_width, perimeter_gaps_line_width, skin_overlap, skin_angle, z, extra_infill_shift);
        infill_comp.generate(gap_polygons, gap_lines);

        gcode_layer.addLinesByOptimizer(gap_lines, &mesh->perimeter_gap_config, SpaceFillType::Lines);
    }
}

bool FffGcodeWriter::addSupportToGCode(SliceDataStorage& storage, GCodePlanner& gcode_layer, int layer_nr, int extruder_nr)
{
    bool support_added = false;
    if (!storage.support.generated || layer_nr > storage.support.layer_nr_max_filled_layer)
    {
        return support_added;
    }

    int support_interface_extruder_nr = getSettingAsIndex("support_interface_extruder_nr");
    int support_infill_extruder_nr = (layer_nr <= 0)? getSettingAsIndex("support_extruder_nr_layer_0") : getSettingAsIndex("support_infill_extruder_nr");

    SupportLayer& support_layer = storage.support.supportLayers[layer_nr];
    if (support_layer.skin.size() == 0 && support_layer.supportAreas.size() == 0)
    {
        return support_added;
    }


    if (extruder_nr == support_infill_extruder_nr)
    {
        support_added |= addSupportInfillToGCode(storage, gcode_layer, layer_nr);
    }
    if (extruder_nr == support_interface_extruder_nr)
    {
        support_added |= addSupportRoofsToGCode(storage, gcode_layer, layer_nr);
    }
    return support_added;
}

bool FffGcodeWriter::addSupportInfillToGCode(SliceDataStorage& storage, GCodePlanner& gcode_layer, int layer_nr)
{
    bool added = false;
    if (!storage.support.generated 
        || layer_nr > storage.support.layer_nr_max_filled_layer 
        || storage.support.supportLayers[layer_nr].supportAreas.size() == 0)
    {
        return added;
    }

    int64_t z = layer_nr * getSettingInMicrons("layer_height");

    const ExtruderTrain& infill_extr = *storage.meshgroup->getExtruderTrain(getSettingAsIndex("support_infill_extruder_nr"));
    int support_line_distance = infill_extr.getSettingInMicrons("support_line_distance"); // first layer line distance must be the same as the second layer line distance
    const int support_line_width = storage.support_config.getLineWidth();
    EFillMethod support_pattern = infill_extr.getSettingAsFillMethod("support_pattern"); // first layer pattern must be same as other layers
    if (layer_nr <= 0 && (support_pattern == EFillMethod::LINES || support_pattern == EFillMethod::ZIG_ZAG)) { support_pattern = EFillMethod::GRID; }

    int infill_extruder_nr_here = (layer_nr <= 0)? getSettingAsIndex("support_extruder_nr_layer_0") : getSettingAsIndex("support_infill_extruder_nr");
    const ExtruderTrain& infill_extr_here = *storage.meshgroup->getExtruderTrain(infill_extruder_nr_here);

    Polygons& support = storage.support.supportLayers[std::max(0, layer_nr)].supportAreas;

    std::vector<PolygonsPart> support_islands = support.splitIntoParts();

    PathOrderOptimizer island_order_optimizer(gcode_layer.getLastPosition());
    for(unsigned int n=0; n<support_islands.size(); n++)
    {
        island_order_optimizer.addPolygon(support_islands[n][0]);
    }
    island_order_optimizer.optimize();

    for(unsigned int n=0; n<support_islands.size(); n++)
    {
        PolygonsPart& island = support_islands[island_order_optimizer.polyOrder[n]];

        int support_infill_overlap = 0; // support infill should not be expanded outward
        
        int offset_from_outline = 0;
        if (support_pattern == EFillMethod::GRID || support_pattern == EFillMethod::TRIANGLES)
        {
            Polygons boundary = island.offset(-support_line_width / 2);
            if (boundary.size() > 0)
            {
                setExtruder_addPrime(storage, gcode_layer, layer_nr, infill_extruder_nr_here); // only switch extruder if we're sure we're going to switch
                gcode_layer.addPolygonsByOptimizer(boundary, &storage.support_config);
            }
            offset_from_outline = -support_line_width;
            support_infill_overlap = infill_extr_here.getSettingInMicrons("infill_overlap_mm"); // support lines area should be expanded outward to overlap with the boundary polygon
        }

        int extra_infill_shift = 0;
        bool use_endpieces = true;
        Polygons* perimeter_gaps = nullptr;
        double fill_angle = 0;
        Infill infill_comp(support_pattern, island, offset_from_outline, support_line_width, support_line_distance, support_infill_overlap, fill_angle, z, extra_infill_shift, perimeter_gaps, infill_extr.getSettingBoolean("support_connect_zigzags"), use_endpieces);
        Polygons support_polygons;
        Polygons support_lines;
        infill_comp.generate(support_polygons, support_lines);
        if (support_lines.size() > 0 || support_polygons.size() > 0)
        {
            setExtruder_addPrime(storage, gcode_layer, layer_nr, infill_extruder_nr_here); // only switch extruder if we're sure we're going to switch
            gcode_layer.addPolygonsByOptimizer(support_polygons, &storage.support_config);
            gcode_layer.addLinesByOptimizer(support_lines, &storage.support_config, (support_pattern == EFillMethod::ZIG_ZAG)? SpaceFillType::PolyLines : SpaceFillType::Lines);
            added = true;
        }
    }
    return added;
}

bool FffGcodeWriter::addSupportRoofsToGCode(SliceDataStorage& storage, GCodePlanner& gcode_layer, int layer_nr)
{
    bool added = false;
    if (!storage.support.generated 
        || layer_nr > storage.support.layer_nr_max_filled_layer 
        || storage.support.supportLayers[std::max(0, layer_nr)].skin.size() == 0)
    {
        return added;
    }

    int64_t z = layer_nr * getSettingInMicrons("layer_height");

    int skin_extruder_nr = getSettingAsIndex("support_interface_extruder_nr");
    const ExtruderTrain& interface_extr = *storage.meshgroup->getExtruderTrain(skin_extruder_nr);

    EFillMethod pattern = interface_extr.getSettingAsFillMethod("support_interface_pattern");
    int support_line_distance = interface_extr.getSettingInMicrons("support_interface_line_distance");
    
    
    bool all_roofs_are_low = true;
    for (SliceMeshStorage& mesh : storage.meshes)
    {
        if (mesh.getSettingInMicrons("support_roof_height") >= 2 * getSettingInMicrons("layer_height"))
        {
            all_roofs_are_low = false;
            break;
        }
    }
    
    double fillAngle;
    if (pattern == EFillMethod::CONCENTRIC)
    {
        fillAngle = 0;
    }
    else if (all_roofs_are_low || pattern == EFillMethod::TRIANGLES)
    {
        fillAngle = 90; // perpendicular to support lines
    }
    else 
    {
        fillAngle = 45 + (((layer_nr % 2) + 2) % 2) * 90; // alternate between the two kinds of diagonal:  / and \ .
        // +2) %2 to handle negative layer numbers
    }
    int support_skin_overlap = 0; // the skin (roofs/bottoms) should never be expanded outwards
    int outline_offset =  0;
    int extra_infill_shift = 0;
    Polygons* perimeter_gaps = nullptr;
    bool use_endpieces = true;
    bool connected_zigzags = false;
    Infill infill_comp(pattern, storage.support.supportLayers[std::max(0, layer_nr)].skin, outline_offset, storage.support_skin_config.getLineWidth(), support_line_distance, support_skin_overlap, fillAngle, z, extra_infill_shift, perimeter_gaps, connected_zigzags, use_endpieces);
    Polygons support_polygons;
    Polygons support_lines;
    infill_comp.generate(support_polygons, support_lines);

    if (support_lines.size() > 0 || support_polygons.size() > 0)
    {
        setExtruder_addPrime(storage, gcode_layer, layer_nr, skin_extruder_nr);
        gcode_layer.addPolygonsByOptimizer(support_polygons, &storage.support_skin_config);
        gcode_layer.addLinesByOptimizer(support_lines, &storage.support_skin_config, (pattern == EFillMethod::ZIG_ZAG)? SpaceFillType::PolyLines : SpaceFillType::Lines);
        added = true;
    }
    return added;
}

void FffGcodeWriter::setExtruder_addPrime(SliceDataStorage& storage, GCodePlanner& gcode_layer, int layer_nr, int extruder_nr)
{
    if (extruder_nr == -1) // an object with extruder_nr==-1 means it will be printed with any current nozzle
        return;
    
    int previous_extruder = gcode_layer.getExtruder();
    if (previous_extruder == extruder_nr) { return; }
    bool extruder_changed = gcode_layer.setExtruder(extruder_nr);
    
    if (extruder_changed)
    {
        if (!extruder_prime_is_planned[extruder_nr])
        {
            ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder_nr);

            // move to prime position
            bool prime_pos_is_abs = train->getSettingBoolean("extruder_prime_pos_abs");
            Point prime_pos = Point(train->getSettingInMicrons("extruder_prime_pos_x"), train->getSettingInMicrons("extruder_prime_pos_y"));
            gcode_layer.addTravel(prime_pos_is_abs? prime_pos : gcode_layer.getLastPosition() + prime_pos);

            gcode_layer.planPrime();

            extruder_prime_is_planned[extruder_nr] = true;
        }

        assert(extruder_prime_is_planned[extruder_nr] && "extruders should be primed before they are used!");
        if (layer_nr == 0 && !skirt_brim_is_processed[extruder_nr])
        {
            processSkirtBrim(storage, gcode_layer, extruder_nr);
        }
        if (layer_nr >= -Raft::getFillerLayerCount(storage))
        {
            addPrimeTower(storage, gcode_layer, layer_nr, previous_extruder);
        }
    }
}

void FffGcodeWriter::addPrimeTower(SliceDataStorage& storage, GCodePlanner& gcode_layer, int layer_nr, int prev_extruder)
{
    if (!getSettingBoolean("prime_tower_enable"))
    {
        return;
    }

    storage.primeTower.addToGcode(storage, gcode_layer, gcode, layer_nr, prev_extruder, gcode_layer.getExtruder());
}

void FffGcodeWriter::finalize()
{
    double print_time = gcode.getTotalPrintTime();
    std::vector<double> filament_used;
    std::vector<std::string> material_ids;
    for (int extr_nr = 0; extr_nr < getSettingAsCount("machine_extruder_count"); extr_nr++)
    {
        filament_used.emplace_back(gcode.getTotalFilamentUsed(extr_nr));
        material_ids.emplace_back(gcode.getMaterialGUID(extr_nr));
    }
    std::string prefix = gcode.getFileHeader(&print_time, filament_used, material_ids);
    if (CommandSocket::isInstantiated())
    {
        CommandSocket::getInstance()->sendGCodePrefix(prefix);
    }
    else
    {
        std::string prefix = gcode.getFileHeader(&print_time, filament_used, material_ids);
        log("Gcode header after slicing:\n");
        log(prefix.c_str());
        log("End of gcode header.\n");
    }
    if (getSettingBoolean("acceleration_enabled"))
    {
        gcode.writeAcceleration(getSettingInMillimetersPerSecond("machine_acceleration"));
    }
    if (getSettingBoolean("jerk_enabled"))
    {
        gcode.writeJerk(getSettingInMillimetersPerSecond("machine_max_jerk_xy"));
    }
    if (gcode.getCurrentMaxZFeedrate() > 0)
    {
        gcode.writeMaxZFeedrate(getSettingInMillimetersPerSecond("machine_max_feedrate_z"));
    }
    gcode.finalize(getSettingString("machine_end_gcode").c_str());
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

