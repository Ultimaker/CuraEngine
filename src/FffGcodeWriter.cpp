
#include <list>

#include "FffGcodeWriter.h"
#include "FffProcessor.h"
#include "progress/Progress.h"
#include "wallOverlap.h"

namespace cura
{


void FffGcodeWriter::writeGCode(SliceDataStorage& storage, TimeKeeper& time_keeper)
{
    gcode.preSetup(storage.meshgroup);
    
    if (FffProcessor::getInstance()->getMeshgroupNr() == 0)
    { // first meshgroup
        gcode.resetTotalPrintTimeAndFilament();
        gcode.setInitialTemps(*storage.meshgroup);
    }
    
    if (CommandSocket::isInstantiated())
    {
        CommandSocket::getInstance()->beginGCode();
    }

    setConfigFanSpeedLayerTime();
    
    setConfigCoasting(storage);

    setConfigRetraction(storage);
    
    initConfigs(storage);
    
    for (int extruder = 0; extruder < storage.meshgroup->getExtruderCount(); extruder++)
    { // skirt
        storage.skirt_config[extruder].setLayerHeight(getSettingInMicrons("layer_height_0"));
    }
    
    
    layer_plan_buffer.setPreheatConfig(*storage.meshgroup);
    
    if (FffProcessor::getInstance()->getMeshgroupNr() == 0)
    {
        processStartingCode(storage);
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
    }
    
    for (int extruder = 0; extruder < storage.meshgroup->getExtruderCount(); extruder++)
        last_prime_tower_poly_printed[extruder] = -1; // layer 0 has its prime tower printed during the brim (?)
    
    for(unsigned int layer_nr=0; layer_nr<total_layers; layer_nr++)
    {
        processLayer(storage, layer_nr, total_layers, has_raft);
    }
    
    Progress::messageProgressStage(Progress::Stage::FINISH, &time_keeper);

    //Store the object height for when we are printing multiple objects, as we need to clear every one of them when moving to the next position.
    max_object_height = std::max(max_object_height, storage.model_max.z);

    layer_plan_buffer.flush();
}

void FffGcodeWriter::setConfigFanSpeedLayerTime()
{
    fan_speed_layer_time_settings.cool_min_layer_time = getSettingInSeconds("cool_min_layer_time");
    fan_speed_layer_time_settings.cool_min_layer_time_fan_speed_max = getSettingInSeconds("cool_min_layer_time_fan_speed_max");
    fan_speed_layer_time_settings.cool_fan_speed_min = getSettingInPercentage("cool_fan_speed_min");
    fan_speed_layer_time_settings.cool_fan_speed_max = getSettingInPercentage("cool_fan_speed_max");
    fan_speed_layer_time_settings.cool_min_speed = getSettingInMillimetersPerSecond("cool_min_speed");
    fan_speed_layer_time_settings.cool_fan_full_layer = getSettingAsCount("cool_fan_full_layer");
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
    }
    for(SliceMeshStorage& mesh : storage.meshes)
    {
        mesh.retraction_config.distance = (mesh.getSettingBoolean("retraction_enable"))? mesh.getSettingInMillimeters("retraction_amount") : 0;
        mesh.retraction_config.prime_volume = mesh.getSettingInCubicMillimeters("retraction_extra_prime_amount");
        mesh.retraction_config.speed = mesh.getSettingInMillimetersPerSecond("retraction_retract_speed");
        mesh.retraction_config.primeSpeed = mesh.getSettingInMillimetersPerSecond("retraction_prime_speed");
        mesh.retraction_config.zHop = mesh.getSettingInMicrons("retraction_hop");
        mesh.retraction_config.retraction_min_travel_distance = mesh.getSettingInMicrons("retraction_min_travel");
        mesh.retraction_config.retraction_extrusion_window = mesh.getSettingInMillimeters("retraction_extrusion_window");
        mesh.retraction_config.retraction_count_max = mesh.getSettingAsCount("retraction_count_max");
    }
}

void FffGcodeWriter::initConfigs(SliceDataStorage& storage)
{
    storage.travel_config.init(getSettingInMillimetersPerSecond("speed_travel"), 0, 0);
    
    for (int extruder = 0; extruder < storage.meshgroup->getExtruderCount(); extruder++)
    { // skirt
        SettingsBase* train = storage.meshgroup->getExtruderTrain(extruder);
        storage.skirt_config[extruder].init(train->getSettingInMillimetersPerSecond("skirt_speed"), train->getSettingInMicrons("skirt_line_width"), train->getSettingInPercentage("material_flow"));
    }

    { // support 
        SettingsBase* train = storage.meshgroup->getExtruderTrain(getSettingAsIndex("support_infill_extruder_nr"));
        storage.support_config.init(getSettingInMillimetersPerSecond("speed_support_infill"), getSettingInMicrons("support_line_width"), train->getSettingInPercentage("material_flow"));
        
        storage.support_roof_config.init(getSettingInMillimetersPerSecond("speed_support_roof"), getSettingInMicrons("support_roof_line_width"), train->getSettingInPercentage("material_flow"));
    }
    
    for (SliceMeshStorage& mesh : storage.meshes)
    {
        mesh.inset0_config.init(mesh.getSettingInMillimetersPerSecond("speed_wall_0"), mesh.getSettingInMicrons("wall_line_width_0"), mesh.getSettingInPercentage("material_flow"));
        mesh.insetX_config.init(mesh.getSettingInMillimetersPerSecond("speed_wall_x"), mesh.getSettingInMicrons("wall_line_width_x"), mesh.getSettingInPercentage("material_flow"));
        mesh.skin_config.init(mesh.getSettingInMillimetersPerSecond("speed_topbottom"), mesh.getSettingInMicrons("skin_line_width"), mesh.getSettingInPercentage("material_flow"));
    
        for(unsigned int idx=0; idx<MAX_INFILL_COMBINE; idx++)
        {
            mesh.infill_config[idx].init(mesh.getSettingInMillimetersPerSecond("speed_infill"), mesh.getSettingInMicrons("infill_line_width") * (idx + 1), mesh.getSettingInPercentage("material_flow"));
        }
    }
    
    storage.primeTower.initConfigs(storage.meshgroup, storage.retraction_config_per_extruder);
}

void FffGcodeWriter::processStartingCode(SliceDataStorage& storage)
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
            if (getSettingBoolean("machine_heated_bed") && getSettingInDegreeCelsius("material_bed_temperature") > 0)
            {
                gcode.writeBedTemperatureCommand(getSettingInDegreeCelsius("material_bed_temperature"), getSettingBoolean("material_bed_temp_wait"));
            }
        }

        if (getSettingBoolean("material_print_temp_prepend")) 
        {
            for (int extruder_nr = 0; extruder_nr < storage.getSettingAsCount("extruder_count"); extruder_nr++)
            {
                double print_temp = storage.meshgroup->getExtruderTrain(extruder_nr)->getSettingInDegreeCelsius("material_print_temperature");
                gcode.writeTemperatureCommand(extruder_nr, print_temp);
            }
            if (getSettingBoolean("material_print_temp_wait")) 
            {
                for (int extruder_nr = 0; extruder_nr < storage.getSettingAsCount("extruder_count"); extruder_nr++)
                {
                    double print_temp = storage.meshgroup->getExtruderTrain(extruder_nr)->getSettingInDegreeCelsius("material_print_temperature");
                    gcode.writeTemperatureCommand(extruder_nr, print_temp, true);
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
        gcode.writeCode("G92 E0"); // E-value already assumed to be at E0, but writing it just to be safe...
//         G1 X175 Y6 Z20 F9000
        gcode.writeMove(FPoint3(175, 6, 2).toPoint3(), getSettingInMillimetersPerSecond("speed_travel"), 0.0);
        gcode.writePrimeTrain();
        gcode.switchExtruder(1);
//         G1 X180 Y6 Z20 F9000
        gcode.writeMove(FPoint3(198, 6, 2).toPoint3(), getSettingInMillimetersPerSecond("speed_travel"), 0.0);
        gcode.writeTemperatureCommand(1, storage.meshgroup->getExtruderTrain(1)->getSettingInDegreeCelsius("material_print_temperature"), true); // TODO: this is a hack job which should get fixed as soon as we prime the first time we need to
        gcode.writePrimeTrain();
        gcode.writeTemperatureCommand(1, storage.meshgroup->getExtruderTrain(1)->getSettingInDegreeCelsius("material_standby_temperature"), false); // TODO: this is a hack job which should get fixed as soon as we prime the first time we need to
    }
}

void FffGcodeWriter::processNextMeshGroupCode(SliceDataStorage& storage)
{
    gcode.writeFanCommand(0);
    gcode.resetExtrusionValue();
    gcode.setZ(max_object_height + 5000);
    gcode.writeMove(gcode.getPositionXY(), getSettingInMillimetersPerSecond("speed_travel"), 0);
    last_position_planned = Point(storage.model_min.x, storage.model_min.y);
    gcode.writeMove(last_position_planned, getSettingInMillimetersPerSecond("speed_travel"), 0);
}
    
void FffGcodeWriter::processRaft(SliceDataStorage& storage, unsigned int total_layers)
{
    int extruder_nr = getSettingAsIndex("adhesion_extruder_nr");
    ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder_nr);
    
    CombingMode combing_mode = storage.getSettingAsCombingMode("retraction_combing"); 
    
    int n_raft_surface_layers = train->getSettingAsCount("raft_surface_layers");
    
    int z = 0;
    
    { // set configs 
        storage.raft_base_config.init(train->getSettingInMillimetersPerSecond("raft_base_speed"), train->getSettingInMicrons("raft_base_line_width"), train->getSettingInPercentage("material_flow"));
        storage.raft_base_config.setLayerHeight(train->getSettingInMicrons("raft_base_thickness"));
        
        storage.raft_interface_config.init(train->getSettingInMillimetersPerSecond("raft_interface_speed"), train->getSettingInMicrons("raft_interface_line_width"), train->getSettingInPercentage("material_flow"));
        storage.raft_interface_config.setLayerHeight(train->getSettingInMicrons("raft_interface_thickness"));

        storage.raft_surface_config.init(train->getSettingInMillimetersPerSecond("raft_surface_speed"), train->getSettingInMicrons("raft_surface_line_width"), train->getSettingInPercentage("material_flow"));
        storage.raft_surface_config.setLayerHeight(train->getSettingInMicrons("raft_surface_thickness"));
    }
    
    // some infill config for all lines infill generation below
    int offset_from_poly_outline = 0;
    double fill_overlap = 0; // raft line shouldn't be expanded - there is no boundary polygon printed
    Polygons raft_polygons; // should remain empty, since we only have the lines pattern for the raft...
    
    { // raft base layer
        
        int layer_nr = -n_raft_surface_layers - 2;
        int layer_height = train->getSettingInMicrons("raft_base_thickness");
        z += layer_height;
        int64_t comb_offset = train->getSettingInMicrons("raft_base_line_spacing");
        GCodePlanner& gcode_layer = layer_plan_buffer.emplace_back(storage, layer_nr, z, layer_height, last_position_planned, current_extruder_planned, is_inside_mesh_layer_part, fan_speed_layer_time_settings, combing_mode, comb_offset, train->getSettingBoolean("travel_avoid_other_parts"), train->getSettingInMicrons("travel_avoid_distance"));
        
        if (getSettingAsIndex("adhesion_extruder_nr") > 0)
        {
            gcode_layer.setExtruder(extruder_nr);
        }
        if (CommandSocket::isInstantiated())
        {
            CommandSocket::getInstance()->sendLayerInfo(layer_nr, z, layer_height);
        }
        gcode_layer.addPolygonsByOptimizer(storage.raftOutline, &storage.raft_base_config);

        Polygons raftLines;
        double fill_angle = 0;
        Infill infill_comp(EFillMethod::LINES, storage.raftOutline, offset_from_poly_outline, storage.raft_base_config.getLineWidth(), train->getSettingInMicrons("raft_base_line_spacing"), fill_overlap, fill_angle);
        infill_comp.generate(raft_polygons, raftLines);
        gcode_layer.addLinesByOptimizer(raftLines, &storage.raft_base_config, SpaceFillType::Lines);

        last_position_planned = gcode_layer.getLastPosition();
        current_extruder_planned = gcode_layer.getExtruder();
        is_inside_mesh_layer_part = gcode_layer.getIsInsideMesh();
        
        gcode_layer.setFanSpeed(train->getSettingInPercentage("raft_base_fan_speed"));
        gcode_layer.processFanSpeedAndMinimalLayerTime();
    }

    { // raft interface layer
        int layer_nr = -n_raft_surface_layers - 1;
        int layer_height = train->getSettingInMicrons("raft_interface_thickness");
        z += layer_height;
        int64_t comb_offset = train->getSettingInMicrons("raft_interface_line_spacing");
        GCodePlanner& gcode_layer = layer_plan_buffer.emplace_back(storage, layer_nr, z, layer_height, last_position_planned, current_extruder_planned, is_inside_mesh_layer_part, fan_speed_layer_time_settings, combing_mode, comb_offset, train->getSettingBoolean("travel_avoid_other_parts"), train->getSettingInMicrons("travel_avoid_distance"));
        
        if (CommandSocket::isInstantiated())
        {
            CommandSocket::getInstance()->sendLayerInfo(layer_nr, z, layer_height);
        }
        
        Polygons raftLines;
        int offset_from_poly_outline = 0;
        double fill_angle = train->getSettingAsCount("raft_surface_layers") > 0 ? 45 : 90;
        Infill infill_comp(EFillMethod::LINES, storage.raftOutline, offset_from_poly_outline, storage.raft_interface_config.getLineWidth(), train->getSettingInMicrons("raft_interface_line_spacing"), fill_overlap, fill_angle);
        infill_comp.generate(raft_polygons, raftLines);
        gcode_layer.addLinesByOptimizer(raftLines, &storage.raft_interface_config, SpaceFillType::Lines);
        
        last_position_planned = gcode_layer.getLastPosition();
        current_extruder_planned = gcode_layer.getExtruder();
        is_inside_mesh_layer_part = gcode_layer.getIsInsideMesh();

        gcode_layer.setFanSpeed(train->getSettingInPercentage("raft_interface_fan_speed"));
        gcode_layer.processFanSpeedAndMinimalLayerTime();
    }
    
    int layer_height = train->getSettingInMicrons("raft_surface_thickness");

    for (int raftSurfaceLayer=1; raftSurfaceLayer <= n_raft_surface_layers; raftSurfaceLayer++)
    { // raft surface layers
        int layer_nr = -n_raft_surface_layers + raftSurfaceLayer - 1;
        z += layer_height;
        int64_t comb_offset = train->getSettingInMicrons("raft_surface_line_spacing");
        GCodePlanner& gcode_layer = layer_plan_buffer.emplace_back(storage, layer_nr, z, layer_height, last_position_planned, current_extruder_planned, is_inside_mesh_layer_part, fan_speed_layer_time_settings, combing_mode, comb_offset, train->getSettingBoolean("travel_avoid_other_parts"), train->getSettingInMicrons("travel_avoid_distance"));
        
        if (CommandSocket::isInstantiated())
        {
            CommandSocket::getInstance()->sendLayerInfo(layer_nr, z, layer_height);
        }
        
        Polygons raft_lines;
        int offset_from_poly_outline = 0;
        double fill_angle = 90 * raftSurfaceLayer;
        Infill infill_comp(EFillMethod::LINES, storage.raftOutline, offset_from_poly_outline, storage.raft_surface_config.getLineWidth(), train->getSettingInMicrons("raft_surface_line_spacing"), fill_overlap, fill_angle);
        infill_comp.generate(raft_polygons, raft_lines);
        gcode_layer.addLinesByOptimizer(raft_lines, &storage.raft_surface_config, SpaceFillType::Lines);

        last_position_planned = gcode_layer.getLastPosition();
        current_extruder_planned = gcode_layer.getExtruder();
        is_inside_mesh_layer_part = gcode_layer.getIsInsideMesh();
        
        gcode_layer.setFanSpeed(train->getSettingInPercentage("raft_surface_fan_speed"));
        gcode_layer.processFanSpeedAndMinimalLayerTime();
    }
}

void FffGcodeWriter::processLayer(SliceDataStorage& storage, unsigned int layer_nr, unsigned int total_layers, bool has_raft)
{
    Progress::messageProgress(Progress::Stage::EXPORT, layer_nr+1, total_layers);
    
    int layer_thickness = getSettingInMicrons("layer_height");
    if (layer_nr == 0)
    {
        layer_thickness = getSettingInMicrons("layer_height_0");
    }

    int max_inner_wall_width = 0;
    std::vector<bool> extruders_used = storage.getExtrudersUsed(layer_nr);
    for (int extr_nr = 0; extr_nr < storage.meshgroup->getExtruderCount(); extr_nr++)
    {
        if (extruders_used[extr_nr])
        {
            ExtruderTrain* extr = storage.meshgroup->getExtruderTrain(extr_nr);
            max_inner_wall_width = std::max(max_inner_wall_width, extr->getSettingInMicrons((extr->getSettingAsCount("wall_line_count") > 1) ? "wall_line_width_x" : "wall_line_width_0")); 
        }
    }
    ExtruderTrain* current_extruder_train = storage.meshgroup->getExtruderTrain(current_extruder_planned);
    
    int64_t comb_offset_from_outlines = current_extruder_train->getSettingInMicrons((current_extruder_train->getSettingAsCount("wall_line_count") > 1) ? "wall_line_width_x" : "wall_line_width_0") * 2; // TODO: only used when there is no second wall.
    int64_t z = storage.meshes[0].layers[layer_nr].printZ;
    GCodePlanner& gcode_layer = layer_plan_buffer.emplace_back(storage, layer_nr, z, layer_thickness, last_position_planned, current_extruder_planned, is_inside_mesh_layer_part, fan_speed_layer_time_settings, getSettingAsCombingMode("retraction_combing"), comb_offset_from_outlines, getSettingBoolean("travel_avoid_other_parts"), getSettingInMicrons("travel_avoid_distance"));
    
    if (layer_nr == 0)
    { // process the skirt of the starting extruder
        int start_extruder = 0; // TODO: make settable
        gcode_layer.setExtruder(start_extruder);
        processSkirt(storage, gcode_layer, start_extruder);
    }
    
    int extruder_nr_before = gcode_layer.getExtruder();
    addSupportToGCode(storage, gcode_layer, layer_nr, extruder_nr_before, true);

    processOozeShield(storage, gcode_layer, layer_nr);
    
    processDraftShield(storage, gcode_layer, layer_nr);

    //Figure out in which order to print the meshes, do this by looking at the current extruder and preferer the meshes that use that extruder.
    std::vector<unsigned int> mesh_order = calculateMeshOrder(storage, gcode_layer.getExtruder());
    for(unsigned int mesh_idx : mesh_order)
    {
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
    
    addSupportToGCode(storage, gcode_layer, layer_nr, extruder_nr_before, false);

    if (layer_nr == 0)
    { // add skirt for all extruders which haven't primed the skirt yet
        for (int extruder_nr = 0; extruder_nr < storage.meshgroup->getExtruderCount(); extruder_nr++)
        {
            if (!skirt_is_processed[extruder_nr])
            {
                setExtruder_addPrime(storage, gcode_layer, layer_nr, extruder_nr);
            }
        }
    }
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

void FffGcodeWriter::processSkirt(SliceDataStorage& storage, GCodePlanner& gcode_layer, unsigned int extruder_nr)
{
    if (skirt_is_processed[extruder_nr])
    {
        return;
    }
    Polygons& skirt = storage.skirt[extruder_nr];
    skirt_is_processed[extruder_nr] = true;
    if (skirt.size() == 0)
    {
        return;
    }
    gcode_layer.addTravel(skirt[skirt.size()-1].closestPointTo(gcode_layer.getLastPosition()));
    gcode_layer.addPolygonsByOptimizer(skirt, &storage.skirt_config[extruder_nr]);
    
}

void FffGcodeWriter::processOozeShield(SliceDataStorage& storage, GCodePlanner& gcode_layer, unsigned int layer_nr)
{
    if (storage.oozeShield.size() > 0)
    {
        gcode_layer.addPolygonsByOptimizer(storage.oozeShield[layer_nr], &storage.skirt_config[0]); // TODO: skirt config idx should correspond to ooze shield extruder nr
    }
}

void FffGcodeWriter::processDraftShield(SliceDataStorage& storage, GCodePlanner& gcode_layer, unsigned int layer_nr)
{
    if (storage.draft_protection_shield.size() == 0)
    {
        return;
    }

    int draft_shield_height = getSettingInMicrons("draft_shield_height");
    int layer_height_0 = getSettingInMicrons("layer_height_0");
    int layer_height = getSettingInMicrons("layer_height");

    int max_screen_layer = (draft_shield_height - layer_height_0) / layer_height + 1;

    if (int(layer_nr) > max_screen_layer)
    {
        return;
    }

    gcode_layer.addPolygonsByOptimizer(storage.draft_protection_shield, &storage.skirt_config[0]); // TODO: skirt config idx should correspond to draft shield extruder nr
}

std::vector<unsigned int> FffGcodeWriter::calculateMeshOrder(SliceDataStorage& storage, int current_extruder)
{
    std::vector<unsigned int> ret;
    std::list<unsigned int> add_list;
    for(unsigned int mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
        add_list.push_back(mesh_idx);

    int add_extruder_nr = current_extruder;
    while(add_list.size() > 0)
    {
        for(auto add_it = add_list.begin(); add_it != add_list.end(); )
        {
            if (storage.meshes[*add_it].getSettingAsIndex("extruder_nr") == add_extruder_nr)
            {
                ret.push_back(*add_it);
                add_it = add_list.erase(add_it);
            } 
            else 
            {
                ++add_it;
            }
        }
        if (add_list.size() > 0)
            add_extruder_nr = storage.meshes[*add_list.begin()].getSettingAsIndex("extruder_nr");
    }
    return ret;
}

void FffGcodeWriter::addMeshLayerToGCode_meshSurfaceMode(SliceDataStorage& storage, SliceMeshStorage* mesh, GCodePlanner& gcode_layer, int layer_nr)
{
    if (layer_nr > mesh->layer_nr_max_filled_layer)
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
    gcode_layer.addPolygonsByOptimizer(polygons, &mesh->inset0_config, nullptr, z_seam_type, mesh->getSettingBoolean("magic_spiralize"));

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
    PathOrderOptimizer part_order_optimizer(last_position_planned, z_seam_type);
    for(unsigned int partNr=0; partNr<layer->parts.size(); partNr++)
    {
        part_order_optimizer.addPolygon(layer->parts[partNr].insets[0][0]);
    }
    part_order_optimizer.optimize();

    bool skin_alternate_rotation = mesh->getSettingBoolean("skin_alternate_rotation") && ( mesh->getSettingAsCount("top_layers") >= 4 || mesh->getSettingAsCount("bottom_layers") >= 4 );
    
    for(int order_idx : part_order_optimizer.polyOrder)
    {
        SliceLayerPart& part = layer->parts[order_idx];

        EFillMethod infill_pattern = mesh->getSettingAsFillMethod("infill_pattern");
        int infill_angle = 45;
        if ((infill_pattern == EFillMethod::LINES || infill_pattern == EFillMethod::ZIG_ZAG))
        {
            unsigned int combined_infill_layers = std::max(1, mesh->getSettingInMicrons("infill_sparse_thickness") / std::max(getSettingInMicrons("layer_height"), 1));
            if ((layer_nr / combined_infill_layers) & 1)
            { // switch every [combined_infill_layers] layers
                infill_angle += 90;
            }
        }
        int infill_line_width =  mesh->infill_config[0].getLineWidth();
        
        int infill_line_distance = mesh->getSettingInMicrons("infill_line_distance");
        int infill_overlap = mesh->getSettingInMicrons("infill_overlap_mm");
        
        gcode_layer.setIsInside(true); // going to print inside stuff below
        
        if (mesh->getSettingBoolean("infill_before_walls"))
        {
            processMultiLayerInfill(gcode_layer, mesh, part, layer_nr, infill_line_distance, infill_overlap, infill_angle, infill_line_width);
            processSingleLayerInfill(gcode_layer, mesh, part, layer_nr, infill_line_distance, infill_overlap, infill_angle, infill_line_width);
        }
        
        processInsets(gcode_layer, mesh, part, layer_nr, z_seam_type);

        if (!mesh->getSettingBoolean("infill_before_walls"))
        {
            processMultiLayerInfill(gcode_layer, mesh, part, layer_nr, infill_line_distance, infill_overlap, infill_angle, infill_line_width);
            processSingleLayerInfill(gcode_layer, mesh, part, layer_nr, infill_line_distance, infill_overlap, infill_angle, infill_line_width);
        }

        EFillMethod skin_pattern = mesh->getSettingAsFillMethod("top_bottom_pattern");
        int skin_angle = 45;
        if ((skin_pattern == EFillMethod::LINES || skin_pattern == EFillMethod::ZIG_ZAG) && layer_nr & 1)
        {
            skin_angle += 90; // should coincide with infill_angle (if both skin and infill are lines) so that the first top layer is orthogonal to the last infill layer
        }
        if (skin_alternate_rotation && ( layer_nr / 2 ) & 1)
            skin_angle -= 45;
        
        int64_t skin_overlap =  mesh->getSettingInMicrons("skin_overlap_mm");
        processSkin(gcode_layer, mesh, part, layer_nr, skin_overlap, skin_angle, mesh->skin_config.getLineWidth());    
        
        //After a layer part, make sure the nozzle is inside the comb boundary, so we do not retract on the perimeter.
        if (!mesh->getSettingBoolean("magic_spiralize") || static_cast<int>(layer_nr) < mesh->getSettingAsCount("bottom_layers"))
        {
            gcode_layer.moveInsideCombBoundary(mesh->getSettingInMicrons((mesh->getSettingAsCount("wall_line_count") > 1) ? "wall_line_width_x" : "wall_line_width_0") * 1);
        }
        
        gcode_layer.setIsInside(false);
    }
    if (mesh->getSettingAsSurfaceMode("magic_mesh_surface_mode") != ESurfaceMode::NORMAL)
    {
        addMeshOpenPolyLinesToGCode(storage, mesh, gcode_layer, layer_nr);
    }
}


            


void FffGcodeWriter::processMultiLayerInfill(GCodePlanner& gcode_layer, SliceMeshStorage* mesh, SliceLayerPart& part, unsigned int layer_nr, int infill_line_distance, int infill_overlap, int infill_angle, int extrusion_width)
{
    if (infill_line_distance > 0)
    {
        //Print the thicker infill lines first. (double or more layer thickness, infill combined with previous layers)
        for(unsigned int n=1; n<part.infill_area_per_combine.size(); n++)
        {
            EFillMethod infill_pattern = mesh->getSettingAsFillMethod("infill_pattern");
            Infill infill_comp(infill_pattern, part.infill_area_per_combine[n], 0, extrusion_width, infill_line_distance, infill_overlap, infill_angle, false, false);
            Polygons infill_polygons;
            Polygons infill_lines;
            infill_comp.generate(infill_polygons, infill_lines);
            gcode_layer.addPolygonsByOptimizer(infill_polygons, &mesh->infill_config[n]);
            gcode_layer.addLinesByOptimizer(infill_lines, &mesh->infill_config[n], (infill_pattern == EFillMethod::ZIG_ZAG)? SpaceFillType::PolyLines : SpaceFillType::Lines);
        }
    }
}

void FffGcodeWriter::processSingleLayerInfill(GCodePlanner& gcode_layer, SliceMeshStorage* mesh, SliceLayerPart& part, unsigned int layer_nr, int infill_line_distance, int infill_overlap, int infill_angle, int extrusion_width)
{
    
    if (infill_line_distance == 0 || part.infill_area_per_combine.size() == 0)
    {
        return;
    }
        
    //Combine the 1 layer thick infill with the top/bottom skin and print that as one thing.
    Polygons infill_polygons;
    Polygons infill_lines;
    
    EFillMethod pattern = mesh->getSettingAsFillMethod("infill_pattern");
    Infill infill_comp(pattern, part.infill_area_per_combine[0], 0, extrusion_width, infill_line_distance, infill_overlap, infill_angle, false, false);
    infill_comp.generate(infill_polygons, infill_lines);
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

void FffGcodeWriter::processInsets(GCodePlanner& gcode_layer, SliceMeshStorage* mesh, SliceLayerPart& part, unsigned int layer_nr, EZSeamType z_seam_type)
{
    bool compensate_overlap_0 = mesh->getSettingBoolean("travel_compensate_overlapping_walls_0_enabled");
    bool compensate_overlap_x = mesh->getSettingBoolean("travel_compensate_overlapping_walls_x_enabled");
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
                gcode_layer.addPolygonsByOptimizer(part.insets[0], &mesh->insetX_config, nullptr, EZSeamType::SHORTEST, false);
            }
        }
        for(int inset_number=part.insets.size()-1; inset_number>-1; inset_number--)
        {
            if (inset_number == 0)
            {
                if (!compensate_overlap_0)
                {
                    gcode_layer.addPolygonsByOptimizer(part.insets[0], &mesh->inset0_config, nullptr, z_seam_type, spiralize);
                }
                else
                {
                    Polygons& outer_wall = part.insets[0];
                    WallOverlapComputation wall_overlap_computation(outer_wall, mesh->getSettingInMicrons("wall_line_width_0"));
                    gcode_layer.addPolygonsByOptimizer(outer_wall, &mesh->inset0_config, &wall_overlap_computation, z_seam_type, spiralize);
                }
            }
            else
            {
                if (!compensate_overlap_x)
                {
                    gcode_layer.addPolygonsByOptimizer(part.insets[inset_number], &mesh->insetX_config);
                }
                else
                {
                    Polygons& outer_wall = part.insets[inset_number];
                    WallOverlapComputation wall_overlap_computation(outer_wall, mesh->getSettingInMicrons("wall_line_width_x"));
                    gcode_layer.addPolygonsByOptimizer(outer_wall, &mesh->insetX_config, &wall_overlap_computation);
                }
            }
        }
    }
}


void FffGcodeWriter::processSkin(GCodePlanner& gcode_layer, SliceMeshStorage* mesh, SliceLayerPart& part, unsigned int layer_nr, int skin_overlap, int skin_angle, int extrusion_width)
{
    for(SkinPart& skin_part : part.skin_parts) // TODO: optimize parts order
    {
        Polygons skin_polygons;
        Polygons skin_lines;
        
        EFillMethod pattern = mesh->getSettingAsFillMethod("top_bottom_pattern");
        int bridge = -1;
        if (layer_nr > 0)
            bridge = bridgeAngle(skin_part.outline, &mesh->layers[layer_nr-1]);
        if (bridge > -1)
        {
            pattern = EFillMethod::LINES;
        } 
        Polygons* inner_skin_outline = nullptr;
        int offset_from_inner_skin_outline = 0;
        if (pattern != EFillMethod::CONCENTRIC)
        {
            for (Polygons& skin_perimeter : skin_part.insets)
            {
                gcode_layer.addPolygonsByOptimizer(skin_perimeter, &mesh->skin_config); // add polygons to gcode in inward order
            }
            if (skin_part.insets.size() > 0)
            {
                inner_skin_outline = &skin_part.insets.back();
                offset_from_inner_skin_outline = -extrusion_width/2;
            }
        }
        
        if (inner_skin_outline == nullptr)
        {
            inner_skin_outline = &skin_part.outline;
        }
        
        Infill infill_comp(pattern, *inner_skin_outline, offset_from_inner_skin_outline, extrusion_width, extrusion_width, skin_overlap, skin_angle, false, false);
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
    }
}

void FffGcodeWriter::addSupportToGCode(SliceDataStorage& storage, GCodePlanner& gcode_layer, int layer_nr, int extruder_nr_before, bool before_rest)
{
    if (!storage.support.generated || layer_nr > storage.support.layer_nr_max_filled_layer)
        return; 
    
    int support_roof_extruder_nr = getSettingAsIndex("support_roof_extruder_nr");
    int support_infill_extruder_nr = (layer_nr == 0)? getSettingAsIndex("support_extruder_nr_layer_0") : getSettingAsIndex("support_infill_extruder_nr");
    
    bool print_support_before_rest = support_infill_extruder_nr == extruder_nr_before
                                    || support_roof_extruder_nr == extruder_nr_before;
    // TODO: always print support after rest when only one nozzle is used for the whole meshgroup
    
    if (print_support_before_rest != before_rest)
        return;
    
    SupportLayer& support_layer = storage.support.supportLayers[layer_nr];
    if (support_layer.roofs.size() == 0 && support_layer.supportAreas.size() == 0)
    {
        return;
    }
    
    int current_extruder_nr = gcode_layer.getExtruder();
    
    if (support_layer.roofs.size() > 0)
    {
        if (support_roof_extruder_nr != support_infill_extruder_nr && support_roof_extruder_nr == current_extruder_nr)
        {
            addSupportRoofsToGCode(storage, gcode_layer, layer_nr);
            addSupportInfillToGCode(storage, gcode_layer, layer_nr);
        }
        else 
        {
            addSupportInfillToGCode(storage, gcode_layer, layer_nr);
            addSupportRoofsToGCode(storage, gcode_layer, layer_nr);
        }
    }
    else
    {
        addSupportInfillToGCode(storage, gcode_layer, layer_nr);
    }
}

void FffGcodeWriter::addSupportInfillToGCode(SliceDataStorage& storage, GCodePlanner& gcode_layer, int layer_nr)
{
    if (!storage.support.generated 
        || layer_nr > storage.support.layer_nr_max_filled_layer 
        || storage.support.supportLayers[layer_nr].supportAreas.size() == 0)
    {
        return;
    }

    int support_line_distance = getSettingInMicrons("support_line_distance");
    int extrusion_width = storage.support_config.getLineWidth();
    EFillMethod support_pattern = getSettingAsFillMethod("support_pattern");
    if (layer_nr == 0 && (support_pattern == EFillMethod::LINES || support_pattern == EFillMethod::ZIG_ZAG)) { support_pattern = EFillMethod::GRID; }

    int support_infill_extruder_nr = (layer_nr == 0)? getSettingAsIndex("support_extruder_nr_layer_0") : getSettingAsIndex("support_infill_extruder_nr");

    Polygons& support = storage.support.supportLayers[layer_nr].supportAreas;

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
            Polygons boundary = island.offset(-extrusion_width / 2);
            if (boundary.size() > 0)
            {
                setExtruder_addPrime(storage, gcode_layer, layer_nr, support_infill_extruder_nr); // only switch extruder if we're sure we're going to switch
                gcode_layer.addPolygonsByOptimizer(boundary, &storage.support_config);
            }
            offset_from_outline = -extrusion_width;
            support_infill_overlap = storage.meshgroup->getExtruderTrain(support_infill_extruder_nr)->getSettingInMicrons("infill_overlap_mm"); // support lines area should be expanded outward to overlap with the boundary polygon
        }
        Infill infill_comp(support_pattern, island, offset_from_outline, extrusion_width, support_line_distance, support_infill_overlap, 0, getSettingBoolean("support_connect_zigzags"), true);
        Polygons support_polygons;
        Polygons support_lines;
        infill_comp.generate(support_polygons, support_lines);
        if (support_lines.size() > 0 || support_polygons.size() > 0)
        {
            setExtruder_addPrime(storage, gcode_layer, layer_nr, support_infill_extruder_nr); // only switch extruder if we're sure we're going to switch
            gcode_layer.addPolygonsByOptimizer(support_polygons, &storage.support_config);
            gcode_layer.addLinesByOptimizer(support_lines, &storage.support_config, (support_pattern == EFillMethod::ZIG_ZAG)? SpaceFillType::PolyLines : SpaceFillType::Lines);
        }
    }
}

void FffGcodeWriter::addSupportRoofsToGCode(SliceDataStorage& storage, GCodePlanner& gcode_layer, int layer_nr)
{
    if (!storage.support.generated 
        || layer_nr > storage.support.layer_nr_max_filled_layer 
        || storage.support.supportLayers[layer_nr].roofs.size() == 0)
    {
        return;
    }
    
    EFillMethod pattern = getSettingAsFillMethod("support_roof_pattern");
    int support_line_distance = getSettingInMicrons("support_roof_line_distance");
    
    int roof_extruder_nr = getSettingAsIndex("support_roof_extruder_nr");
    setExtruder_addPrime(storage, gcode_layer, layer_nr, roof_extruder_nr);
    
    bool all_roofs_are_low = true;
    for (SliceMeshStorage& mesh : storage.meshes)
    {
        if (mesh.getSettingInMicrons("support_roof_height") >= 2 * getSettingInMicrons("layer_height"))
        {
            all_roofs_are_low = false;
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
        fillAngle = 45 + (layer_nr % 2) * 90; // alternate between the two kinds of diagonal:  / and \ .
    }
    int support_skin_overlap = 0; // the roofs should never be expanded outwards
    int outline_offset =  0; 
    
    Infill infill_comp(pattern, storage.support.supportLayers[layer_nr].roofs, outline_offset, storage.support_roof_config.getLineWidth(), support_line_distance, support_skin_overlap, fillAngle, false, true);
    Polygons support_polygons;
    Polygons support_lines;
    infill_comp.generate(support_polygons, support_lines);

    gcode_layer.addPolygonsByOptimizer(support_polygons, &storage.support_roof_config);
    gcode_layer.addLinesByOptimizer(support_lines, &storage.support_roof_config, (pattern == EFillMethod::ZIG_ZAG)? SpaceFillType::PolyLines : SpaceFillType::Lines);
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
        if (layer_nr == 0 && !skirt_is_processed[extruder_nr])
        {
            processSkirt(storage, gcode_layer, extruder_nr);
        }
        else
        {
            addPrimeTower(storage, gcode_layer, layer_nr, previous_extruder);
            
        }
    }
}

void FffGcodeWriter::addPrimeTower(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int layer_nr, int prev_extruder)
{
    
    if (!getSettingBoolean("prime_tower_enable"))
    {
        return;
    }
    
    bool prime_tower_dir_outward = getSettingBoolean("prime_tower_dir_outward");
    bool wipe = getSettingBoolean("prime_tower_wipe_enabled");
    
    storage.primeTower.addToGcode(storage, gcodeLayer, gcode, layer_nr, prev_extruder, prime_tower_dir_outward, wipe, last_prime_tower_poly_printed);
}

void FffGcodeWriter::finalize()
{
    if (CommandSocket::isInstantiated())
    {
        double print_time = gcode.getTotalPrintTime();
        std::vector<double> filament_used;
        for (int extr_nr = 0; extr_nr < getSettingAsCount("machine_extruder_count"); extr_nr++)
        {
            filament_used.emplace_back(gcode.getTotalFilamentUsed(extr_nr));
        }
        std::string prefix = gcode.getFileHeader(&print_time, filament_used);
        CommandSocket::getInstance()->sendGCodePrefix(prefix);
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

