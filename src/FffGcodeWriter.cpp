
#include <list>

#include "FffGcodeWriter.h"
#include "Progress.h"
#include "wallOverlap.h"

namespace cura
{


void FffGcodeWriter::writeGCode(SliceDataStorage& storage, TimeKeeper& time_keeper)
{
    PrimeTower primetower();
    
    gcode.preSetup(storage.meshgroup);
    
    gcode.resetTotalPrintTimeAndFilament();
    
    if (command_socket)
        command_socket->beginGCode();

    setConfigCoasting(storage);

    setConfigRetraction(storage);

    if (meshgroup_number == 1)
    {
        processStartingCode(storage);
    }
    else
    {
        processNextMeshGroupCode(storage);
    }
    meshgroup_number++;

    unsigned int total_layers = storage.meshes[0].layers.size();
    //gcode.writeComment("Layer count: %d", totalLayers);

    bool has_raft = getSettingAsPlatformAdhesion("adhesion_type") == Adhesion_Raft;
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
    
//     gcode.writeRetraction(&storage.retraction_config, true);
    gcode.writeRetraction_extruderSwitch();

    Progress::messageProgressStage(Progress::Stage::FINISH, &time_keeper, command_socket);
    
    gcode.writeFanCommand(0);

    //Store the object height for when we are printing multiple objects, as we need to clear every one of them when moving to the next position.
    max_object_height = std::max(max_object_height, storage.model_max.z);
    
    if (command_socket)
    {
        finalize();
        command_socket->sendGCodeLayer();
        command_socket->endSendSlicedObject();
        if (gcode.getFlavor() == GCODE_FLAVOR_ULTIGCODE)
        {
            std::ostringstream prefix;
            prefix << ";FLAVOR:UltiGCode\n";
            prefix << ";TIME:" << int(gcode.getTotalPrintTime()) << "\n";
            prefix << ";MATERIAL:" << int(gcode.getTotalFilamentUsed(0)) << "\n";
            prefix << ";MATERIAL2:" << int(gcode.getTotalFilamentUsed(1)) << "\n";
            command_socket->sendGCodePrefix(prefix.str());
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
        coasting_config.coasting_volume_move = train->getSettingInCubicMillimeters("coasting_volume_move"); 
        coasting_config.coasting_min_volume_move = train->getSettingInCubicMillimeters("coasting_min_volume_move"); 
        coasting_config.coasting_speed_move = train->getSettingInPercentage("coasting_speed_move"); 
        coasting_config.coasting_volume_retract = train->getSettingInCubicMillimeters("coasting_volume_retract");
        coasting_config.coasting_min_volume_retract = train->getSettingInCubicMillimeters("coasting_min_volume_retract");
        coasting_config.coasting_speed_retract = train->getSettingInPercentage("coasting_speed_retract");
    }
}

void FffGcodeWriter::setConfigRetraction(SliceDataStorage& storage) 
{
    storage.retraction_config.amount = INT2MM(getSettingInMicrons("retraction_amount"));
    storage.retraction_config.primeAmount = INT2MM(getSettingInMicrons("retraction_extra_prime_amount"));
    storage.retraction_config.speed = getSettingInMillimetersPerSecond("retraction_retract_speed");
    storage.retraction_config.primeSpeed = getSettingInMillimetersPerSecond("retraction_prime_speed");
    storage.retraction_config.zHop = getSettingInMicrons("retraction_hop");
    storage.retraction_config.retraction_min_travel_distance = getSettingInMicrons("retraction_min_travel");
    storage.retraction_config.retraction_extrusion_window = getSettingInMicrons("retraction_extrusion_window");
    storage.retraction_config.retraction_count_max = getSettingInMicrons("retraction_count_max");
    for (int extruder = 0; extruder < storage.meshgroup->getExtruderCount(); extruder++)
    {
        ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder);
        storage.retraction_config_per_extruder[extruder].amount = INT2MM(train->getSettingInMicrons("retraction_amount"));
        storage.retraction_config_per_extruder[extruder].primeAmount = INT2MM(train->getSettingInMicrons("retraction_extra_prime_amount"));
        storage.retraction_config_per_extruder[extruder].speed = train->getSettingInMillimetersPerSecond("retraction_retract_speed");
        storage.retraction_config_per_extruder[extruder].primeSpeed = train->getSettingInMillimetersPerSecond("retraction_prime_speed");
        storage.retraction_config_per_extruder[extruder].zHop = train->getSettingInMicrons("retraction_hop");
        storage.retraction_config_per_extruder[extruder].retraction_min_travel_distance = train->getSettingInMicrons("retraction_min_travel");
        storage.retraction_config_per_extruder[extruder].retraction_extrusion_window = train->getSettingInMicrons("retraction_extrusion_window");
        storage.retraction_config_per_extruder[extruder].retraction_count_max = train->getSettingInMicrons("retraction_count_max");
    }
    for(SliceMeshStorage& mesh : storage.meshes)
    {
        mesh.retraction_config.amount = INT2MM(mesh.getSettingInMicrons("retraction_amount"));
        mesh.retraction_config.primeAmount = INT2MM(mesh.getSettingInMicrons("retraction_extra_prime_amount"));
        mesh.retraction_config.speed = mesh.getSettingInMillimetersPerSecond("retraction_retract_speed");
        mesh.retraction_config.primeSpeed = mesh.getSettingInMillimetersPerSecond("retraction_prime_speed");
        mesh.retraction_config.zHop = mesh.getSettingInMicrons("retraction_hop");
        mesh.retraction_config.retraction_min_travel_distance = mesh.getSettingInMicrons("retraction_min_travel");
        mesh.retraction_config.retraction_extrusion_window = mesh.getSettingInMicrons("retraction_extrusion_window");
        mesh.retraction_config.retraction_count_max = mesh.getSettingInMicrons("retraction_count_max");
    }
}

void FffGcodeWriter::setConfigSkirt(SliceDataStorage& storage, int layer_thickness)
{
    for (int extruder = 0; extruder < storage.meshgroup->getExtruderCount(); extruder++)
    {
        SettingsBase* train = storage.meshgroup->getExtruderTrain(extruder);
        storage.skirt_config[extruder].setSpeed(train->getSettingInMillimetersPerSecond("skirt_speed"));
        storage.skirt_config[extruder].setLineWidth(train->getSettingInMicrons("skirt_line_width"));
        storage.skirt_config[extruder].setFlow(train->getSettingInPercentage("material_flow"));
        storage.skirt_config[extruder].setLayerHeight(layer_thickness);
    }
}

void FffGcodeWriter::setConfigSupport(SliceDataStorage& storage, int layer_thickness)
{
    storage.support_config.setLineWidth(getSettingInMicrons("support_line_width"));
    storage.support_config.setSpeed(getSettingInMillimetersPerSecond("speed_support_lines"));
    storage.support_config.setFlow(storage.meshgroup->getExtruderTrain(getSettingAsIndex("support_extruder_nr"))->getSettingInPercentage("material_flow"));
    storage.support_config.setLayerHeight(layer_thickness);
    
    storage.support_roof_config.setLineWidth(getSettingInMicrons("support_roof_line_width"));
    storage.support_roof_config.setSpeed(getSettingInMillimetersPerSecond("speed_support_roof"));
    storage.support_roof_config.setFlow(storage.meshgroup->getExtruderTrain(getSettingAsIndex("support_roof_extruder_nr"))->getSettingInPercentage("material_flow"));
    storage.support_roof_config.setLayerHeight(layer_thickness);
}

void FffGcodeWriter::setConfigInsets(SliceMeshStorage& mesh, int layer_thickness)
{
    mesh.inset0_config.setLineWidth(mesh.getSettingInMicrons("wall_line_width_0"));
    mesh.inset0_config.setSpeed(mesh.getSettingInMillimetersPerSecond("speed_wall_0"));
    mesh.inset0_config.setFlow(mesh.getSettingInPercentage("material_flow"));
    mesh.inset0_config.setLayerHeight(layer_thickness);

    mesh.insetX_config.setLineWidth(mesh.getSettingInMicrons("wall_line_width_x"));
    mesh.insetX_config.setSpeed(mesh.getSettingInMillimetersPerSecond("speed_wall_x"));
    mesh.insetX_config.setFlow(mesh.getSettingInPercentage("material_flow"));
    mesh.insetX_config.setLayerHeight(layer_thickness);
}

void FffGcodeWriter::setConfigSkin(SliceMeshStorage& mesh, int layer_thickness)
{
    mesh.skin_config.setLineWidth(mesh.getSettingInMicrons("skin_line_width"));
    mesh.skin_config.setSpeed(mesh.getSettingInMillimetersPerSecond("speed_topbottom"));
    mesh.skin_config.setFlow(mesh.getSettingInPercentage("material_flow"));
    mesh.skin_config.setLayerHeight(layer_thickness);
}

void FffGcodeWriter::setConfigInfill(SliceMeshStorage& mesh, int layer_thickness)
{
    for(unsigned int idx=0; idx<MAX_SPARSE_COMBINE; idx++)
    {
        mesh.infill_config[idx].setLineWidth(mesh.getSettingInMicrons("infill_line_width") * (idx + 1));
        mesh.infill_config[idx].setSpeed(mesh.getSettingInMillimetersPerSecond("speed_infill"));
        mesh.infill_config[idx].setFlow(mesh.getSettingInPercentage("material_flow"));
        mesh.infill_config[idx].setLayerHeight(layer_thickness);
    }
}

void FffGcodeWriter::processStartingCode(SliceDataStorage& storage)
{
    if (gcode.getFlavor() == GCODE_FLAVOR_ULTIGCODE)
    {
        if (!command_socket)
        {
            gcode.writeCode(";FLAVOR:UltiGCode\n;TIME:666\n;MATERIAL:666\n;MATERIAL2:-1\n");
        }
    }
    else 
    {
        if (getSettingBoolean("machine_heated_bed") && getSettingInDegreeCelsius("material_bed_temperature") > 0)
            gcode.writeBedTemperatureCommand(getSettingInDegreeCelsius("material_bed_temperature"), true);
        
        for(SliceMeshStorage& mesh : storage.meshes)
            if (mesh.getSettingInDegreeCelsius("material_print_temperature") > 0)
                gcode.writeTemperatureCommand(mesh.getSettingAsIndex("extruder_nr"), mesh.getSettingInDegreeCelsius("material_print_temperature"));
        for(SliceMeshStorage& mesh : storage.meshes)
            if (mesh.getSettingInDegreeCelsius("material_print_temperature") > 0)
                gcode.writeTemperatureCommand(mesh.getSettingAsIndex("extruder_nr"), mesh.getSettingInDegreeCelsius("material_print_temperature"), true);
    }
    
    gcode.writeCode(getSettingString("machine_start_gcode").c_str());

    gcode.writeComment("Generated with Cura_SteamEngine " VERSION);
    if (gcode.getFlavor() == GCODE_FLAVOR_BFB)
    {
        gcode.writeComment("enable auto-retraction");
        std::ostringstream tmp;
        tmp << "M227 S" << (getSettingInMicrons("retraction_amount") * 2560 / 1000) << " P" << (getSettingInMicrons("retraction_amount") * 2560 / 1000);
        gcode.writeLine(tmp.str().c_str());
    }
}

void FffGcodeWriter::processNextMeshGroupCode(SliceDataStorage& storage)
{
    gcode.writeFanCommand(0);
    gcode.resetExtrusionValue();
    gcode.setZ(max_object_height + 5000);
    gcode.writeMove(gcode.getPositionXY(), getSettingInMillimetersPerSecond("speed_travel"), 0);
    gcode.writeMove(Point(storage.model_min.x, storage.model_min.y), getSettingInMillimetersPerSecond("speed_travel"), 0);
}
    
void FffGcodeWriter::processRaft(SliceDataStorage& storage, unsigned int totalLayers)
{
    int extruder_nr = getSettingAsIndex("adhesion_extruder_nr");
    ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder_nr);
    
    GCodePathConfig raft_base_config(&storage.retraction_config_per_extruder[extruder_nr], "SUPPORT");
    raft_base_config.setSpeed(getSettingInMillimetersPerSecond("raft_base_speed"));
    raft_base_config.setLineWidth(getSettingInMicrons("raft_base_line_width"));
    raft_base_config.setLayerHeight(getSettingInMicrons("raft_base_thickness"));
    raft_base_config.setFlow(train->getSettingInPercentage("material_flow"));
    GCodePathConfig raft_interface_config(&storage.retraction_config_per_extruder[extruder_nr], "SUPPORT");
    raft_interface_config.setSpeed(getSettingInMillimetersPerSecond("raft_interface_speed"));
    raft_interface_config.setLineWidth(getSettingInMicrons("raft_interface_line_width"));
    raft_interface_config.setLayerHeight(getSettingInMicrons("raft_base_thickness"));
    raft_interface_config.setFlow(train->getSettingInPercentage("material_flow"));
    GCodePathConfig raft_surface_config(&storage.retraction_config_per_extruder[extruder_nr], "SUPPORT");
    raft_surface_config.setSpeed(getSettingInMillimetersPerSecond("raft_surface_speed"));
    raft_surface_config.setLineWidth(getSettingInMicrons("raft_surface_line_width"));
    raft_surface_config.setLayerHeight(getSettingInMicrons("raft_base_thickness"));
    raft_surface_config.setFlow(train->getSettingInPercentage("material_flow"));

    bool retraction_combing = false; // the raft isn't added to the parts to avoid
    
    { // raft base layer
        gcode.writeLayerComment(-3);
        gcode.writeComment("RAFT");
        GCodePlanner gcode_layer(gcode, storage, &storage.retraction_config_per_extruder[extruder_nr], train->getSettingInMillimetersPerSecond("speed_travel"), retraction_combing, 0, train->getSettingInMicrons("machine_nozzle_size"), train->getSettingBoolean("travel_avoid_other_parts"), train->getSettingInMicrons("travel_avoid_distance"));
        gcode_layer.setCombing(false);
        if (getSettingAsIndex("adhesion_extruder_nr") > 0)
            gcode_layer.setExtruder(extruder_nr);
        gcode.setZ(getSettingInMicrons("raft_base_thickness"));
        gcode_layer.addPolygonsByOptimizer(storage.raftOutline, &raft_base_config);

        Polygons raftLines;
        int offset_from_poly_outline = 0;
        generateLineInfill(storage.raftOutline, offset_from_poly_outline, raftLines, train->getSettingInMicrons("raft_base_line_width"), train->getSettingInMicrons("raft_base_line_spacing"), train->getSettingInPercentage("infill_overlap"), 0);
        gcode_layer.addLinesByOptimizer(raftLines, &raft_base_config);

        gcode.writeFanCommand(train->getSettingInPercentage("raft_base_fan_speed"));
        gcode_layer.writeGCode(false, train->getSettingInMicrons("raft_base_thickness"));
    }

    { // raft interface layer
        gcode.writeLayerComment(-2);
        gcode.writeComment("RAFT");
        GCodePlanner gcode_layer(gcode, storage, &storage.retraction_config_per_extruder[extruder_nr], train->getSettingInMillimetersPerSecond("speed_travel"), retraction_combing, 0, train->getSettingInMicrons("machine_nozzle_size"), train->getSettingBoolean("travel_avoid_other_parts"), train->getSettingInMicrons("travel_avoid_distance"));
        gcode_layer.setCombing(false);
        gcode.setZ(train->getSettingInMicrons("raft_base_thickness") + train->getSettingInMicrons("raft_interface_thickness"));

        Polygons raftLines;
        int offset_from_poly_outline = 0;
        generateLineInfill(storage.raftOutline, offset_from_poly_outline, raftLines, train->getSettingInMicrons("raft_interface_line_width"), train->getSettingInMicrons("raft_interface_line_spacing"), train->getSettingInPercentage("infill_overlap"), train->getSettingAsCount("raft_surface_layers") > 0 ? 45 : 90);
        gcode_layer.addLinesByOptimizer(raftLines, &raft_interface_config);

        gcode_layer.writeGCode(false, train->getSettingInMicrons("raft_interface_thickness"));
    }

    for (int raftSurfaceLayer=1; raftSurfaceLayer<=train->getSettingAsCount("raft_surface_layers"); raftSurfaceLayer++)
    { // raft surface layers
        gcode.writeLayerComment(-1);
        gcode.writeComment("RAFT");
        GCodePlanner gcode_layer(gcode, storage, &storage.retraction_config_per_extruder[extruder_nr], train->getSettingInMillimetersPerSecond("speed_travel"), retraction_combing, 0, train->getSettingInMicrons("machine_nozzle_size"), train->getSettingBoolean("travel_avoid_other_parts"), train->getSettingInMicrons("travel_avoid_distance"));
        gcode_layer.setCombing(false);
        gcode.setZ(train->getSettingInMicrons("raft_base_thickness") + train->getSettingInMicrons("raft_interface_thickness") + train->getSettingInMicrons("raft_surface_thickness")*raftSurfaceLayer);

        Polygons raft_lines;
        int offset_from_poly_outline = 0;
        generateLineInfill(storage.raftOutline, offset_from_poly_outline, raft_lines, train->getSettingInMicrons("raft_surface_line_width"), train->getSettingInMicrons("raft_surface_line_spacing"), train->getSettingInPercentage("infill_overlap"), 90 * raftSurfaceLayer);
        gcode_layer.addLinesByOptimizer(raft_lines, &raft_surface_config);

        gcode_layer.writeGCode(false, train->getSettingInMicrons("raft_interface_thickness"));
    }
}

void FffGcodeWriter::processLayer(SliceDataStorage& storage, unsigned int layer_nr, unsigned int total_layers, bool has_raft)
{
    Progress::messageProgress(Progress::Stage::EXPORT, layer_nr+1, total_layers, command_socket);

    int layer_thickness = getSettingInMicrons("layer_height");
    if (layer_nr == 0 && !has_raft)
    {
        layer_thickness = getSettingInMicrons("layer_height_0");
    }
    
    

    setConfigSkirt(storage, layer_thickness);

    setConfigSupport(storage, layer_thickness);
    
    storage.primeTower.setConfigs(storage.meshgroup, storage.retraction_config_per_extruder, layer_thickness);
    
    for(SliceMeshStorage& mesh : storage.meshes)
    {
        setConfigInsets(mesh, layer_thickness);
        setConfigSkin(mesh, layer_thickness);
        setConfigInfill(mesh, layer_thickness);
    }

    processInitialLayersSpeedup(storage, layer_nr);

    gcode.writeLayerComment(layer_nr);

    int64_t comb_offset_from_outlines = storage.meshgroup->getExtruderTrain(gcode.getExtruderNr())->getSettingInMicrons("machine_nozzle_size") * 2; // TODO: only used when there is no second wall.
    GCodePlanner gcode_layer(gcode, storage, &storage.retraction_config, getSettingInMillimetersPerSecond("speed_travel"), getSettingBoolean("retraction_combing"), layer_nr, comb_offset_from_outlines, getSettingBoolean("travel_avoid_other_parts"), getSettingInMicrons("travel_avoid_distance"));

    int z = storage.meshes[0].layers[layer_nr].printZ;         
    gcode.setZ(z);
    gcode.resetStartPosition();
    
    if (layer_nr == 0)
    {
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
    gcode_layer.setCombing(true);
    for(unsigned int mesh_idx : mesh_order)
    {
        SliceMeshStorage* mesh = &storage.meshes[mesh_idx];
        if (mesh->getSettingBoolean("magic_mesh_surface_mode"))
        {
            addMeshLayerToGCode_magicPolygonMode(storage, mesh, gcode_layer, layer_nr);
        }
        else
        {
            addMeshLayerToGCode(storage, mesh, gcode_layer, layer_nr);
        }
    }
    gcode_layer.setCombing(false);
    
    addSupportToGCode(storage, gcode_layer, layer_nr, extruder_nr_before, false);

    { // add prime tower if it hasn't already been added
        // print the prime tower if it hasn't been printed yet
        int prev_extruder = gcode_layer.getExtruder(); // most likely the same extruder as we are extruding with now
        addPrimeTower(storage, gcode_layer, layer_nr, prev_extruder);
    }
    processFanSpeedAndMinimalLayerTime(storage, gcode_layer, layer_nr);
    
    gcode_layer.writeGCode(getSettingBoolean("cool_lift_head"), layer_nr > 0 ? getSettingInMicrons("layer_height") : getSettingInMicrons("layer_height_0"));
    if (command_socket)
        command_socket->sendGCodeLayer();
}

void FffGcodeWriter::processInitialLayersSpeedup(SliceDataStorage& storage, unsigned int layer_nr)
{
    double initial_speedup_layers = getSettingAsCount("speed_slowdown_layers");
    if (static_cast<int>(layer_nr) < initial_speedup_layers)
    {
        double initial_layer_speed = getSettingInMillimetersPerSecond("speed_layer_0");
        storage.support_config.smoothSpeed(initial_layer_speed, layer_nr, initial_speedup_layers);
        for(SliceMeshStorage& mesh : storage.meshes)
        {
            initial_layer_speed = mesh.getSettingInMillimetersPerSecond("speed_layer_0");
            mesh.inset0_config.smoothSpeed(initial_layer_speed, layer_nr, initial_speedup_layers);
            mesh.insetX_config.smoothSpeed(initial_layer_speed, layer_nr, initial_speedup_layers);
            mesh.skin_config.smoothSpeed(initial_layer_speed, layer_nr, initial_speedup_layers);
            for(unsigned int idx=0; idx<MAX_SPARSE_COMBINE; idx++)
            {
                mesh.infill_config[idx].smoothSpeed(initial_layer_speed, layer_nr, initial_speedup_layers);
            }
        }
    }
}

void FffGcodeWriter::processSkirt(SliceDataStorage& storage, GCodePlanner& gcode_layer, unsigned int extruder_nr)
{
    gcode_layer.setCombing(false);
    Polygons& skirt = storage.skirt[extruder_nr];
    if (skirt.size() > 0)
    {
        gcode_layer.addTravel(skirt[skirt.size()-1].closestPointTo(gcode.getPositionXY()));
    }
    gcode_layer.addPolygonsByOptimizer(skirt, &storage.skirt_config[extruder_nr]);
    
}

void FffGcodeWriter::processOozeShield(SliceDataStorage& storage, GCodePlanner& gcode_layer, unsigned int layer_nr)
{
    if (storage.oozeShield.size() > 0)
    {
        gcode_layer.setCombing(false);
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
    
    gcode_layer.setCombing(false);
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

void FffGcodeWriter::addMeshLayerToGCode_magicPolygonMode(SliceDataStorage& storage, SliceMeshStorage* mesh, GCodePlanner& gcode_layer, int layer_nr)
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
        for(unsigned int n=0; n<layer->parts[partNr].outline.size(); n++)
        {
            for(unsigned int m=1; m<layer->parts[partNr].outline[n].size(); m++)
            {
                Polygon p;
                p.add(layer->parts[partNr].outline[n][m-1]);
                p.add(layer->parts[partNr].outline[n][m]);
                polygons.add(p);
            }
            if (layer->parts[partNr].outline[n].size() > 0)
            {
                Polygon p;
                p.add(layer->parts[partNr].outline[n][layer->parts[partNr].outline[n].size()-1]);
                p.add(layer->parts[partNr].outline[n][0]);
                polygons.add(p);
            }
        }
    }
    for(unsigned int n=0; n<layer->openLines.size(); n++)
    {
        for(unsigned int m=1; m<layer->openLines[n].size(); m++)
        {
            Polygon p;
            p.add(layer->openLines[n][m-1]);
            p.add(layer->openLines[n][m]);
            polygons.add(p);
        }
    }
    if (mesh->getSettingBoolean("magic_spiralize"))
        mesh->inset0_config.spiralize = true;

    gcode_layer.addPolygonsByOptimizer(polygons, &mesh->inset0_config);
    
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
    
    setExtruder_addPrime(storage, gcode_layer, layer_nr, mesh->getSettingAsIndex("extruder_nr"));

    
    EZSeamType z_seam_type = mesh->getSettingAsZSeamType("z_seam_type");
    PathOrderOptimizer part_order_optimizer(gcode.getStartPositionXY(), z_seam_type);
    for(unsigned int partNr=0; partNr<layer->parts.size(); partNr++)
    {
        part_order_optimizer.addPolygon(layer->parts[partNr].insets[0][0]);
    }
    part_order_optimizer.optimize();

    bool skin_alternate_rotation = mesh->getSettingBoolean("skin_alternate_rotation") && ( mesh->getSettingAsCount("top_layers") >= 4 || mesh->getSettingAsCount("bottom_layers") >= 4 );
    
    for(int order_idx : part_order_optimizer.polyOrder)
    {
        SliceLayerPart& part = layer->parts[order_idx];

        int infill_angle = 45;
        if (layer_nr & 1)
            infill_angle += 90;
        int extrusion_width =  mesh->infill_config[0].getLineWidth();
        
        int sparse_infill_line_distance = mesh->getSettingInMicrons("infill_line_distance");
        double infill_overlap = mesh->getSettingInPercentage("infill_overlap");
        
        processMultiLayerInfill(gcode_layer, mesh, part, sparse_infill_line_distance, infill_overlap, infill_angle, extrusion_width);
        processSingleLayerInfill(gcode_layer, mesh, part, sparse_infill_line_distance, infill_overlap, infill_angle, extrusion_width);

        processInsets(gcode_layer, mesh, part, layer_nr, z_seam_type);

        if (skin_alternate_rotation && ( layer_nr / 2 ) & 1)
            infill_angle -= 45;
        
        int64_t skin_overlap = 0;
        processSkin(gcode_layer, mesh, part, layer_nr, skin_overlap, infill_angle, extrusion_width);    
        
        //After a layer part, make sure the nozzle is inside the comb boundary, so we do not retract on the perimeter.
        if (!mesh->getSettingBoolean("magic_spiralize") || static_cast<int>(layer_nr) < mesh->getSettingAsCount("bottom_layers"))
            gcode_layer.moveInsideCombBoundary(mesh->getSettingInMicrons("machine_nozzle_size") * 1);
    }
}


            


void FffGcodeWriter::processMultiLayerInfill(GCodePlanner& gcode_layer, SliceMeshStorage* mesh, SliceLayerPart& part, int sparse_infill_line_distance, double infill_overlap, int infill_angle, int extrusion_width)
{
    if (sparse_infill_line_distance > 0)
    {
        //Print the thicker sparse lines first. (double or more layer thickness, infill combined with previous layers)
        for(unsigned int n=1; n<part.sparse_outline.size(); n++)
        {
            Polygons infill_polygons;
            switch(mesh->getSettingAsFillMethod("infill_pattern"))
            {
            case Fill_Grid:
                generateGridInfill(part.sparse_outline[n], 0, infill_polygons, extrusion_width, sparse_infill_line_distance * 2, infill_overlap, infill_angle);
                gcode_layer.addLinesByOptimizer(infill_polygons, &mesh->infill_config[n]);
                break;
            case Fill_Lines:
                generateLineInfill(part.sparse_outline[n], 0, infill_polygons, extrusion_width, sparse_infill_line_distance, infill_overlap, infill_angle);
                gcode_layer.addLinesByOptimizer(infill_polygons, &mesh->infill_config[n]);
                break;
            case Fill_Triangles:
                generateTriangleInfill(part.sparse_outline[n], 0, infill_polygons, extrusion_width, sparse_infill_line_distance * 3, infill_overlap, 0);
                gcode_layer.addLinesByOptimizer(infill_polygons, &mesh->infill_config[n]);
                break;
            case Fill_Concentric:
                generateConcentricInfill(part.sparse_outline[n], infill_polygons, sparse_infill_line_distance);
                gcode_layer.addPolygonsByOptimizer(infill_polygons, &mesh->infill_config[n]);
                break;
            case Fill_ZigZag:
                generateZigZagInfill(part.sparse_outline[n], infill_polygons, extrusion_width, sparse_infill_line_distance, infill_overlap, infill_angle, false, false);
                gcode_layer.addPolygonsByOptimizer(infill_polygons, &mesh->infill_config[n]);
                break;
            default:
                logError("infill_pattern has unknown value.\n");
                break;
            }
        }
    }
}

void FffGcodeWriter::processSingleLayerInfill(GCodePlanner& gcode_layer, SliceMeshStorage* mesh, SliceLayerPart& part, int sparse_infill_line_distance, double infill_overlap, int infill_angle, int extrusion_width)
{
    //Combine the 1 layer thick infill with the top/bottom skin and print that as one thing.
    Polygons infill_polygons;
    Polygons infill_lines;
    EFillMethod pattern = mesh->getSettingAsFillMethod("infill_pattern");
    if (sparse_infill_line_distance > 0 && part.sparse_outline.size() > 0)
    {
        switch(pattern)
        {
        case Fill_Grid:
            generateGridInfill(part.sparse_outline[0], 0, infill_lines, extrusion_width, sparse_infill_line_distance * 2, infill_overlap, infill_angle);
            break;
        case Fill_Lines:
            generateLineInfill(part.sparse_outline[0], 0, infill_lines, extrusion_width, sparse_infill_line_distance, infill_overlap, infill_angle);
            break;
        case Fill_Triangles:
            generateTriangleInfill(part.sparse_outline[0], 0, infill_lines, extrusion_width, sparse_infill_line_distance * 3, infill_overlap, 0);
            break;
        case Fill_Concentric:
            generateConcentricInfill(part.sparse_outline[0], infill_polygons, sparse_infill_line_distance);
            break;
        case Fill_ZigZag:
            generateZigZagInfill(part.sparse_outline[0], infill_lines, extrusion_width, sparse_infill_line_distance, infill_overlap, infill_angle, false, false);
            break;
        default:
            logError("infill_pattern has unknown value.\n");
            break;
        }
    }
    gcode_layer.addPolygonsByOptimizer(infill_polygons, &mesh->infill_config[0]);
    if (pattern == Fill_Grid || pattern == Fill_Lines || pattern == Fill_Triangles)
    {
        gcode_layer.addLinesByOptimizer(infill_lines, &mesh->infill_config[0], mesh->getSettingInMicrons("infill_wipe_dist")); 
    }
    else 
    {
        gcode_layer.addLinesByOptimizer(infill_lines, &mesh->infill_config[0]); 
    }
}

void FffGcodeWriter::processInsets(GCodePlanner& gcode_layer, SliceMeshStorage* mesh, SliceLayerPart& part, unsigned int layer_nr, EZSeamType z_seam_type)
{
    bool compensate_overlap = mesh->getSettingBoolean("travel_compensate_overlapping_walls_enabled");
    if (mesh->getSettingAsCount("wall_line_count") > 0)
    {
        if (mesh->getSettingBoolean("magic_spiralize"))
        {
            if (static_cast<int>(layer_nr) >= mesh->getSettingAsCount("bottom_layers"))
                mesh->inset0_config.spiralize = true;
            if (static_cast<int>(layer_nr) == mesh->getSettingAsCount("bottom_layers") && part.insets.size() > 0)
                gcode_layer.addPolygonsByOptimizer(part.insets[0], &mesh->insetX_config);
        }
        for(int inset_number=part.insets.size()-1; inset_number>-1; inset_number--)
        {
            if (inset_number == 0)
            {
                if (!compensate_overlap)
                {
                    gcode_layer.addPolygonsByOptimizer(part.insets[0], &mesh->inset0_config, nullptr, z_seam_type);
                }
                else
                {
                    Polygons& outer_wall = part.insets[0];
                    WallOverlapComputation wall_overlap_computation(outer_wall, mesh->getSettingInMicrons("wall_line_width_0"));
                    gcode_layer.addPolygonsByOptimizer(outer_wall, &mesh->inset0_config, &wall_overlap_computation, z_seam_type);
                }
            }
            else
            {
                gcode_layer.addPolygonsByOptimizer(part.insets[inset_number], &mesh->insetX_config);
            }
        }
    }
}


void FffGcodeWriter::processSkin(GCodePlanner& gcode_layer, SliceMeshStorage* mesh, SliceLayerPart& part, unsigned int layer_nr, double infill_overlap, int infill_angle, int extrusion_width)
{
    Polygons skin_polygons;
    Polygons skin_lines;
    for(SkinPart& skin_part : part.skin_parts) // TODO: optimize parts order
    {
        int bridge = -1;
        if (layer_nr > 0)
            bridge = bridgeAngle(skin_part.outline, &mesh->layers[layer_nr-1]);
        if (bridge > -1)
        {
            generateLineInfill(skin_part.outline, 0, skin_lines, extrusion_width, extrusion_width, infill_overlap, bridge);
        }else{
            switch(mesh->getSettingAsFillMethod("top_bottom_pattern"))
            {
            case Fill_Lines:
                for (Polygons& skin_perimeter : skin_part.insets)
                {
                    gcode_layer.addPolygonsByOptimizer(skin_perimeter, &mesh->skin_config); // add polygons to gcode in inward order
                }
                if (skin_part.insets.size() > 0)
                {
                    generateLineInfill(skin_part.insets.back(), -extrusion_width/2, skin_lines, extrusion_width, extrusion_width, infill_overlap, infill_angle);
                    if (mesh->getSettingString("fill_perimeter_gaps") != "Nowhere")
                    {
                        generateLineInfill(skin_part.perimeterGaps, 0, skin_lines, extrusion_width, extrusion_width, 0, infill_angle);
                    }
                } 
                else
                {
                    generateLineInfill(skin_part.outline, 0, skin_lines, extrusion_width, extrusion_width, infill_overlap, infill_angle);
                }
                break;
            case Fill_Concentric:
                {
                    Polygons in_outline;
                    PolygonUtils::offsetSafe(skin_part.outline, -extrusion_width/2, extrusion_width, in_outline, mesh->getSettingBoolean("remove_overlapping_walls_x_enabled"));
                    if (mesh->getSettingString("fill_perimeter_gaps") != "Nowhere")
                    {
                        generateConcentricInfillDense(in_outline, skin_polygons, &part.perimeterGaps, extrusion_width, mesh->getSettingBoolean("remove_overlapping_walls_x_enabled"));
                    }
                }
                break;
            default:
                logError("Unknown fill method for skin\n");
                break;
            }
        }
    }
    
    // handle gaps between perimeters etc.
    if (mesh->getSettingString("fill_perimeter_gaps") != "Nowhere")
    {
        generateLineInfill(part.perimeterGaps, 0, skin_lines, extrusion_width, extrusion_width, 0, infill_angle);
    }
    
    
    gcode_layer.addPolygonsByOptimizer(skin_polygons, &mesh->skin_config);
    gcode_layer.addLinesByOptimizer(skin_lines, &mesh->skin_config);
}

void FffGcodeWriter::addSupportToGCode(SliceDataStorage& storage, GCodePlanner& gcode_layer, int layer_nr, int extruder_nr_before, bool before_rest)
{
    if (!storage.support.generated || layer_nr > storage.support.layer_nr_max_filled_layer)
        return;
    
    int support_roof_extruder_nr = getSettingAsIndex("support_roof_extruder_nr");
    int support_extruder_nr = (layer_nr == 0)? getSettingAsIndex("support_extruder_nr_layer_0") : getSettingAsIndex("support_extruder_nr");
    
    bool print_support_before_rest = support_extruder_nr == extruder_nr_before
                                    || support_roof_extruder_nr == extruder_nr_before;
    // TODO: always print support after rest when only one nozzle is used for the whole meshgroup
    
    if (print_support_before_rest != before_rest)
        return;
    
    gcode_layer.setCombing(false);
    
    int current_extruder_nr = gcode_layer.getExtruder();
    
    if (getSettingBoolean("support_roof_enable"))
    {   
        if (support_roof_extruder_nr != support_extruder_nr && support_roof_extruder_nr == current_extruder_nr)
        {
            addSupportRoofsToGCode(storage, gcode_layer, layer_nr);
            addSupportLinesToGCode(storage, gcode_layer, layer_nr);
        }
        else 
        {
            addSupportLinesToGCode(storage, gcode_layer, layer_nr);
            addSupportRoofsToGCode(storage, gcode_layer, layer_nr);
        }
    }
    else
    {
        addSupportLinesToGCode(storage, gcode_layer, layer_nr);
    }
}

void FffGcodeWriter::addSupportLinesToGCode(SliceDataStorage& storage, GCodePlanner& gcode_layer, int layer_nr)
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
    
    int support_extruder_nr = (layer_nr == 0)? getSettingAsIndex("support_extruder_nr_layer_0") : getSettingAsIndex("support_extruder_nr");
    
    double infill_overlap = storage.meshgroup->getExtruderTrain(support_extruder_nr)->getSettingInPercentage("infill_overlap");
    
    setExtruder_addPrime(storage, gcode_layer, layer_nr, support_extruder_nr);
    
    Polygons& support = storage.support.supportLayers[layer_nr].supportAreas;
    
    std::vector<PolygonsPart> support_islands = support.splitIntoParts();

    PathOrderOptimizer island_order_optimizer(gcode.getPositionXY());
    for(unsigned int n=0; n<support_islands.size(); n++)
    {
        island_order_optimizer.addPolygon(support_islands[n][0]);
    }
    island_order_optimizer.optimize();

    for(unsigned int n=0; n<support_islands.size(); n++)
    {
        PolygonsPart& island = support_islands[island_order_optimizer.polyOrder[n]];

        Polygons support_lines;
        if (support_line_distance > 0)
        {
            switch(support_pattern)
            {
            case Fill_Grid:
                {
                    int offset_from_outline = 0;
                    if (support_line_distance > extrusion_width * 4)
                    {
                        generateGridInfill(island, offset_from_outline, support_lines, extrusion_width, support_line_distance*2, infill_overlap, 0);
                    }else{
                        generateLineInfill(island, offset_from_outline, support_lines, extrusion_width, support_line_distance, infill_overlap, (layer_nr & 1) ? 0 : 90);
                    }
                }
                break;
            case Fill_Lines:
                {
                    int offset_from_outline = 0;
                    if (layer_nr == 0)
                    {
                        generateGridInfill(island, offset_from_outline, support_lines, extrusion_width, support_line_distance, 0 + 150, 0);
                    }else{
                        generateLineInfill(island, offset_from_outline, support_lines, extrusion_width, support_line_distance, 0, 0);
                    }
                }
                break;
            case Fill_ZigZag:
                {
                    int offset_from_outline = 0;
                    if (layer_nr == 0)
                    {
                        generateGridInfill(island, offset_from_outline, support_lines, extrusion_width, support_line_distance, 0 + 150, 0);
                    }else{
                        generateZigZagInfill(island, support_lines, extrusion_width, support_line_distance, 0, 0, getSettingBoolean("support_connect_zigzags"), true);
                    }
                }
                break;
            default:
                logError("Unknown fill method for support\n");
                break;
            }
        }

        if (support_pattern == Fill_Grid || ( support_pattern == Fill_ZigZag && layer_nr == 0 ) )
            gcode_layer.addPolygonsByOptimizer(island, &storage.support_config);
        gcode_layer.addLinesByOptimizer(support_lines, &storage.support_config);
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
    
    int roof_extruder_nr = getSettingAsIndex("support_roof_extruder_nr");
    setExtruder_addPrime(storage, gcode_layer, layer_nr, roof_extruder_nr);
    
    double fillAngle;
    if (getSettingInMicrons("support_roof_height") < 2 * getSettingInMicrons("layer_height"))
    {
        fillAngle = 90; // perpendicular to support lines
    }
    else 
    {
        fillAngle = 45 + (layer_nr % 2) * 90; // alternate between the two kinds of diagonal:  / and \ .
    }
    double infill_overlap = 0;
    int outline_offset =  0; // - roofConfig.getLineWidth() / 2;
    
    Polygons skinLines;
    generateLineInfill(storage.support.supportLayers[layer_nr].roofs, outline_offset, skinLines, storage.support_roof_config.getLineWidth(), storage.support_roof_config.getLineWidth(), infill_overlap, fillAngle);
    gcode_layer.addLinesByOptimizer(skinLines, &storage.support_roof_config);
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
        if (layer_nr == 0)
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
    
    if (getSettingInMicrons("prime_tower_size") < 1)
    {
        return;
    }
    
    bool prime_tower_dir_outward = getSettingBoolean("prime_tower_dir_outward");
    bool wipe = getSettingBoolean("prime_tower_wipe_enabled");
    
    storage.primeTower.addToGcode(storage, gcodeLayer, gcode, layer_nr, prev_extruder, prime_tower_dir_outward, wipe, last_prime_tower_poly_printed);
}

void FffGcodeWriter::processFanSpeedAndMinimalLayerTime(SliceDataStorage& storage, GCodePlanner& gcodeLayer, unsigned int layer_nr)
{ 
    double travelTime;
    double extrudeTime;
    gcodeLayer.getTimes(travelTime, extrudeTime);
    gcodeLayer.forceMinimalLayerTime(getSettingInSeconds("cool_min_layer_time"), getSettingInMillimetersPerSecond("cool_min_speed"), travelTime, extrudeTime);

    // interpolate fan speed (for cool_fan_full_layer and for cool_min_layer_time_fan_speed_max)
    double fanSpeed = getSettingInPercentage("cool_fan_speed_min");
    double totalLayerTime = travelTime + extrudeTime;
    if (totalLayerTime < getSettingInSeconds("cool_min_layer_time"))
    {
        fanSpeed = getSettingInPercentage("cool_fan_speed_max");
    }
    else if (totalLayerTime < getSettingInSeconds("cool_min_layer_time_fan_speed_max"))
    { 
        // when forceMinimalLayerTime didn't change the extrusionSpeedFactor, we adjust the fan speed
        double minTime = (getSettingInSeconds("cool_min_layer_time"));
        double maxTime = (getSettingInSeconds("cool_min_layer_time_fan_speed_max"));
        double fanSpeedMin = getSettingInPercentage("cool_fan_speed_min");
        double fanSpeedMax = getSettingInPercentage("cool_fan_speed_max");
        fanSpeed = fanSpeedMax - (fanSpeedMax-fanSpeedMin) * (totalLayerTime - minTime) / (maxTime - minTime);
    }
    if (static_cast<int>(layer_nr) < getSettingAsCount("cool_fan_full_layer"))
    {
        //Slow down the fan on the layers below the [cool_fan_full_layer], where layer 0 is speed 0.
        fanSpeed = fanSpeed * layer_nr / getSettingAsCount("cool_fan_full_layer");
    }
    gcode.writeFanCommand(fanSpeed);
}

void FffGcodeWriter::finalize()
{
    gcode.finalize(max_object_height, getSettingInMillimetersPerSecond("speed_travel"), getSettingString("machine_end_gcode").c_str());
    for(int e=0; e<getSettingAsCount("machine_extruder_count"); e++)
        gcode.writeTemperatureCommand(e, 0, false);
}


}//namespace cura
