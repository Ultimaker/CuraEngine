
#include "FffGcodeWriter.h"

namespace cura
{


void GcodeWriter::writeGCode(SliceDataStorage& storage, TimeKeeper timeKeeper)
{
    gcode.preSetup(settings);
    
    gcode.resetTotalPrintTime();
    
    if (commandSocket)
        commandSocket->beginGCode();

    //Setup the retraction parameters.
    storage.retraction_config.amount = INT2MM(settings.getSettingInMicrons("retraction_amount"));
    storage.retraction_config.primeAmount = INT2MM(settings.getSettingInMicrons("retraction_extra_prime_amount"));
    storage.retraction_config.speed = settings.getSettingInMillimetersPerSecond("retraction_retract_speed");
    storage.retraction_config.primeSpeed = settings.getSettingInMillimetersPerSecond("retraction_prime_speed");
    storage.retraction_config.zHop = settings.getSettingInMicrons("retraction_hop");
    for(SliceMeshStorage& mesh : storage.meshes)
    {
        mesh.retraction_config.amount = INT2MM(mesh.settings->getSettingInMicrons("retraction_amount"));
        mesh.retraction_config.primeAmount = INT2MM(mesh.settings->getSettingInMicrons("retraction_extra_prime_amount"));
        mesh.retraction_config.speed = mesh.settings->getSettingInMillimetersPerSecond("retraction_retract_speed");
        mesh.retraction_config.primeSpeed = mesh.settings->getSettingInMillimetersPerSecond("retraction_prime_speed");
        mesh.retraction_config.zHop = mesh.settings->getSettingInMicrons("retraction_hop");
    }

    if (fileNr == 1)
    {
        if (gcode.getFlavor() != GCODE_FLAVOR_ULTIGCODE)
        {
            if (settings.hasSetting("material_bed_temperature") && settings.getSettingInDegreeCelsius("material_bed_temperature") > 0)
                gcode.writeBedTemperatureCommand(settings.getSettingInDegreeCelsius("material_bed_temperature"), true);
            
            for(SliceMeshStorage& mesh : storage.meshes)
                if (mesh.settings->hasSetting("material_print_temperature") && mesh.settings->getSettingInDegreeCelsius("material_print_temperature") > 0)
                    gcode.writeTemperatureCommand(mesh.settings->getSettingAsIndex("extruder_nr"), mesh.settings->getSettingInDegreeCelsius("material_print_temperature"));
            for(SliceMeshStorage& mesh : storage.meshes)
                if (mesh.settings->hasSetting("material_print_temperature") && mesh.settings->getSettingInDegreeCelsius("material_print_temperature") > 0)
                    gcode.writeTemperatureCommand(mesh.settings->getSettingAsIndex("extruder_nr"), mesh.settings->getSettingInDegreeCelsius("material_print_temperature"), true);
            gcode.writeCode(settings.getSettingString("machine_start_gcode").c_str());
        }
        gcode.writeComment("Generated with Cura_SteamEngine " VERSION);
        if (gcode.getFlavor() == GCODE_FLAVOR_BFB)
        {
            gcode.writeComment("enable auto-retraction");
            std::ostringstream tmp;
            tmp << "M227 S" << (settings.getSettingInMicrons("retraction_amount") * 2560 / 1000) << " P" << (settings.getSettingInMicrons("retraction_amount") * 2560 / 1000);
            gcode.writeLine(tmp.str().c_str());
        }
    }
    else
    {
        gcode.writeFanCommand(0);
        gcode.resetExtrusionValue();
        gcode.setZ(maxObjectHeight + 5000);
        gcode.writeMove(gcode.getPositionXY(), settings.getSettingInMillimetersPerSecond("speed_travel"), 0);
        gcode.writeMove(Point(storage.model_min.x, storage.model_min.y), settings.getSettingInMillimetersPerSecond("speed_travel"), 0);
    }
    fileNr++;

    unsigned int totalLayers = storage.meshes[0].layers.size();
    //gcode.writeComment("Layer count: %d", totalLayers);

    bool has_raft = settings.getSettingAsPlatformAdhesion("adhesion_type") == Adhesion_Raft;
    if (has_raft)
    {
        GCodePathConfig raft_base_config(&storage.retraction_config, "SUPPORT");
        raft_base_config.setSpeed(settings.getSettingInMillimetersPerSecond("raft_base_speed"));
        raft_base_config.setLineWidth(settings.getSettingInMicrons("raft_base_linewidth"));
        raft_base_config.setLayerHeight(settings.getSettingInMicrons("raft_base_thickness"));
        raft_base_config.setFilamentDiameter(settings.getSettingInMicrons("material_diameter"));
        raft_base_config.setFlow(settings.getSettingInPercentage("material_flow"));
        GCodePathConfig raft_interface_config(&storage.retraction_config, "SUPPORT");
        raft_interface_config.setSpeed(settings.getSettingInMillimetersPerSecond("raft_interface_speed"));
        raft_interface_config.setLineWidth(settings.getSettingInMicrons("raft_interface_linewidth"));
        raft_interface_config.setLayerHeight(settings.getSettingInMicrons("raft_base_thickness"));
        raft_interface_config.setFilamentDiameter(settings.getSettingInMicrons("material_diameter"));
        raft_interface_config.setFlow(settings.getSettingInPercentage("material_flow"));
        GCodePathConfig raft_surface_config(&storage.retraction_config, "SUPPORT");
        raft_surface_config.setSpeed(settings.getSettingInMillimetersPerSecond("raft_surface_speed"));
        raft_surface_config.setLineWidth(settings.getSettingInMicrons("raft_surface_line_width"));
        raft_surface_config.setLayerHeight(settings.getSettingInMicrons("raft_base_thickness"));
        raft_surface_config.setFilamentDiameter(settings.getSettingInMicrons("material_diameter"));
        raft_surface_config.setFlow(settings.getSettingInPercentage("material_flow"));

        {
            gcode.writeLayerComment(-2);
            gcode.writeComment("RAFT");
            GCodePlanner gcodeLayer(gcode, &storage.retraction_config, settings.getSettingInMillimetersPerSecond("speed_travel"), settings.getSettingInMicrons("retraction_min_travel"));
            if (settings.getSettingAsIndex("support_extruder_nr") > 0)
                gcodeLayer.setExtruder(settings.getSettingAsIndex("support_extruder_nr"));
            gcode.setZ(settings.getSettingInMicrons("raft_base_thickness"));
            gcodeLayer.addPolygonsByOptimizer(storage.raftOutline, &raft_base_config);

            Polygons raftLines;
            int offset_from_poly_outline = 0;
            generateLineInfill(storage.raftOutline, offset_from_poly_outline, raftLines, settings.getSettingInMicrons("raft_base_linewidth"), settings.getSettingInMicrons("raft_line_spacing"), settings.getSettingInPercentage("fill_overlap"), 0);
            gcodeLayer.addLinesByOptimizer(raftLines, &raft_base_config);

            gcode.writeFanCommand(settings.getSettingInPercentage("raft_base_fan_speed"));
            gcodeLayer.writeGCode(false, settings.getSettingInMicrons("raft_base_thickness"));
        }

        { /// this code block is about something which is of yet unknown
            gcode.writeLayerComment(-1);
            gcode.writeComment("RAFT");
            GCodePlanner gcodeLayer(gcode, &storage.retraction_config, settings.getSettingInMillimetersPerSecond("speed_travel"), settings.getSettingInMicrons("retraction_min_travel"));
            gcode.setZ(settings.getSettingInMicrons("raft_base_thickness") + settings.getSettingInMicrons("raft_interface_thickness"));

            Polygons raftLines;
            int offset_from_poly_outline = 0;
            generateLineInfill(storage.raftOutline, offset_from_poly_outline, raftLines, settings.getSettingInMicrons("raft_interface_line_width"), settings.getSettingInMicrons("raft_interface_line_spacing"), settings.getSettingInPercentage("fill_overlap"), settings.getSettingAsCount("raft_surface_layers") > 0 ? 45 : 90);
            gcodeLayer.addLinesByOptimizer(raftLines, &raft_interface_config);

            gcodeLayer.writeGCode(false, settings.getSettingInMicrons("raft_interface_thickness"));
        }

        for (int raftSurfaceLayer=1; raftSurfaceLayer<=settings.getSettingAsCount("raft_surface_layers"); raftSurfaceLayer++)
        {
            gcode.writeLayerComment(-1);
            gcode.writeComment("RAFT");
            GCodePlanner gcodeLayer(gcode, &storage.retraction_config, settings.getSettingInMillimetersPerSecond("speed_travel"), settings.getSettingInMicrons("retraction_min_travel"));
            gcode.setZ(settings.getSettingInMicrons("raft_base_thickness") + settings.getSettingInMicrons("raft_interface_thickness") + settings.getSettingInMicrons("raft_surface_thickness")*raftSurfaceLayer);

            Polygons raftLines;
            int offset_from_poly_outline = 0;
            generateLineInfill(storage.raftOutline, offset_from_poly_outline, raftLines, settings.getSettingInMicrons("raft_surface_line_width"), settings.getSettingInMicrons("raft_surface_line_spacing"), settings.getSettingInPercentage("fill_overlap"), 90 * raftSurfaceLayer);
            gcodeLayer.addLinesByOptimizer(raftLines, &raft_surface_config);

            gcodeLayer.writeGCode(false, settings.getSettingInMicrons("raft_interface_thickness"));
        }
    }

    for(unsigned int layer_nr=0; layer_nr<totalLayers; layer_nr++)
    {
        logProgress("export", layer_nr+1, totalLayers);
        if (commandSocket) commandSocket->sendProgress(2.0/3.0 + 1.0/3.0 * float(layer_nr) / float(totalLayers));

        int layer_thickness = settings.getSettingInMicrons("layer_height");
        if (layer_nr == 0)
        {
            layer_thickness = settings.getSettingInMicrons("layer_height_0");
        }

        storage.skirt_config.setSpeed(settings.getSettingInMillimetersPerSecond("skirt_speed"));
        storage.skirt_config.setLineWidth(settings.getSettingInMicrons("skirt_line_width"));
        storage.skirt_config.setFilamentDiameter(settings.getSettingInMicrons("material_diameter"));
        storage.skirt_config.setFlow(settings.getSettingInPercentage("material_flow"));
        storage.skirt_config.setLayerHeight(layer_thickness);

        storage.support_config.setLineWidth(settings.getSettingInMicrons("support_line_width"));
        storage.support_config.setSpeed(settings.getSettingInMillimetersPerSecond("speed_support"));
        storage.support_config.setFilamentDiameter(settings.getSettingInMicrons("material_diameter"));
        storage.support_config.setFlow(settings.getSettingInPercentage("material_flow"));
        storage.support_config.setLayerHeight(layer_thickness);
        for(SliceMeshStorage& mesh : storage.meshes)
        {
            mesh.inset0_config.setLineWidth(mesh.settings->getSettingInMicrons("wall_line_width_0"));
            mesh.inset0_config.setSpeed(mesh.settings->getSettingInMillimetersPerSecond("speed_wall_0"));
            mesh.inset0_config.setFilamentDiameter(mesh.settings->getSettingInMicrons("material_diameter"));
            mesh.inset0_config.setFlow(mesh.settings->getSettingInPercentage("material_flow"));
            mesh.inset0_config.setLayerHeight(layer_thickness);

            mesh.insetX_config.setLineWidth(mesh.settings->getSettingInMicrons("wall_line_width_x"));
            mesh.insetX_config.setSpeed(mesh.settings->getSettingInMillimetersPerSecond("speed_wall_x"));
            mesh.insetX_config.setFilamentDiameter(mesh.settings->getSettingInMicrons("material_diameter"));
            mesh.insetX_config.setFlow(mesh.settings->getSettingInPercentage("material_flow"));
            mesh.insetX_config.setLayerHeight(layer_thickness);

            mesh.skin_config.setLineWidth(mesh.settings->getSettingInMicrons("skin_line_width"));
            mesh.skin_config.setSpeed(mesh.settings->getSettingInMillimetersPerSecond("speed_topbottom"));
            mesh.skin_config.setFilamentDiameter(mesh.settings->getSettingInMicrons("material_diameter"));
            mesh.skin_config.setFlow(mesh.settings->getSettingInPercentage("material_flow"));
            mesh.skin_config.setLayerHeight(layer_thickness);

            for(unsigned int idx=0; idx<MAX_SPARSE_COMBINE; idx++)
            {
                mesh.infill_config[idx].setLineWidth(mesh.settings->getSettingInMicrons("infill_line_width") * (idx + 1));
                mesh.infill_config[idx].setSpeed(mesh.settings->getSettingInMillimetersPerSecond("speed_infill"));
                mesh.infill_config[idx].setFilamentDiameter(mesh.settings->getSettingInMicrons("material_diameter"));
                mesh.infill_config[idx].setFlow(mesh.settings->getSettingInPercentage("material_flow"));
                mesh.infill_config[idx].setLayerHeight(layer_thickness);
            }
        }

        int initial_speedup_layers = settings.getSettingAsCount("speed_slowdown_layers");
        if (static_cast<int>(layer_nr) < initial_speedup_layers)
        {
            int initial_layer_speed = settings.getSettingInMillimetersPerSecond("speed_layer_0");
            storage.support_config.smoothSpeed(initial_layer_speed, layer_nr, initial_speedup_layers);
            for(SliceMeshStorage& mesh : storage.meshes)
            {
                mesh.inset0_config.smoothSpeed(initial_layer_speed, layer_nr, initial_speedup_layers);
                mesh.insetX_config.smoothSpeed(initial_layer_speed, layer_nr, initial_speedup_layers);
                mesh.skin_config.smoothSpeed(initial_layer_speed, layer_nr, initial_speedup_layers);
                for(unsigned int idx=0; idx<MAX_SPARSE_COMBINE; idx++)
                {
                    mesh.infill_config[idx].smoothSpeed(initial_layer_speed, layer_nr, initial_speedup_layers);
                }
            }
        }

        gcode.writeLayerComment(layer_nr);

        GCodePlanner gcodeLayer(gcode, &storage.retraction_config, settings.getSettingInMillimetersPerSecond("speed_travel"), settings.getSettingInMicrons("retraction_min_travel"));
        int32_t z = settings.getSettingInMicrons("layer_height_0") + layer_nr * settings.getSettingInMicrons("layer_height");
        if (has_raft)
        {
            z += settings.getSettingInMicrons("raft_base_thickness") + settings.getSettingInMicrons("raft_interface_thickness") + settings.getSettingAsCount("raft_surface_layers")*settings.getSettingInMicrons("raft_surface_thickness");
            if (layer_nr == 0)
            {
                z += settings.getSettingInMicrons("raft_airgap_layer_0");
            } else {
                z += settings.getSettingInMicrons("raft_airgap");
            }
        }
        gcode.setZ(z);
        gcode.resetStartPosition();

        if (layer_nr == 0)
        {
            if (storage.skirt.size() > 0)
                gcodeLayer.addTravel(storage.skirt[storage.skirt.size()-1].closestPointTo(gcode.getPositionXY()));
            gcodeLayer.addPolygonsByOptimizer(storage.skirt, &storage.skirt_config);
        }

        bool printSupportFirst = (storage.support.generated && settings.getSettingAsIndex("support_extruder_nr") > 0 && settings.getSettingAsIndex("support_extruder_nr") == gcodeLayer.getExtruder());
        if (printSupportFirst)
            addSupportToGCode(storage, gcodeLayer, layer_nr);

        if (storage.oozeShield.size() > 0)
        {
            gcodeLayer.setAlwaysRetract(true);
            gcodeLayer.addPolygonsByOptimizer(storage.oozeShield[layer_nr], &storage.skirt_config);
            gcodeLayer.setAlwaysRetract(!settings.getSettingBoolean("retraction_combing"));
        }

        //Figure out in which order to print the meshes, do this by looking at the current extruder and preferer the meshes that use that extruder.
        std::vector<SliceMeshStorage*> mesh_order = calculateMeshOrder(storage, gcodeLayer.getExtruder());
        for(SliceMeshStorage* mesh : mesh_order)
        {
            addMeshLayerToGCode(storage, mesh, gcodeLayer, layer_nr);
        }
        if (!printSupportFirst)
            addSupportToGCode(storage, gcodeLayer, layer_nr);

        { //Finish the layer by applying speed corrections for minimal layer times and determine the fanSpeed
            double travelTime;
            double extrudeTime;
            gcodeLayer.getTimes(travelTime, extrudeTime);
            gcodeLayer.forceMinimalLayerTime(settings.getSettingInSeconds("cool_min_layer_time"), settings.getSettingInMillimetersPerSecond("cool_min_speed"), travelTime, extrudeTime);

            // interpolate fan speed (for cool_fan_full_layer and for cool_min_layer_time_fan_speed_max)
            int fanSpeed = settings.getSettingInPercentage("cool_fan_speed_min");
            double totalLayerTime = travelTime + extrudeTime;
            if (totalLayerTime < settings.getSettingInSeconds("cool_min_layer_time"))
            {
                fanSpeed = settings.getSettingInPercentage("cool_fan_speed_max");
            }
            else if (totalLayerTime < settings.getSettingInSeconds("cool_min_layer_time_fan_speed_max"))
            { 
                // when forceMinimalLayerTime didn't change the extrusionSpeedFactor, we adjust the fan speed
                double minTime = (settings.getSettingInSeconds("cool_min_layer_time"));
                double maxTime = (settings.getSettingInSeconds("cool_min_layer_time_fan_speed_max"));
                int fanSpeedMin = settings.getSettingInPercentage("cool_fan_speed_min");
                int fanSpeedMax = settings.getSettingInPercentage("cool_fan_speed_max");
                fanSpeed = fanSpeedMax - (fanSpeedMax-fanSpeedMin) * (totalLayerTime - minTime) / (maxTime - minTime);
            }
            if (static_cast<int>(layer_nr) < settings.getSettingAsCount("cool_fan_full_layer"))
            {
                //Slow down the fan on the layers below the [cool_fan_full_layer], where layer 0 is speed 0.
                fanSpeed = fanSpeed * layer_nr / settings.getSettingAsCount("cool_fan_full_layer");
            }
            gcode.writeFanCommand(fanSpeed);
        }

        gcodeLayer.writeGCode(settings.getSettingBoolean("cool_lift_head"), layer_nr > 0 ? settings.getSettingInMicrons("layer_height") : settings.getSettingInMicrons("layer_height_0"));
        if (commandSocket)
            commandSocket->sendGCodeLayer();
    }
    gcode.writeRetraction(&storage.retraction_config, true);

    log("Wrote layers in %5.2fs.\n", timeKeeper.restart());
    gcode.writeFanCommand(0);

    //Store the object height for when we are printing multiple objects, as we need to clear every one of them when moving to the next position.
    maxObjectHeight = std::max(maxObjectHeight, storage.model_max.z);

    if (commandSocket)
    {
        finalize();
        commandSocket->sendGCodeLayer();
        commandSocket->endSendSlicedObject();
        if (gcode.getFlavor() == GCODE_FLAVOR_ULTIGCODE)
        {
            std::ostringstream prefix;
            prefix << ";FLAVOR:UltiGCode\n";
            prefix << ";TIME:" << int(gcode.getTotalPrintTime()) << "\n";
            prefix << ";MATERIAL:" << int(gcode.getTotalFilamentUsed(0)) << "\n";
            prefix << ";MATERIAL2:" << int(gcode.getTotalFilamentUsed(1)) << "\n";
            commandSocket->sendGCodePrefix(prefix.str());
        }
    }
}


std::vector<SliceMeshStorage*> GcodeWriter::calculateMeshOrder(SliceDataStorage& storage, int current_extruder)
{
    std::vector<SliceMeshStorage*> ret;
    std::vector<SliceMeshStorage*> add_list;
    for(SliceMeshStorage& mesh : storage.meshes)
        add_list.push_back(&mesh);

    int add_extruder_nr = current_extruder;
    while(add_list.size() > 0)
    {
        for(unsigned int idx=0; idx<add_list.size(); idx++)
        {
            if (add_list[idx]->settings->getSettingAsIndex("extruder_nr") == add_extruder_nr)
            {
                ret.push_back(add_list[idx]);
                add_list.erase(add_list.begin() + idx);
                idx--;
            }
        }
        if (add_list.size() > 0)
            add_extruder_nr = add_list[0]->settings->getSettingAsIndex("extruder_nr");
    }
    return ret;
}

//Add a single layer from a single mesh-volume to the GCode
void GcodeWriter::addMeshLayerToGCode(SliceDataStorage& storage, SliceMeshStorage* mesh, GCodePlanner& gcodeLayer, int layer_nr)
{
    int prevExtruder = gcodeLayer.getExtruder();
    bool extruder_changed = gcodeLayer.setExtruder(mesh->settings->getSettingAsIndex("extruder_nr"));

    if (extruder_changed)
        addWipeTower(storage, gcodeLayer, layer_nr, prevExtruder);

    SliceLayer* layer = &mesh->layers[layer_nr];

    if (settings.getSettingBoolean("magic_polygon_mode"))
    {
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
        if (mesh->settings->getSettingBoolean("magic_spiralize"))
            mesh->inset0_config.spiralize = true;

        gcodeLayer.addPolygonsByOptimizer(polygons, &mesh->inset0_config);
        return;
    }


    PathOrderOptimizer partOrderOptimizer(gcode.getStartPositionXY());
    for(unsigned int partNr=0; partNr<layer->parts.size(); partNr++)
    {
        partOrderOptimizer.addPolygon(layer->parts[partNr].insets[0][0]);
    }
    partOrderOptimizer.optimize();

    for(int order_idx : partOrderOptimizer.polyOrder)
    {
        SliceLayerPart& part = layer->parts[order_idx];

        if (settings.getSettingBoolean("retraction_combing"))
            gcodeLayer.setCombBoundary(&part.combBoundary);
        else
            gcodeLayer.setAlwaysRetract(true);

        int fillAngle = 45;
        if (layer_nr & 1)
            fillAngle += 90;
        int extrusionWidth = settings.getSettingInMicrons("infill_line_width");

        //Add thicker (multiple layers) sparse infill.
        int sparse_infill_line_distance = settings.getSettingInMicrons("infill_line_distance");
        double infill_overlap = settings.getSettingInPercentage("fill_overlap");
        if (sparse_infill_line_distance > 0)
        {
            //Print the thicker sparse lines first. (double or more layer thickness, infill combined with previous layers)
            for(unsigned int n=1; n<part.sparse_outline.size(); n++)
            {
                Polygons fillPolygons;
                switch(settings.getSettingAsFillMethod("fill_pattern"))
                {
                case Fill_Grid:
                    generateGridInfill(part.sparse_outline[n], 0, fillPolygons, extrusionWidth, sparse_infill_line_distance * 2, infill_overlap, fillAngle);
                    gcodeLayer.addLinesByOptimizer(fillPolygons, &mesh->infill_config[n]);
                    break;
                case Fill_Lines:
                    generateLineInfill(part.sparse_outline[n], 0, fillPolygons, extrusionWidth, sparse_infill_line_distance, infill_overlap, fillAngle);
                    gcodeLayer.addLinesByOptimizer(fillPolygons, &mesh->infill_config[n]);
                    break;
                case Fill_Triangles:
                    generateTriangleInfill(part.sparse_outline[n], 0, fillPolygons, extrusionWidth, sparse_infill_line_distance * 3, infill_overlap, 0);
                    gcodeLayer.addLinesByOptimizer(fillPolygons, &mesh->infill_config[n]);
                    break;
                case Fill_Concentric:
                    generateConcentricInfill(part.sparse_outline[n], fillPolygons, sparse_infill_line_distance);
                    gcodeLayer.addPolygonsByOptimizer(fillPolygons, &mesh->infill_config[n]);
                    break;
                case Fill_ZigZag:
                    generateZigZagInfill(part.sparse_outline[n], fillPolygons, extrusionWidth, sparse_infill_line_distance, infill_overlap, fillAngle, false, false);
                    gcodeLayer.addPolygonsByOptimizer(fillPolygons, &mesh->infill_config[n]);
                    break;
                default:
                    logError("fill_pattern has unknown value.\n");
                    break;
                }
            }
        }

        //Combine the 1 layer thick infill with the top/bottom skin and print that as one thing.
        Polygons infillPolygons;
        Polygons infillLines;
        if (sparse_infill_line_distance > 0 && part.sparse_outline.size() > 0)
        {
            switch(settings.getSettingAsFillMethod("fill_pattern"))
            {
            case Fill_Grid:
                generateGridInfill(part.sparse_outline[0], 0, infillLines, extrusionWidth, sparse_infill_line_distance * 2, infill_overlap, fillAngle);
                break;
            case Fill_Lines:
                generateLineInfill(part.sparse_outline[0], 0, infillLines, extrusionWidth, sparse_infill_line_distance, infill_overlap, fillAngle);
                break;
            case Fill_Triangles:
                generateTriangleInfill(part.sparse_outline[0], 0, infillLines, extrusionWidth, sparse_infill_line_distance * 3, infill_overlap, 0);
                break;
            case Fill_Concentric:
                generateConcentricInfill(part.sparse_outline[0], infillPolygons, sparse_infill_line_distance);
                break;
            case Fill_ZigZag:
                generateZigZagInfill(part.sparse_outline[0], infillLines, extrusionWidth, sparse_infill_line_distance, infill_overlap, fillAngle, false, false);
                break;
            default:
                logError("fill_pattern has unknown value.\n");
                break;
            }
        }
        gcodeLayer.addPolygonsByOptimizer(infillPolygons, &mesh->infill_config[0]);
        gcodeLayer.addLinesByOptimizer(infillLines, &mesh->infill_config[0]); 

        if (settings.getSettingAsCount("wall_line_count") > 0)
        {
            if (settings.getSettingBoolean("magic_spiralize"))
            {
                if (static_cast<int>(layer_nr) >= settings.getSettingAsCount("bottom_layers"))
                    mesh->inset0_config.spiralize = true;
                if (static_cast<int>(layer_nr) == settings.getSettingAsCount("bottom_layers") && part.insets.size() > 0)
                    gcodeLayer.addPolygonsByOptimizer(part.insets[0], &mesh->insetX_config);
            }
            for(int insetNr=part.insets.size()-1; insetNr>-1; insetNr--)
            {
                if (insetNr == 0)
                    gcodeLayer.addPolygonsByOptimizer(part.insets[insetNr], &mesh->inset0_config);
                else
                    gcodeLayer.addPolygonsByOptimizer(part.insets[insetNr], &mesh->insetX_config);
            }
        }

        Polygons skinPolygons;
        Polygons skinLines;
        for(SkinPart& skin_part : part.skin_parts)
        {
            int bridge = -1;
            if (layer_nr > 0)
                bridge = bridgeAngle(skin_part.outline, &mesh->layers[layer_nr-1]);
            if (bridge > -1)
            {
                generateLineInfill(skin_part.outline, 0, skinLines, extrusionWidth, extrusionWidth, infill_overlap, bridge);
            }else{
                switch(settings.getSettingAsFillMethod("top_bottom_pattern"))
                {
                case Fill_Lines:
                    for (Polygons& skin_perimeter : skin_part.insets)
                    {
                        gcodeLayer.addPolygonsByOptimizer(skin_perimeter, &mesh->skin_config); // add polygons to gcode in inward order
                    }
                    if (skin_part.insets.size() > 0)
                    {
                        generateLineInfill(skin_part.insets.back(), -extrusionWidth/2, skinLines, extrusionWidth, extrusionWidth, infill_overlap, fillAngle);
                        if (settings.getSettingString("fill_perimeter_gaps") != "Nowhere")
                        {
                            generateLineInfill(skin_part.perimeterGaps, 0, skinLines, extrusionWidth, extrusionWidth, 0, fillAngle);
                        }
                    } 
                    else
                    {
                        generateLineInfill(skin_part.outline, 0, skinLines, extrusionWidth, extrusionWidth, infill_overlap, fillAngle);
                    }
                    break;
                case Fill_Concentric:
                    {
                        Polygons in_outline;
                        offsetSafe(skin_part.outline, -extrusionWidth/2, extrusionWidth, in_outline, settings.getSettingBoolean("wall_overlap_avoid_enabled"));
                        if (settings.getSettingString("fill_perimeter_gaps") != "Nowhere")
                        {
                            generateConcentricInfillDense(in_outline, skinPolygons, &part.perimeterGaps, extrusionWidth, settings.getSettingBoolean("wall_overlap_avoid_enabled"));
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
        if (settings.getSettingString("fill_perimeter_gaps") != "Nowhere")
        {
            generateLineInfill(part.perimeterGaps, 0, skinLines, extrusionWidth, extrusionWidth, 0, fillAngle);
        }
        
        
        gcodeLayer.addPolygonsByOptimizer(skinPolygons, &mesh->skin_config);
        gcodeLayer.addLinesByOptimizer(skinLines, &mesh->skin_config);

        //After a layer part, make sure the nozzle is inside the comb boundary, so we do not retract on the perimeter.
        if (!settings.getSettingBoolean("magic_spiralize") || static_cast<int>(layer_nr) < settings.getSettingAsCount("bottom_layers"))
            gcodeLayer.moveInsideCombBoundary(extrusionWidth * 2);
    }
    gcodeLayer.setCombBoundary(nullptr);
}

void GcodeWriter::addSupportToGCode(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int layer_nr)
{
    if (!storage.support.generated)
        return;
    
    
    if (settings.getSettingAsIndex("support_extruder_nr") > -1)
    {
        int prevExtruder = gcodeLayer.getExtruder();
        if (gcodeLayer.setExtruder(settings.getSettingAsIndex("support_extruder_nr")))
            addWipeTower(storage, gcodeLayer, layer_nr, prevExtruder);
    }
    Polygons support;
    if (storage.support.generated) 
        support = storage.support.supportAreasPerLayer[layer_nr];
    
    std::vector<PolygonsPart> supportIslands = support.splitIntoParts();

    PathOrderOptimizer islandOrderOptimizer(gcode.getPositionXY());
    for(unsigned int n=0; n<supportIslands.size(); n++)
    {
        islandOrderOptimizer.addPolygon(supportIslands[n][0]);
    }
    islandOrderOptimizer.optimize();

    for(unsigned int n=0; n<supportIslands.size(); n++)
    {
        PolygonsPart& island = supportIslands[islandOrderOptimizer.polyOrder[n]];

        Polygons supportLines;
        int support_line_distance = settings.getSettingInMicrons("support_line_distance");
        double infill_overlap = settings.getSettingInPercentage("fill_overlap");
        if (support_line_distance > 0)
        {
            int extrusionWidth = settings.getSettingInMicrons("wall_line_width_x");
            switch(settings.getSettingAsFillMethod("support_pattern"))
            {
            case Fill_Grid:
                {
                    int offset_from_outline = 0;
                    if (support_line_distance > extrusionWidth * 4)
                    {
                        generateGridInfill(island, offset_from_outline, supportLines, extrusionWidth, support_line_distance*2, infill_overlap, 0);
                    }else{
                        generateLineInfill(island, offset_from_outline, supportLines, extrusionWidth, support_line_distance, infill_overlap, (layer_nr & 1) ? 0 : 90);
                    }
                }
                break;
            case Fill_Lines:
                {
                    int offset_from_outline = 0;
                    if (layer_nr == 0)
                    {
                        generateGridInfill(island, offset_from_outline, supportLines, extrusionWidth, support_line_distance, infill_overlap + 150, 0);
                    }else{
                        generateLineInfill(island, offset_from_outline, supportLines, extrusionWidth, support_line_distance, infill_overlap, 0);
                    }
                }
                break;
            case Fill_ZigZag:
                {
                    int offset_from_outline = 0;
                    if (layer_nr == 0)
                    {
                        generateGridInfill(island, offset_from_outline, supportLines, extrusionWidth, support_line_distance, infill_overlap + 150, 0);
                    }else{
                        generateZigZagInfill(island, supportLines, extrusionWidth, support_line_distance, infill_overlap, 0, settings.getSettingBoolean("support_connect_zigzags"), true);
                    }
                }
                break;
            default:
                logError("Unknown fill method for support\n");
                break;
            }
        }

        gcodeLayer.forceRetract();
        if (settings.getSettingBoolean("retraction_combing"))
            gcodeLayer.setCombBoundary(&island);
        if (settings.getSettingAsFillMethod("support_pattern") == Fill_Grid || ( settings.getSettingAsFillMethod("support_pattern") == Fill_ZigZag && layer_nr == 0 ) )
            gcodeLayer.addPolygonsByOptimizer(island, &storage.support_config);
        gcodeLayer.addLinesByOptimizer(supportLines, &storage.support_config);
        gcodeLayer.setCombBoundary(nullptr);
    }
}


void GcodeWriter::addWipeTower(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int layer_nr, int prevExtruder)
{
    if (settings.getSettingInMicrons("wipe_tower_size") < 1)
        return;

    int64_t offset = -settings.getSettingInMicrons("wall_line_width_x");
    if (layer_nr > 0)
        offset *= 2;
    
    //If we changed extruder, print the wipe/prime tower for this nozzle;
    std::vector<Polygons> insets;
    if ((layer_nr % 2) == 1)
        insets.push_back(storage.wipeTower.offset(offset / 2));
    else
        insets.push_back(storage.wipeTower);
    while(true)
    {
        Polygons new_inset = insets[insets.size() - 1].offset(offset);
        if (new_inset.size() < 1)
            break;
        insets.push_back(new_inset);
    }
    for(unsigned int n=0; n<insets.size(); n++)
    {
        gcodeLayer.addPolygonsByOptimizer(insets[insets.size() - 1 - n], &storage.meshes[0].insetX_config);
    }
    
    //Make sure we wipe the old extruder on the wipe tower.
    gcodeLayer.addTravel(storage.wipePoint - gcode.getExtruderOffset(prevExtruder) + gcode.getExtruderOffset(gcodeLayer.getExtruder()));
}



} // namespace cura