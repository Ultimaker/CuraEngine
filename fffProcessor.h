#ifndef FFF_PROCESSOR_H
#define FFF_PROCESSOR_H

//FusedFilamentFabrication processor.
class fffProcessor
{
private:
    int maxObjectHeight;
    int fileNr;
    GCodeExport gcode;
    ConfigSettings& config;
    TimeKeeper timeKeeper;

    GCodePathConfig skirtConfig;
    GCodePathConfig inset0Config;
    GCodePathConfig inset1Config;
    GCodePathConfig fillConfig;
    GCodePathConfig supportConfig;
public:
    fffProcessor(ConfigSettings& config)
    : config(config)
    {
        fileNr = 1;
        maxObjectHeight = 0;
    }
    
    bool setTargetFile(const char* filename)
    {
        gcode.setFilename(filename);
        if (gcode.isValid())
            gcode.addComment("Generated with Cura_SteamEngine %s", VERSION);
        return gcode.isValid();
    }
    
    bool processFile(const char* input_filename)
    {
        if (!gcode.isValid())
            return false;

        TimeKeeper timeKeeperTotal;
        SliceDataStorage storage;
        preSetup();
        if (!prepareModel(storage, input_filename))
            return false;
        
        processSliceData(storage);
        writeGCode(storage);

        logProgress("process", 1, 1);
        log("Total time elapsed %5.2fs.\n", timeKeeperTotal.restart());

        return true;
    }
    
    void finalize()
    {
        if (!gcode.isValid())
            return;

        gcode.addFanCommand(0);
        gcode.addRetraction();
        gcode.setZ(maxObjectHeight + 5000);
        gcode.addMove(gcode.getPositionXY(), config.moveSpeed, 0);
        gcode.addCode(config.endCode);
        log("Print time: %d\n", int(gcode.getTotalPrintTime()));
        log("Filament: %d\n", int(gcode.getTotalFilamentUsed(0)));
        log("Filament2: %d\n", int(gcode.getTotalFilamentUsed(1)));
        
        if (gcode.getFlavor() == GCODE_FLAVOR_ULTIGCODE)
        {
            char numberString[16];
            sprintf(numberString, "%d", int(gcode.getTotalPrintTime()));
            gcode.replaceTagInStart("<__TIME__>", numberString);
            sprintf(numberString, "%d", int(gcode.getTotalFilamentUsed(0)));
            gcode.replaceTagInStart("<FILAMENT>", numberString);
            sprintf(numberString, "%d", int(gcode.getTotalFilamentUsed(1)));
            gcode.replaceTagInStart("<FILAMEN2>", numberString);
        }
    }

private:
    void preSetup()
    {
        skirtConfig.setData(config.printSpeed, config.extrusionWidth, "SKIRT");
        inset0Config.setData(config.printSpeed, config.extrusionWidth, "WALL-OUTER");
        inset1Config.setData(config.printSpeed, config.extrusionWidth, "WALL-INNER");
        fillConfig.setData(config.infillSpeed, config.extrusionWidth, "FILL");
        supportConfig.setData(config.printSpeed, config.extrusionWidth, "SUPPORT");

        for(unsigned int n=1; n<MAX_EXTRUDERS;n++)
            gcode.setExtruderOffset(n, config.extruderOffset[n].p());
        gcode.setFlavor(config.gcodeFlavor);
        gcode.setRetractionSettings(config.retractionAmount, config.retractionSpeed, config.retractionAmountExtruderSwitch, config.minimalExtrusionBeforeRetraction);
    }

    bool prepareModel(SliceDataStorage& storage, const char* input_filename)
    {
        timeKeeper.restart();
        log("Loading %s from disk...\n", input_filename);
        SimpleModel* m = loadModel(input_filename, config.matrix);
        if (!m)
        {
            log("Failed to load model: %s\n", input_filename);
            return false;
        }
        log("Loaded from disk in %5.3fs\n", timeKeeper.restart());
        log("Analyzing and optimizing model...\n");
        OptimizedModel* om = new OptimizedModel(m, Point3(config.objectPosition.X, config.objectPosition.Y, -config.objectSink));
        for(unsigned int v = 0; v < m->volumes.size(); v++)
        {
            log("  Face counts: %i -> %i %0.1f%%\n", (int)m->volumes[v].faces.size(), (int)om->volumes[v].faces.size(), float(om->volumes[v].faces.size()) / float(m->volumes[v].faces.size()) * 100);
            log("  Vertex counts: %i -> %i %0.1f%%\n", (int)m->volumes[v].faces.size() * 3, (int)om->volumes[v].points.size(), float(om->volumes[v].points.size()) / float(m->volumes[v].faces.size() * 3) * 100);
        }
        delete m;
        log("Optimize model %5.3fs \n", timeKeeper.restart());
        //om->saveDebugSTL("c:\\models\\output.stl");
        
        log("Slicing model...\n");
        vector<Slicer*> slicerList;
        for(unsigned int volumeIdx=0; volumeIdx < om->volumes.size(); volumeIdx++)
        {
            slicerList.push_back(new Slicer(&om->volumes[volumeIdx], config.initialLayerThickness / 2, config.layerThickness, config.fixHorrible & FIX_HORRIBLE_KEEP_NONE_CLOSED, config.fixHorrible & FIX_HORRIBLE_EXTENSIVE_STITCHING));
            //slicerList[volumeIdx]->dumpSegmentsToHTML("C:\\models\\output.html");
        }
        log("Sliced model in %5.3fs\n", timeKeeper.restart());

        fprintf(stdout,"Generating support map...\n");
        generateSupportGrid(storage.support, om, config.supportAngle, config.supportEverywhere > 0, config.supportXYDistance, config.supportZDistance);
        
        storage.modelSize = om->modelSize;
        storage.modelMin = om->vMin;
        storage.modelMax = om->vMax;
        delete om;

        log("Generating layer parts...\n");
        for(unsigned int volumeIdx=0; volumeIdx < slicerList.size(); volumeIdx++)
        {
            storage.volumes.push_back(SliceVolumeStorage());
            createLayerParts(storage.volumes[volumeIdx], slicerList[volumeIdx], config.fixHorrible & (FIX_HORRIBLE_UNION_ALL_TYPE_A | FIX_HORRIBLE_UNION_ALL_TYPE_B | FIX_HORRIBLE_UNION_ALL_TYPE_C));
            delete slicerList[volumeIdx];
        }
        log("Generated layer parts in %5.3fs\n", timeKeeper.restart());
        return true;
    }
    
    void processSliceData(SliceDataStorage& storage)
    {
        //carveMultipleVolumes(storage.volumes);
        generateMultipleVolumesOverlap(storage.volumes, config.multiVolumeOverlap);
        //dumpLayerparts(storage, "c:/models/output.html");
        
        const unsigned int totalLayers = storage.volumes[0].layers.size();
        for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
        {
            for(unsigned int volumeIdx=0; volumeIdx<storage.volumes.size(); volumeIdx++)
            {
                int insetCount = config.insetCount;
                if (config.spiralizeMode && int(layerNr) < config.downSkinCount && layerNr % 2 == 1)//Add extra insets every 2 layers when spiralizing, this makes bottoms of cups watertight.
                    insetCount += 5;
                generateInsets(&storage.volumes[volumeIdx].layers[layerNr], config.extrusionWidth, insetCount);
            }
            logProgress("inset",layerNr+1,totalLayers);
        }
        if (config.enableOozeShield)
        {
            for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
            {
                Polygons oozeShield;
                for(unsigned int volumeIdx=0; volumeIdx<storage.volumes.size(); volumeIdx++)
                {
                    for(unsigned int partNr=0; partNr<storage.volumes[volumeIdx].layers[layerNr].parts.size(); partNr++)
                    {
                        oozeShield = oozeShield.unionPolygons(storage.volumes[volumeIdx].layers[layerNr].parts[partNr].outline.offset(2000));
                    }
                }
                storage.oozeShield.push_back(oozeShield);
            }
            
            for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
                storage.oozeShield[layerNr] = storage.oozeShield[layerNr].offset(-1000).offset(1000);
            int offsetAngle = tan(60.0*M_PI/180) * config.layerThickness;//Allow for a 60deg angle in the oozeShield.
            for(unsigned int layerNr=1; layerNr<totalLayers; layerNr++)
                storage.oozeShield[layerNr] = storage.oozeShield[layerNr].unionPolygons(storage.oozeShield[layerNr-1].offset(-offsetAngle));
            for(unsigned int layerNr=totalLayers-1; layerNr>0; layerNr--)
                storage.oozeShield[layerNr-1] = storage.oozeShield[layerNr-1].unionPolygons(storage.oozeShield[layerNr].offset(-offsetAngle));
        }
        log("Generated inset in %5.3fs\n", timeKeeper.restart());

        for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
        {
            if (!config.spiralizeMode || int(layerNr) < config.downSkinCount)    //Only generate up/downskin and infill for the first X layers when spiralize is choosen.
            {
                for(unsigned int volumeIdx=0; volumeIdx<storage.volumes.size(); volumeIdx++)
                {
                    generateSkins(layerNr, storage.volumes[volumeIdx], config.extrusionWidth, config.downSkinCount, config.upSkinCount, config.infillOverlap);
                    generateSparse(layerNr, storage.volumes[volumeIdx], config.extrusionWidth, config.downSkinCount, config.upSkinCount);
                }
            }
            logProgress("skin",layerNr+1,totalLayers);
        }
        log("Generated up/down skin in %5.3fs\n", timeKeeper.restart());

        if (config.wipeTowerSize > 0)
        {
            PolygonRef p = storage.wipeTower.newPoly();
            p.add(Point(storage.modelMin.x - 3000, storage.modelMax.y + 3000));
            p.add(Point(storage.modelMin.x - 3000, storage.modelMax.y + 3000 + config.wipeTowerSize));
            p.add(Point(storage.modelMin.x - 3000 - config.wipeTowerSize, storage.modelMax.y + 3000 + config.wipeTowerSize));
            p.add(Point(storage.modelMin.x - 3000 - config.wipeTowerSize, storage.modelMax.y + 3000));
            
            storage.wipePoint = Point(storage.modelMin.x - 3000 - config.wipeTowerSize / 2, storage.modelMax.y + 3000 + config.wipeTowerSize / 2);
        }

        generateSkirt(storage, config.skirtDistance, config.extrusionWidth, config.skirtLineCount, config.skirtMinLength);
        generateRaft(storage, config.raftMargin);
        
        for(unsigned int volumeIdx=0; volumeIdx<storage.volumes.size(); volumeIdx++)
        {
            for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
            {
                for(unsigned int partNr=0; partNr<storage.volumes[volumeIdx].layers[layerNr].parts.size(); partNr++)
                {
                    if (layerNr > 0)
                        storage.volumes[volumeIdx].layers[layerNr].parts[partNr].bridgeAngle = bridgeAngle(&storage.volumes[volumeIdx].layers[layerNr].parts[partNr], &storage.volumes[volumeIdx].layers[layerNr-1]);
                    else
                        storage.volumes[volumeIdx].layers[layerNr].parts[partNr].bridgeAngle = -1;
                }
            }
        }
    }

    void writeGCode(SliceDataStorage& storage)
    {
        if (fileNr == 1)
        {
            if (gcode.getFlavor() == GCODE_FLAVOR_ULTIGCODE)
            {
                gcode.addCode(";FLAVOR:UltiGCode");
                gcode.addCode(";TIME:<__TIME__>");
                gcode.addCode(";MATERIAL:<FILAMENT>");
                gcode.addCode(";MATERIAL2:<FILAMEN2>");
            }
            gcode.addCode(config.startCode);
        }else{
            gcode.addFanCommand(0);
            gcode.resetExtrusionValue();
            gcode.addRetraction();
            gcode.setZ(maxObjectHeight + 5000);
            gcode.addMove(Point(storage.modelMin.x, storage.modelMin.y), config.moveSpeed, 0);
        }
        fileNr++;
        
        unsigned int totalLayers = storage.volumes[0].layers.size();
        gcode.addComment("Layer count: %d", totalLayers);

        if (config.raftBaseThickness > 0 && config.raftInterfaceThickness > 0)
        {
            GCodePathConfig raftBaseConfig(config.initialLayerSpeed, config.raftBaseLinewidth, "SUPPORT");
            GCodePathConfig raftInterfaceConfig(config.initialLayerSpeed, config.raftInterfaceLinewidth, "SUPPORT");
            {
                gcode.addComment("LAYER:-2");
                gcode.addComment("RAFT");
                GCodePlanner gcodeLayer(gcode, config.moveSpeed, config.retractionMinimalDistance);
                gcode.setZ(config.raftBaseThickness);
                gcode.setExtrusion(config.raftBaseThickness, config.filamentDiameter, config.filamentFlow);
                gcodeLayer.addPolygonsByOptimizer(storage.raftOutline, &raftBaseConfig);
                
                Polygons raftLines;
                generateLineInfill(storage.raftOutline, raftLines, config.raftBaseLinewidth, config.raftLineSpacing, config.infillOverlap, 0);
                gcodeLayer.addPolygonsByOptimizer(raftLines, &raftBaseConfig);
                
                gcodeLayer.writeGCode(false, config.raftBaseThickness);
            }

            {
                gcode.addComment("LAYER:-1");
                gcode.addComment("RAFT");
                GCodePlanner gcodeLayer(gcode, config.moveSpeed, config.retractionMinimalDistance);
                gcode.setZ(config.raftBaseThickness + config.raftInterfaceThickness);
                gcode.setExtrusion(config.raftInterfaceThickness, config.filamentDiameter, config.filamentFlow);
                
                Polygons raftLines;
                generateLineInfill(storage.raftOutline, raftLines, config.raftInterfaceLinewidth, config.raftLineSpacing, config.infillOverlap, 90);
                gcodeLayer.addPolygonsByOptimizer(raftLines, &raftInterfaceConfig);
                
                gcodeLayer.writeGCode(false, config.raftInterfaceThickness);
            }
        }

        int volumeIdx = 0;
        for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
        {
            logProgress("export", layerNr+1, totalLayers);
            
            gcode.addComment("LAYER:%d", layerNr);
            if (layerNr == 0)
                gcode.setExtrusion(config.initialLayerThickness, config.filamentDiameter, config.filamentFlow);
            else
                gcode.setExtrusion(config.layerThickness, config.filamentDiameter, config.filamentFlow);

            GCodePlanner gcodeLayer(gcode, config.moveSpeed, config.retractionMinimalDistance);
            int32_t z = config.initialLayerThickness + layerNr * config.layerThickness;
            z += config.raftBaseThickness + config.raftInterfaceThickness;
            gcode.setZ(z);
            
            bool printSupportFirst = (storage.support.generated && config.supportExtruder > 0 && config.supportExtruder == gcodeLayer.getExtruder());
            if (printSupportFirst)
                addSupportToGCode(storage, gcodeLayer, layerNr);
            
            for(unsigned int volumeCnt = 0; volumeCnt < storage.volumes.size(); volumeCnt++)
            {
                if (volumeCnt > 0)
                    volumeIdx = (volumeIdx + 1) % storage.volumes.size();
                addVolumeLayerToGCode(storage, gcodeLayer, volumeIdx, layerNr);
            }
            if (!printSupportFirst)
                addSupportToGCode(storage, gcodeLayer, layerNr);
            
            //Finish the layer by applying speed corrections for minimal layer times and slowdown for the initial layer.
            if (int(layerNr) < config.initialSpeedupLayers)
            {
                int n = config.initialSpeedupLayers;
                int layer0Factor = config.initialLayerSpeed * 100 / config.printSpeed;
                gcodeLayer.setExtrudeSpeedFactor((layer0Factor * (n - layerNr) + 100 * (layerNr)) / n);
                if (layerNr == 0)//On the first layer, also slow down the travel
                    gcodeLayer.setTravelSpeedFactor(layer0Factor);
            }
            gcodeLayer.forceMinimalLayerTime(config.minimalLayerTime, config.minimalFeedrate);

            int fanSpeed = config.fanSpeedMin;
            if (gcodeLayer.getExtrudeSpeedFactor() <= 50)
            {
                fanSpeed = config.fanSpeedMax;
            }else{
                int n = gcodeLayer.getExtrudeSpeedFactor() - 50;
                fanSpeed = config.fanSpeedMin * n / 50 + config.fanSpeedMax * (50 - n) / 50;
            }
            if (int(layerNr) < config.fanFullOnLayerNr)
            {
                //Slow down the fan on the layers below the [fanFullOnLayerNr], where layer 0 is speed 0.
                fanSpeed = fanSpeed * layerNr / config.fanFullOnLayerNr;
            }
            gcode.addFanCommand(fanSpeed);

            gcodeLayer.writeGCode(config.coolHeadLift > 0, int(layerNr) > 0 ? config.layerThickness : config.initialLayerThickness);
        }

        /* support debug
        for(int32_t y=0; y<storage.support.gridHeight; y++)
        {
            for(int32_t x=0; x<storage.support.gridWidth; x++)
            {
                unsigned int n = x+y*storage.support.gridWidth;
                if (storage.support.grid[n].size() < 1) continue;
                int32_t z = storage.support.grid[n][0].z;
                gcode.addMove(Point3(x * storage.support.gridScale + storage.support.gridOffset.X, y * storage.support.gridScale + storage.support.gridOffset.Y, 0), 0);
                gcode.addMove(Point3(x * storage.support.gridScale + storage.support.gridOffset.X, y * storage.support.gridScale + storage.support.gridOffset.Y, z), z);
                gcode.addMove(Point3(x * storage.support.gridScale + storage.support.gridOffset.X, y * storage.support.gridScale + storage.support.gridOffset.Y, 0), 0);
            }
        }
        //*/
        
        log("Wrote layers in %5.2fs.\n", timeKeeper.restart());
        gcode.tellFileSize();
        gcode.addFanCommand(0);

        //Store the object height for when we are printing multiple objects, as we need to clear every one of them when moving to the next position.
        maxObjectHeight = std::max(maxObjectHeight, storage.modelSize.z);
    }
    
    //Add a single layer from a single mesh-volume to the GCode
    void addVolumeLayerToGCode(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int volumeIdx, int layerNr)
    {
        int prevExtruder = gcodeLayer.getExtruder();
        bool extruderChanged = gcodeLayer.setExtruder(volumeIdx);
        if (layerNr == 0 && volumeIdx == 0)
            gcodeLayer.addPolygonsByOptimizer(storage.skirt, &skirtConfig);

        SliceLayer* layer = &storage.volumes[volumeIdx].layers[layerNr];
        if (extruderChanged)
            addWipeTower(storage, gcodeLayer, layerNr, prevExtruder);
        
        if (storage.oozeShield.size() > 0 && storage.volumes.size() > 1)
        {
            gcodeLayer.setAlwaysRetract(true);
            gcodeLayer.addPolygonsByOptimizer(storage.oozeShield[layerNr], &skirtConfig);
            gcodeLayer.setAlwaysRetract(!config.enableCombing);
        }
        
        PathOrderOptimizer partOrderOptimizer(gcode.getPositionXY());
        for(unsigned int partNr=0; partNr<layer->parts.size(); partNr++)
        {
            partOrderOptimizer.addPolygon(layer->parts[partNr].insets[0][0]);
        }
        partOrderOptimizer.optimize();
        
        for(unsigned int partCounter=0; partCounter<partOrderOptimizer.polyOrder.size(); partCounter++)
        {
            SliceLayerPart* part = &layer->parts[partOrderOptimizer.polyOrder[partCounter]];
            
            if (config.enableCombing)
                gcodeLayer.setCombBoundary(&part->combBoundery);
            else
                gcodeLayer.setAlwaysRetract(true);
            
            if (config.insetCount > 0)
            {
                if (config.spiralizeMode)
                {
                    if (int(layerNr) >= config.downSkinCount)
                        inset0Config.spiralize = true;
                    if (int(layerNr) == config.downSkinCount && part->insets.size() > 0)
                        gcodeLayer.addPolygonsByOptimizer(part->insets[0], &inset1Config);
                }
                for(int insetNr=part->insets.size()-1; insetNr>-1; insetNr--)
                {
                    if (insetNr == 0)
                        gcodeLayer.addPolygonsByOptimizer(part->insets[insetNr], &inset0Config);
                    else
                        gcodeLayer.addPolygonsByOptimizer(part->insets[insetNr], &inset1Config);
                }
            }
            
            Polygons fillPolygons;
            int fillAngle = 45;
            if (layerNr & 1) 
                fillAngle += 90;
            //int sparseSteps[1] = {config.extrusionWidth};
            //generateConcentricInfill(part->skinOutline, fillPolygons, sparseSteps, 1);
            generateLineInfill(part->skinOutline, fillPolygons, config.extrusionWidth, config.extrusionWidth, config.infillOverlap, (part->bridgeAngle > -1) ? part->bridgeAngle : fillAngle);
            //int sparseSteps[2] = {config.extrusionWidth*5, config.extrusionWidth * 0.8};
            //generateConcentricInfill(part->sparseOutline, fillPolygons, sparseSteps, 2);
            if (config.sparseInfillLineDistance > 0)
            {
                if (config.sparseInfillLineDistance > config.extrusionWidth * 4)
                {
                    generateLineInfill(part->sparseOutline, fillPolygons, config.extrusionWidth, config.sparseInfillLineDistance * 2, config.infillOverlap, 45);
                    generateLineInfill(part->sparseOutline, fillPolygons, config.extrusionWidth, config.sparseInfillLineDistance * 2, config.infillOverlap, 45 + 90);
                }
                else
                {
                    generateLineInfill(part->sparseOutline, fillPolygons, config.extrusionWidth, config.sparseInfillLineDistance, config.infillOverlap, fillAngle);
                }
            }

            gcodeLayer.addPolygonsByOptimizer(fillPolygons, &fillConfig);
            
            //After a layer part, make sure the nozzle is inside the comb boundary, so we do not retract on the perimeter.
            if (!config.spiralizeMode || int(layerNr) < config.downSkinCount)
                gcodeLayer.moveInsideCombBoundary(config.extrusionWidth * 2);
        }
        gcodeLayer.setCombBoundary(NULL);
    }
    
    void addSupportToGCode(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int layerNr)
    {
        if (!storage.support.generated)
            return;
        
        if (config.supportExtruder > -1)
        {
            int prevExtruder = gcodeLayer.getExtruder();
            if (gcodeLayer.setExtruder(config.supportExtruder))
                addWipeTower(storage, gcodeLayer, layerNr, prevExtruder);
            
            if (storage.oozeShield.size() > 0 && storage.volumes.size() == 1)
            {
                gcodeLayer.setAlwaysRetract(true);
                gcodeLayer.addPolygonsByOptimizer(storage.oozeShield[layerNr], &skirtConfig);
                gcodeLayer.setAlwaysRetract(!config.enableCombing);
            }
        }
        int32_t z = config.initialLayerThickness + layerNr * config.layerThickness;
        SupportPolyGenerator supportGenerator(storage.support, z);
        for(unsigned int volumeCnt = 0; volumeCnt < storage.volumes.size(); volumeCnt++)
        {
            SliceLayer* layer = &storage.volumes[volumeCnt].layers[layerNr];
            for(unsigned int n=0; n<layer->parts.size(); n++)
                supportGenerator.polygons = supportGenerator.polygons.difference(layer->parts[n].outline.offset(config.supportXYDistance));
        }
        //Contract and expand the suppory polygons so small sections are removed and the final polygon is smoothed a bit.
        supportGenerator.polygons = supportGenerator.polygons.offset(-config.extrusionWidth * 3);
        supportGenerator.polygons = supportGenerator.polygons.offset(config.extrusionWidth * 3);
        
        vector<Polygons> supportIslands = supportGenerator.polygons.splitIntoParts();
        
        for(unsigned int n=0; n<supportIslands.size(); n++)
        {
            Polygons supportLines;
            if (config.supportLineDistance > 0)
            {
                if (config.supportLineDistance > config.extrusionWidth * 4)
                {
                    generateLineInfill(supportIslands[n], supportLines, config.extrusionWidth, config.supportLineDistance*2, config.infillOverlap, 0);
                    generateLineInfill(supportIslands[n], supportLines, config.extrusionWidth, config.supportLineDistance*2, config.infillOverlap, 90);
                }else{
                    generateLineInfill(supportIslands[n], supportLines, config.extrusionWidth, config.supportLineDistance, config.infillOverlap, (layerNr & 1) ? 0 : 90);
                }
            }
        
            gcodeLayer.forceRetract();
            if (config.enableCombing)
                gcodeLayer.setCombBoundary(&supportIslands[n]);
            gcodeLayer.addPolygonsByOptimizer(supportIslands[n], &supportConfig);
            gcodeLayer.addPolygonsByOptimizer(supportLines, &supportConfig);
            gcodeLayer.setCombBoundary(NULL);
        }
    }
    
    void addWipeTower(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int layerNr, int prevExtruder)
    {
        if (config.wipeTowerSize < 1)
            return;
        //If we changed extruder, print the wipe/prime tower for this nozzle;
        gcodeLayer.addPolygonsByOptimizer(storage.wipeTower, &supportConfig);
        Polygons fillPolygons;
        generateLineInfill(storage.wipeTower, fillPolygons, config.extrusionWidth, config.extrusionWidth, config.infillOverlap, 45 + 90 * (layerNr % 2));
        gcodeLayer.addPolygonsByOptimizer(fillPolygons, &supportConfig);
        
        //Make sure we wipe the old extruder on the wipe tower.
        gcodeLayer.addTravel(storage.wipePoint - config.extruderOffset[prevExtruder].p() + config.extruderOffset[gcodeLayer.getExtruder()].p());
    }
};

#endif//FFF_PROCESSOR_H
