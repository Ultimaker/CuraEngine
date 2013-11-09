/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <signal.h>
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
#include <execinfo.h>
#include <sys/resource.h>
#endif
#include <stddef.h>

#include "utils/gettime.h"
#include "utils/logoutput.h"
#include "sliceDataStorage.h"

#include "modelFile/modelFile.h"
#include "settings.h"
#include "optimizedModel.h"
#include "multiVolumes.h"
#include "polygonOptimizer.h"
#include "slicer.h"
#include "layerPart.h"
#include "inset.h"
#include "skin.h"
#include "infill.h"
#include "bridge.h"
#include "support.h"
#include "pathOrderOptimizer.h"
#include "skirt.h"
#include "raft.h"
#include "comb.h"
#include "gcodeExport.h"

#define VERSION "13.11"

int maxObjectHeight;

void processFile(const char* input_filename, ConfigSettings& config, GCodeExport& gcode, bool firstFile)
{
    for(unsigned int n=1; n<MAX_EXTRUDERS;n++)
        gcode.setExtruderOffset(n, config.extruderOffset[n].p());
    gcode.setFlavor(config.gcodeFlavor);
    
    double t = getTime();
    log("Loading %s from disk...\n", input_filename);
    SimpleModel* m = loadModel(input_filename, config.matrix);
    if (!m)
    {
        log("Failed to load model: %s\n", input_filename);
        return;
    }
    log("Loaded from disk in %5.3fs\n", timeElapsed(t));
    log("Analyzing and optimizing model...\n");
    OptimizedModel* om = new OptimizedModel(m, Point3(config.objectPosition.X, config.objectPosition.Y, -config.objectSink));
    for(unsigned int v = 0; v < m->volumes.size(); v++)
    {
        log("  Face counts: %i -> %i %0.1f%%\n", (int)m->volumes[v].faces.size(), (int)om->volumes[v].faces.size(), float(om->volumes[v].faces.size()) / float(m->volumes[v].faces.size()) * 100);
        log("  Vertex counts: %i -> %i %0.1f%%\n", (int)m->volumes[v].faces.size() * 3, (int)om->volumes[v].points.size(), float(om->volumes[v].points.size()) / float(m->volumes[v].faces.size() * 3) * 100);
    }
    delete m;
    log("Optimize model %5.3fs \n", timeElapsed(t));
    //om->saveDebugSTL("c:\\models\\output.stl");
    
    log("Slicing model...\n");
    vector<Slicer*> slicerList;
    for(unsigned int volumeIdx=0; volumeIdx < om->volumes.size(); volumeIdx++)
    {
        slicerList.push_back(new Slicer(&om->volumes[volumeIdx], config.initialLayerThickness / 2, config.layerThickness, config.fixHorrible & FIX_HORRIBLE_KEEP_NONE_CLOSED, config.fixHorrible & FIX_HORRIBLE_EXTENSIVE_STITCHING));
        //slicerList[volumeIdx]->dumpSegmentsToHTML("C:\\models\\output.html");
    }
    log("Sliced model in %5.3fs\n", timeElapsed(t));

    SliceDataStorage storage;
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
    //carveMultipleVolumes(storage.volumes);
    generateMultipleVolumesOverlap(storage.volumes, config.multiVolumeOverlap);
    log("Generated layer parts in %5.3fs\n", timeElapsed(t));
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
    log("Generated inset in %5.3fs\n", timeElapsed(t));

    for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
    {
        // disable infill when infill is set to 0 or spiralizeMode is on.
        // also omit the top skins when spiralize is on.
        if ((!config.spiralizeMode
             && ((config.sparseInfillLineDistance > 0) || (layerNr >= totalLayers - config.upSkinCount)))
            || (int(layerNr) < config.downSkinCount)
            ) {
            for(unsigned int volumeIdx=0; volumeIdx<storage.volumes.size(); volumeIdx++)
            {
                generateSkins(layerNr, storage.volumes[volumeIdx], config.extrusionWidth, config.downSkinCount, config.upSkinCount, config.infillOverlap);
                generateSparse(layerNr, storage.volumes[volumeIdx], config.extrusionWidth, config.downSkinCount, config.upSkinCount);
            }
        }
        logProgress("skin",layerNr+1,totalLayers);
    }
    log("Generated up/down skin in %5.3fs\n", timeElapsed(t));
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

    gcode.setRetractionSettings(config.retractionAmount, config.retractionSpeed, config.retractionAmountExtruderSwitch, config.minimalExtrusionBeforeRetraction);
    if (firstFile)
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
    gcode.addComment("total_layers=%d",totalLayers);

    GCodePathConfig skirtConfig(config.printSpeed, config.extrusionWidth, "SKIRT");
    GCodePathConfig inset0Config(config.printSpeed, config.extrusionWidth, "WALL-OUTER");
    GCodePathConfig inset1Config(config.printSpeed, config.extrusionWidth, "WALL-INNER");
    GCodePathConfig fillConfig(config.infillSpeed, config.extrusionWidth, "FILL");
    GCodePathConfig supportConfig(config.printSpeed, config.extrusionWidth, "SUPPORT");
    
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
        
        GCodePlanner gcodeLayer(gcode, config.moveSpeed, config.retractionMinimalDistance);
        gcode.addComment("LAYER:%d", layerNr);
        int32_t z = config.initialLayerThickness + layerNr * config.layerThickness;
        z += config.raftBaseThickness + config.raftInterfaceThickness;
        gcode.setZ(z);
        if (layerNr == 0)
            gcodeLayer.addPolygonsByOptimizer(storage.skirt, &skirtConfig);
        for(unsigned int volumeCnt = 0; volumeCnt < storage.volumes.size(); volumeCnt++)
        {
            if (volumeCnt > 0)
                volumeIdx = (volumeIdx + 1) % storage.volumes.size();
            SliceLayer* layer = &storage.volumes[volumeIdx].layers[layerNr];
            gcodeLayer.setExtruder(volumeIdx);
            
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
                
                //Force retraction on inter-island travel.
                if (partOrderOptimizer.polyOrder.size() > 1)
                    gcodeLayer.forceRetract();
                
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
                if (layerNr & 1) fillAngle += 90;
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
        if (storage.support.generated)
        {
            if (config.supportExtruder > -1)
                gcodeLayer.setExtruder(config.supportExtruder);
            SupportPolyGenerator supportGenerator(storage.support, z);
            for(unsigned int volumeCnt = 0; volumeCnt < storage.volumes.size(); volumeCnt++)
            {
                SliceLayer* layer = &storage.volumes[volumeIdx].layers[layerNr];
                Polygons polys;
                for(unsigned int n=0; n<layer->parts.size(); n++)
                    supportGenerator.polygons = supportGenerator.polygons.difference(layer->parts[n].outline.offset(config.supportXYDistance));
            }
            //Contract and expand the suppory polygons so small sections are removed and the final polygon is smoothed a bit.
            supportGenerator.polygons = supportGenerator.polygons.offset(-1000);
            supportGenerator.polygons = supportGenerator.polygons.offset(1000);
            
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
            
                gcodeLayer.addPolygonsByOptimizer(supportIslands[n], &supportConfig);
                gcodeLayer.addPolygonsByOptimizer(supportLines, &supportConfig);
            }
        }

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
        if (layerNr == 0)
            gcode.setExtrusion(config.initialLayerThickness, config.filamentDiameter, config.filamentFlow);
        else
            gcode.setExtrusion(config.layerThickness, config.filamentDiameter, config.filamentFlow);

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
    
    log("Wrote layers in %5.2fs.\n", timeElapsed(t));
    gcode.tellFileSize();
    gcode.addFanCommand(0);

    logProgress("process", 1, 1);
    log("Total time elapsed %5.2fs.\n", timeElapsed(t,true));
    
    //Store the object height for when we are printing multiple objects, as we need to clear every one of them when moving to the next position.
    maxObjectHeight = std::max(maxObjectHeight, storage.modelSize.z);
}

void print_usage()
{
    printf("usage: CuraEngine [-h] [-v] [-m 3x3matrix] [-s <settingkey>=<value>] -o <output.gcode> <model.stl>\n");
}

void signal_FPE(int n)
{
    (void)n;
    printf("Arithmetic exception.\n");
    exit(1);
}

int main(int argc, char **argv)
{
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
    //Lower the process priority on linux and mac.
    setpriority(PRIO_PROCESS, 0, 10);
#endif
    signal(SIGFPE, signal_FPE);

    GCodeExport gcode;
    ConfigSettings config;
    int fileNr = 0;

    config.filamentDiameter = 2890;
    config.filamentFlow = 100;
    config.initialLayerThickness = 300;
    config.layerThickness = 100;
    config.extrusionWidth = 400;
    config.insetCount = 2;
    config.downSkinCount = 6;
    config.upSkinCount = 6;
    config.initialSpeedupLayers = 4;
    config.initialLayerSpeed = 20;
    config.printSpeed = 50;
    config.infillSpeed = 50;
    config.moveSpeed = 200;
    config.fanFullOnLayerNr = 2;
    config.skirtDistance = 6000;
    config.skirtLineCount = 1;
    config.skirtMinLength = 0;
    config.sparseInfillLineDistance = 100 * config.extrusionWidth / 20;
    config.infillOverlap = 15;
    config.objectPosition.X = 102500;
    config.objectPosition.Y = 102500;
    config.objectSink = 0;
    config.supportAngle = -1;
    config.supportEverywhere = 0;
    config.supportLineDistance = config.sparseInfillLineDistance;
    config.supportExtruder = -1;
    config.supportXYDistance = 700;
    config.supportZDistance = 150;
    config.retractionAmount = 4500;
    config.retractionSpeed = 45;
    config.retractionAmountExtruderSwitch = 14500;
    config.retractionMinimalDistance = 1500;
    config.minimalExtrusionBeforeRetraction = 100;
    config.enableCombing = 1;
    config.multiVolumeOverlap = 0;

    config.minimalLayerTime = 5;
    config.minimalFeedrate = 10;
    config.coolHeadLift = 1;
    config.fanSpeedMin = 100;
    config.fanSpeedMax = 100;

    config.raftMargin = 5000;
    config.raftLineSpacing = 1000;
    config.raftBaseThickness = 0;
    config.raftBaseLinewidth = 0;
    config.raftInterfaceThickness = 0;
    config.raftInterfaceLinewidth = 0;

    config.spiralizeMode = 0;
    config.fixHorrible = 0;
    config.gcodeFlavor = GCODE_FLAVOR_REPRAP;
    
    config.startCode =
        "M109 S210     ;Heatup to 210C\n"
        "G21           ;metric values\n"
        "G90           ;absolute positioning\n"
        "G28           ;Home\n"
        "G1 Z15.0 F300 ;move the platform down 15mm\n"
        "G92 E0        ;zero the extruded length\n"
        "G1 F200 E5    ;extrude 5mm of feed stock\n"
        "G92 E0        ;zero the extruded length again\n";
    config.endCode = 
        "M104 S0                     ;extruder heater off\n"
        "M140 S0                     ;heated bed heater off (if you have it)\n"
        "G91                            ;relative positioning\n"
        "G1 E-1 F300                    ;retract the filament a bit before lifting the nozzle, to release some of the pressure\n"
        "G1 Z+0.5 E-5 X-20 Y-20 F9000   ;move Z up a bit and retract filament even more\n"
        "G28 X0 Y0                      ;move X/Y to min endstops, so the head is out of the way\n"
        "M84                         ;steppers off\n"
        "G90                         ;absolute positioning\n";

    fprintf(stderr,"Cura_SteamEngine version %s\n", VERSION);

    for(int argn = 1; argn < argc; argn++)
    {
        char* str = argv[argn];
        if (str[0] == '-')
        {
            for(str++; *str; str++)
            {
                switch(*str)
                {
                case 'h':
                    print_usage();
                    exit(1);
                case 'v':
                    verbose_level++;
                    break;
                case 'b':
                    argn++;
                    binaryMeshBlob = fopen(argv[argn], "rb");
                    break;
                case 'o':
                    argn++;
                    gcode.setFilename(argv[argn]);
                    if (!gcode.isValid())
                    {
                        logError("Failed to open %s for output.\n", argv[argn]);
                        exit(1);
                    }
                    break;
                case 's':
                    {
                        argn++;
                        char* valuePtr = strchr(argv[argn], '=');
                        if (valuePtr)
                        {
                            *valuePtr++ = '\0';
                            
                            if (!config.setSetting(argv[argn], valuePtr))
                                printf("Setting found: %s %s\n", argv[argn], valuePtr);
                        }
                    }
                    break;
                case 'm':
                    argn++;
                    sscanf(argv[argn], "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                        &config.matrix.m[0][0], &config.matrix.m[0][1], &config.matrix.m[0][2],
                        &config.matrix.m[1][0], &config.matrix.m[1][1], &config.matrix.m[1][2],
                        &config.matrix.m[2][0], &config.matrix.m[2][1], &config.matrix.m[2][2]);
                    break;
                default:
                    logError("Unknown option: %c\n", *str);
                    break;
                }
            }
        }else{
            if (!gcode.isValid())
            {
                logError("No output file specified\n");
                return 1;
            }
			gcode.addComment("Generated with Cura_SteamEngine %s", VERSION);
            processFile(argv[argn], config, gcode, fileNr == 0);
            fileNr ++;
        }
    }
    if (gcode.isValid())
    {
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
}
