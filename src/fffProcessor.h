#ifndef FFF_PROCESSOR_H
#define FFF_PROCESSOR_H

#include <algorithm>
#include <vector>
#include "utils/socket.h"

#define GUI_CMD_REQUEST_MESH 0x01
#define GUI_CMD_SEND_POLYGONS 0x02
#define GUI_CMD_FINISH_OBJECT 0x03

namespace cura {

//FusedFilamentFabrication processor.
class fffProcessor
{
private:
    int maxObjectHeight;
    int fileNr;
    GCodeExport gcode;
    ConfigSettings& config;
    TimeKeeper timeKeeper;
    ClientSocket guiSocket;

    GCodePathConfig skirtConfig;
    GCodePathConfig inset0Config;
    GCodePathConfig insetXConfig;
    GCodePathConfig fillConfig;
    GCodePathConfig supportConfig;
public:
    fffProcessor(ConfigSettings& config)
    : config(config)
    {
        fileNr = 1;
        maxObjectHeight = 0;
    }

    void guiConnect(int portNr)
    {
        guiSocket.connectTo("127.0.0.1", portNr);
    }

    void sendPolygonsToGui(const char* name, int layerNr, int32_t z, Polygons& polygons)
    {
        guiSocket.sendNr(GUI_CMD_SEND_POLYGONS);
        guiSocket.sendNr(polygons.size());
        guiSocket.sendNr(layerNr);
        guiSocket.sendNr(z);
        guiSocket.sendNr(strlen(name));
        guiSocket.sendAll(name, strlen(name));
        for(unsigned int n=0; n<polygons.size(); n++)
        {
            PolygonRef polygon = polygons[n];
            guiSocket.sendNr(polygon.size());
            guiSocket.sendAll(polygon.data(), polygon.size() * sizeof(Point));
        }
    }

    bool setTargetFile(const char* filename)
    {
        gcode.setFilename(filename);
        if (gcode.isOpened())
            gcode.writeComment("Generated with Cura_SteamEngine %s", VERSION);
        return gcode.isOpened();
    }

    bool processFile(const std::vector<std::string> &files)
    {
        if (!gcode.isOpened())
            return false;

        TimeKeeper timeKeeperTotal;
        SliceDataStorage storage;
        preSetup();
        if (!prepareModel(storage, files))
            return false;

        processSliceData(storage);
        writeGCode(storage);

        cura::logProgress("process", 1, 1);//Report the GUI that a file has been fully processed.
        cura::log("Total time elapsed %5.2fs.\n", timeKeeperTotal.restart());
        guiSocket.sendNr(GUI_CMD_FINISH_OBJECT);

        return true;
    }

    void finalize()
    {
        if (!gcode.isOpened())
            return;
        gcode.finalize(maxObjectHeight, config.moveSpeed, config.endCode.c_str());
    }

private:
    void preSetup()
    {
        skirtConfig.setData(config.printSpeed, config.extrusionWidth, "SKIRT");
        inset0Config.setData(config.inset0Speed, config.extrusionWidth, "WALL-OUTER");
        insetXConfig.setData(config.insetXSpeed, config.extrusionWidth, "WALL-INNER");
        fillConfig.setData(config.infillSpeed, config.extrusionWidth, "FILL");
        supportConfig.setData(config.printSpeed, config.extrusionWidth, "SUPPORT");

        for(unsigned int n=1; n<MAX_EXTRUDERS;n++)
            gcode.setExtruderOffset(n, config.extruderOffset[n].p());
        gcode.setSwitchExtruderCode(config.preSwitchExtruderCode, config.postSwitchExtruderCode);
        gcode.setFlavor(config.gcodeFlavor);
        gcode.setRetractionSettings(config.retractionAmount, config.retractionSpeed, config.retractionAmountExtruderSwitch, config.minimalExtrusionBeforeRetraction, config.retractionZHop, config.retractionAmountPrime);
    }

    bool prepareModel(SliceDataStorage& storage, const std::vector<std::string> &files)
    {
        timeKeeper.restart();
        SimpleModel* model = nullptr;
        if (files.size() == 1 && files[0][0] == '$')
        {
            const char *input_filename = files[0].c_str();
            model = new SimpleModel();
            for(unsigned int n=0; input_filename[n]; n++)
            {
                model->volumes.push_back(SimpleVolume());
                SimpleVolume* volume = &model->volumes[model->volumes.size()-1];
                guiSocket.sendNr(GUI_CMD_REQUEST_MESH);

                int32_t vertexCount = guiSocket.recvNr();
                int pNr = 0;
                cura::log("Reading mesh from socket with %i vertexes\n", vertexCount);
                Point3 v[3];
                while(vertexCount)
                {
                    float f[3];
                    guiSocket.recvAll(f, 3 * sizeof(float));
                    FPoint3 fp(f[0], f[1], f[2]);
                    v[pNr++] = config.matrix.apply(fp);
                    if (pNr == 3)
                    {
                        volume->addFace(v[0], v[1], v[2]);
                        pNr = 0;
                    }
                    vertexCount--;
                }
            }
        }else{
            model = new SimpleModel();
            for(unsigned int i=0;i < files.size(); i++) {
                if(files[i] == "-")
                    model->volumes.push_back(SimpleVolume());
                else {
                    cura::log("Loading %s from disk...\n", files[i].c_str());
                    SimpleModel *test = loadModelFromFile(model,files[i].c_str(), config.matrix);
                    if(test == nullptr) { // error while reading occurred
                        cura::logError("Failed to load model: %s\n", files[i].c_str());
                        return false;
                    }
                }
            }
        }
        cura::log("Loaded from disk in %5.3fs\n", timeKeeper.restart());
        cura::log("Analyzing and optimizing model...\n");
        OptimizedModel* optimizedModel = new OptimizedModel(model, Point3(config.objectPosition.X, config.objectPosition.Y, -config.objectSink));
        for(unsigned int v = 0; v < model->volumes.size(); v++)
        {
            cura::log("  Face counts: %i -> %i %0.1f%%\n", (int)model->volumes[v].faces.size(), (int)optimizedModel->volumes[v].faces.size(), float(optimizedModel->volumes[v].faces.size()) / float(model->volumes[v].faces.size()) * 100);
            cura::log("  Vertex counts: %i -> %i %0.1f%%\n", (int)model->volumes[v].faces.size() * 3, (int)optimizedModel->volumes[v].points.size(), float(optimizedModel->volumes[v].points.size()) / float(model->volumes[v].faces.size() * 3) * 100);
            cura::log("  Size: %f %f %f\n", INT2MM(optimizedModel->modelSize.x), INT2MM(optimizedModel->modelSize.y), INT2MM(optimizedModel->modelSize.z));
            cura::log("  vMin: %f %f %f\n", INT2MM(optimizedModel->vMin.x), INT2MM(optimizedModel->vMin.y), INT2MM(optimizedModel->vMin.z));
            cura::log("  vMax: %f %f %f\n", INT2MM(optimizedModel->vMax.x), INT2MM(optimizedModel->vMax.y), INT2MM(optimizedModel->vMax.z));
            cura::log("  vMin: %f %f %f\n", INT2MM(model->min().x), INT2MM(model->min().y), INT2MM(model->min().z));
            cura::log("  vMax: %f %f %f\n", INT2MM(model->max().x), INT2MM(model->max().y), INT2MM(model->max().z));
            cura::log("  Matrix: %f %f %f\n", config.matrix.m[0][0], config.matrix.m[1][0], config.matrix.m[2][0]);
            cura::log("  Matrix: %f %f %f\n", config.matrix.m[0][1], config.matrix.m[1][1], config.matrix.m[2][1]);
            cura::log("  Matrix: %f %f %f\n", config.matrix.m[0][2], config.matrix.m[1][2], config.matrix.m[2][2]);
            if (INT2MM(optimizedModel->modelSize.x) > 10000.0 || INT2MM(optimizedModel->modelSize.y)  > 10000.0 || INT2MM(optimizedModel->modelSize.z) > 10000.0)
            {
                cura::logError("Object is way to big, CuraEngine bug?");
                exit(1);
            }
        }
        delete model;
        cura::log("Optimize model %5.3fs \n", timeKeeper.restart());
        //om->saveDebugSTL("c:\\models\\output.stl");

        cura::log("Slicing model...\n");
        vector<Slicer*> slicerList;
        for(unsigned int volumeIdx=0; volumeIdx < optimizedModel->volumes.size(); volumeIdx++)
        {
            Slicer* slicer = new Slicer(&optimizedModel->volumes[volumeIdx], config.initialLayerThickness - config.layerThickness / 2, config.layerThickness, config.fixHorrible & FIX_HORRIBLE_KEEP_NONE_CLOSED, config.fixHorrible & FIX_HORRIBLE_EXTENSIVE_STITCHING);
            slicerList.push_back(slicer);
            for(unsigned int layerNr=0; layerNr<slicer->layers.size(); layerNr++)
            {
                //Reporting the outline here slows down the engine quite a bit, so only do so when debugging.
                //sendPolygonsToGui("outline", layerNr, slicer->layers[layerNr].z, slicer->layers[layerNr].polygonList);
                sendPolygonsToGui("openoutline", layerNr, slicer->layers[layerNr].z, slicer->layers[layerNr].openPolygonList);
            }
        }
        cura::log("Sliced model in %5.3fs\n", timeKeeper.restart());

        cura::log("Generating support map...\n");
        generateSupportGrid(storage.support, optimizedModel, config.supportAngle, config.supportEverywhere > 0, config.supportXYDistance, config.supportZDistance);

        storage.modelSize = optimizedModel->modelSize;
        storage.modelMin = optimizedModel->vMin;
        storage.modelMax = optimizedModel->vMax;
        delete optimizedModel;

        cura::log("Generating layer parts...\n");
        for(unsigned int volumeIdx=0; volumeIdx < slicerList.size(); volumeIdx++)
        {
            storage.volumes.push_back(SliceVolumeStorage());
            createLayerParts(storage.volumes[volumeIdx], slicerList[volumeIdx], config.fixHorrible & (FIX_HORRIBLE_UNION_ALL_TYPE_A | FIX_HORRIBLE_UNION_ALL_TYPE_B | FIX_HORRIBLE_UNION_ALL_TYPE_C));
            delete slicerList[volumeIdx];

            //Add the raft offset to each layer.
            for(unsigned int layerNr=0; layerNr<storage.volumes[volumeIdx].layers.size(); layerNr++)
                storage.volumes[volumeIdx].layers[layerNr].printZ += config.raftBaseThickness + config.raftInterfaceThickness;
        }
        cura::log("Generated layer parts in %5.3fs\n", timeKeeper.restart());
        return true;
    }

    void processSliceData(SliceDataStorage& storage)
    {
        const unsigned int totalLayers = storage.volumes[0].layers.size();
        
        //carveMultipleVolumes(storage.volumes);
        generateMultipleVolumesOverlap(storage.volumes, config.multiVolumeOverlap);
        //dumpLayerparts(storage, "c:/models/output.html");
        if (config.simpleMode)
        {
            for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
            {
                for(unsigned int volumeIdx=0; volumeIdx<storage.volumes.size(); volumeIdx++)
                {
                    SliceLayer* layer = &storage.volumes[volumeIdx].layers[layerNr];
                    for(unsigned int partNr=0; partNr<layer->parts.size(); partNr++)
                    {
                        sendPolygonsToGui("inset0", layerNr, layer->printZ, layer->parts[partNr].outline);
                    }
                }
            }
            return;
        }

        for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
        {
            for(unsigned int volumeIdx=0; volumeIdx<storage.volumes.size(); volumeIdx++)
            {
                int insetCount = config.insetCount;
                if (config.spiralizeMode && static_cast<int>(layerNr) < config.downSkinCount && layerNr % 2 == 1)//Add extra insets every 2 layers when spiralizing, this makes bottoms of cups watertight.
                    insetCount += 5;
                SliceLayer* layer = &storage.volumes[volumeIdx].layers[layerNr];
                int extrusionWidth = config.extrusionWidth;
                if (layerNr == 0)
                    extrusionWidth = config.layer0extrusionWidth;
                generateInsets(layer, extrusionWidth, insetCount);

                for(unsigned int partNr=0; partNr<layer->parts.size(); partNr++)
                {
                    if (layer->parts[partNr].insets.size() > 0)
                    {
                        sendPolygonsToGui("inset0", layerNr, layer->printZ, layer->parts[partNr].insets[0]);
                        for(unsigned int inset=1; inset<layer->parts[partNr].insets.size(); inset++)
                            sendPolygonsToGui("insetx", layerNr, layer->printZ, layer->parts[partNr].insets[inset]);
                    }
                }
            }
            cura::logProgress("inset",layerNr+1,totalLayers);
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
                        oozeShield = oozeShield.unionPolygons(storage.volumes[volumeIdx].layers[layerNr].parts[partNr].outline.offset(MM2INT(2.0)));
                    }
                }
                storage.oozeShield.push_back(oozeShield);
            }

            for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
                storage.oozeShield[layerNr] = storage.oozeShield[layerNr].offset(-MM2INT(1.0)).offset(MM2INT(1.0));
            int offsetAngle = tan(60.0*M_PI/180) * config.layerThickness;//Allow for a 60deg angle in the oozeShield.
            for(unsigned int layerNr=1; layerNr<totalLayers; layerNr++)
                storage.oozeShield[layerNr] = storage.oozeShield[layerNr].unionPolygons(storage.oozeShield[layerNr-1].offset(-offsetAngle));
            for(unsigned int layerNr=totalLayers-1; layerNr>0; layerNr--)
                storage.oozeShield[layerNr-1] = storage.oozeShield[layerNr-1].unionPolygons(storage.oozeShield[layerNr].offset(-offsetAngle));
        }
        cura::log("Generated inset in %5.3fs\n", timeKeeper.restart());

        for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
        {
            if (!config.spiralizeMode || static_cast<int>(layerNr) < config.downSkinCount)    //Only generate up/downskin and infill for the first X layers when spiralize is choosen.
            {
                for(unsigned int volumeIdx=0; volumeIdx<storage.volumes.size(); volumeIdx++)
                {
                    int extrusionWidth = config.extrusionWidth;
                    if (layerNr == 0)
                        extrusionWidth = config.layer0extrusionWidth;
                    generateSkins(layerNr, storage.volumes[volumeIdx], extrusionWidth, config.downSkinCount, config.upSkinCount, config.infillOverlap);
                    generateSparse(layerNr, storage.volumes[volumeIdx], extrusionWidth, config.downSkinCount, config.upSkinCount);

                    SliceLayer* layer = &storage.volumes[volumeIdx].layers[layerNr];
                    for(unsigned int partNr=0; partNr<layer->parts.size(); partNr++)
                        sendPolygonsToGui("skin", layerNr, layer->printZ, layer->parts[partNr].skinOutline);
                }
            }
            cura::logProgress("skin",layerNr+1,totalLayers);
        }
        cura::log("Generated up/down skin in %5.3fs\n", timeKeeper.restart());

        if (config.wipeTowerSize > 0)
        {
            PolygonRef p = storage.wipeTower.newPoly();
            p.add(Point(storage.modelMin.x - 3000, storage.modelMax.y + 3000));
            p.add(Point(storage.modelMin.x - 3000, storage.modelMax.y + 3000 + config.wipeTowerSize));
            p.add(Point(storage.modelMin.x - 3000 - config.wipeTowerSize, storage.modelMax.y + 3000 + config.wipeTowerSize));
            p.add(Point(storage.modelMin.x - 3000 - config.wipeTowerSize, storage.modelMax.y + 3000));

            storage.wipePoint = Point(storage.modelMin.x - 3000 - config.wipeTowerSize / 2, storage.modelMax.y + 3000 + config.wipeTowerSize / 2);
        }

        generateSkirt(storage, config.skirtDistance, config.layer0extrusionWidth, config.skirtLineCount, config.skirtMinLength, config.initialLayerThickness);
        generateRaft(storage, config.raftMargin);

        sendPolygonsToGui("skirt", 0, config.initialLayerThickness, storage.skirt);
    }

    void writeGCode(SliceDataStorage& storage)
    {
        if (fileNr == 1)
        {
            if (gcode.getFlavor() == GCODE_FLAVOR_ULTIGCODE)
            {
                gcode.writeComment("FLAVOR:UltiGCode");
                gcode.writeComment("TIME:<__TIME__>");
                gcode.writeComment("MATERIAL:<FILAMENT>");
                gcode.writeComment("MATERIAL2:<FILAMEN2>");
            }
            gcode.writeCode(config.startCode.c_str());
            if (gcode.getFlavor() == GCODE_FLAVOR_BFB)
            {
                gcode.writeComment("enable auto-retraction");
                gcode.writeLine("M227 S%d P%d", config.retractionAmount * 2560 / 1000, config.retractionAmount * 2560 / 1000);
            }
        }else{
            gcode.writeFanCommand(0);
            gcode.resetExtrusionValue();
            gcode.writeRetraction();
            gcode.setZ(maxObjectHeight + 5000);
            gcode.writeMove(gcode.getPositionXY(), config.moveSpeed, 0);
            gcode.writeMove(Point(storage.modelMin.x, storage.modelMin.y), config.moveSpeed, 0);
        }
        fileNr++;

        unsigned int totalLayers = storage.volumes[0].layers.size();
        gcode.writeComment("Layer count: %d", totalLayers);

        if (config.raftBaseThickness > 0 && config.raftInterfaceThickness > 0)
        {
            sendPolygonsToGui("support", 0, config.raftBaseThickness, storage.raftOutline);
            sendPolygonsToGui("support", 0, config.raftBaseThickness + config.raftInterfaceThickness, storage.raftOutline);

            GCodePathConfig raftBaseConfig((config.raftBaseSpeed <= 0) ? config.initialLayerSpeed : config.raftBaseSpeed, config.raftBaseLinewidth, "SUPPORT");
            GCodePathConfig raftMiddleConfig(config.printSpeed, config.raftInterfaceLinewidth, "SUPPORT");
            GCodePathConfig raftInterfaceConfig(config.printSpeed, config.raftInterfaceLinewidth, "SUPPORT");
            GCodePathConfig raftSurfaceConfig((config.raftSurfaceSpeed > 0) ? config.raftSurfaceSpeed : config.printSpeed, config.raftSurfaceLinewidth, "SUPPORT");

            {
                gcode.writeComment("LAYER:-2");
                gcode.writeComment("RAFT");
                GCodePlanner gcodeLayer(gcode, config.moveSpeed, config.retractionMinimalDistance);
                if (config.supportExtruder > 0)
                    gcodeLayer.setExtruder(config.supportExtruder);
                gcode.setZ(config.raftBaseThickness);
                gcode.setExtrusion(config.raftBaseThickness, config.filamentDiameter, config.filamentFlow);
                gcodeLayer.addPolygonsByOptimizer(storage.raftOutline, &raftBaseConfig);

                Polygons raftLines;
                generateLineInfill(storage.raftOutline, raftLines, config.raftBaseLinewidth, config.raftLineSpacing, config.infillOverlap, 0);
                gcodeLayer.addPolygonsByOptimizer(raftLines, &raftBaseConfig);

                gcodeLayer.writeGCode(false, config.raftBaseThickness);
            }

            if (config.raftFanSpeed) {
                gcode.writeFanCommand(config.raftFanSpeed);
            }

            {
                gcode.writeComment("LAYER:-1");
                gcode.writeComment("RAFT");
                GCodePlanner gcodeLayer(gcode, config.moveSpeed, config.retractionMinimalDistance);
                gcode.setZ(config.raftBaseThickness + config.raftInterfaceThickness);
                gcode.setExtrusion(config.raftInterfaceThickness, config.filamentDiameter, config.filamentFlow);

                Polygons raftLines;
                generateLineInfill(storage.raftOutline, raftLines, config.raftInterfaceLinewidth, config.raftInterfaceLineSpacing, config.infillOverlap, config.raftSurfaceLayers > 0 ? 45 : 90);
                gcodeLayer.addPolygonsByOptimizer(raftLines, &raftInterfaceConfig);

                gcodeLayer.writeGCode(false, config.raftInterfaceThickness);
            }

            for (int raftSurfaceLayer=1; raftSurfaceLayer<=config.raftSurfaceLayers; raftSurfaceLayer++)
            {
                gcode.writeComment("LAYER:-1");
                gcode.writeComment("RAFT");
                GCodePlanner gcodeLayer(gcode, config.moveSpeed, config.retractionMinimalDistance);
                gcode.setZ(config.raftBaseThickness + config.raftInterfaceThickness + config.raftSurfaceThickness*raftSurfaceLayer);
                gcode.setExtrusion(config.raftSurfaceThickness, config.filamentDiameter, config.filamentFlow);

                Polygons raftLines;
                generateLineInfill(storage.raftOutline, raftLines, config.raftSurfaceLinewidth, config.raftSurfaceLineSpacing, config.infillOverlap, 90 * raftSurfaceLayer);
                gcodeLayer.addPolygonsByOptimizer(raftLines, &raftSurfaceConfig);

                gcodeLayer.writeGCode(false, config.raftInterfaceThickness);
            }
        }

        int volumeIdx = 0;
        for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
        {
            cura::logProgress("export", layerNr+1, totalLayers);

            int extrusionWidth = config.extrusionWidth;
            if (layerNr == 0)
                extrusionWidth = config.layer0extrusionWidth;
            if (static_cast<int>(layerNr) < config.initialSpeedupLayers)
            {
                int n = config.initialSpeedupLayers;
#define SPEED_SMOOTH(speed) \
                std::min<int>((speed), (((speed)*layerNr)/n + (config.initialLayerSpeed*(n-layerNr)/n)))
                skirtConfig.setData(SPEED_SMOOTH(config.printSpeed), extrusionWidth, "SKIRT");
                inset0Config.setData(SPEED_SMOOTH(config.inset0Speed), extrusionWidth, "WALL-OUTER");
                insetXConfig.setData(SPEED_SMOOTH(config.insetXSpeed), extrusionWidth, "WALL-INNER");
                fillConfig.setData(SPEED_SMOOTH(config.infillSpeed), extrusionWidth,  "FILL");
                supportConfig.setData(SPEED_SMOOTH(config.printSpeed), extrusionWidth, "SUPPORT");
#undef SPEED_SMOOTH
            }else{
                skirtConfig.setData(config.printSpeed, extrusionWidth, "SKIRT");
                inset0Config.setData(config.inset0Speed, extrusionWidth, "WALL-OUTER");
                insetXConfig.setData(config.insetXSpeed, extrusionWidth, "WALL-INNER");
                fillConfig.setData(config.infillSpeed, extrusionWidth, "FILL");
                supportConfig.setData(config.printSpeed, extrusionWidth, "SUPPORT");
            }

            gcode.writeComment("LAYER:%d", layerNr);
            if (layerNr == 0)
                gcode.setExtrusion(config.initialLayerThickness, config.filamentDiameter, config.filamentFlow);
            else
                gcode.setExtrusion(config.layerThickness, config.filamentDiameter, config.filamentFlow);

            GCodePlanner gcodeLayer(gcode, config.moveSpeed, config.retractionMinimalDistance);
            int32_t z = config.initialLayerThickness + layerNr * config.layerThickness;
            z += config.raftBaseThickness + config.raftInterfaceThickness + config.raftSurfaceLayers*config.raftSurfaceThickness;
            if (config.raftBaseThickness > 0 && config.raftInterfaceThickness > 0)
            {
                if (layerNr == 0)
                {
                    z += config.raftAirGapLayer0;
                } else {
                    z += config.raftAirGap;
                }
            }
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

            //Finish the layer by applying speed corrections for minimal layer times
            gcodeLayer.forceMinimalLayerTime(config.minimalLayerTime, config.minimalFeedrate);

            int fanSpeed = config.fanSpeedMin;
            if (gcodeLayer.getExtrudeSpeedFactor() <= 50)
            {
                fanSpeed = config.fanSpeedMax;
            }else{
                int n = gcodeLayer.getExtrudeSpeedFactor() - 50;
                fanSpeed = config.fanSpeedMin * n / 50 + config.fanSpeedMax * (50 - n) / 50;
            }
            if (static_cast<int>(layerNr) < config.fanFullOnLayerNr)
            {
                //Slow down the fan on the layers below the [fanFullOnLayerNr], where layer 0 is speed 0.
                fanSpeed = fanSpeed * layerNr / config.fanFullOnLayerNr;
            }
            gcode.writeFanCommand(fanSpeed);

            gcodeLayer.writeGCode(config.coolHeadLift > 0, static_cast<int>(layerNr) > 0 ? config.layerThickness : config.initialLayerThickness);
        }

        cura::log("Wrote layers in %5.2fs.\n", timeKeeper.restart());
        gcode.tellFileSize();
        gcode.writeFanCommand(0);

        //Store the object height for when we are printing multiple objects, as we need to clear every one of them when moving to the next position.
        maxObjectHeight = std::max(maxObjectHeight, storage.modelSize.z - config.objectSink);
    }

    //Add a single layer from a single mesh-volume to the GCode
    void addVolumeLayerToGCode(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int volumeIdx, int layerNr)
    {
        int prevExtruder = gcodeLayer.getExtruder();
        bool extruderChanged = gcodeLayer.setExtruder(volumeIdx);
        if (layerNr == 0 && volumeIdx == 0)
        {
            if (storage.skirt.size() > 0)
                gcodeLayer.addTravel(storage.skirt[storage.skirt.size()-1].closestPointTo(gcode.getPositionXY()));
            gcodeLayer.addPolygonsByOptimizer(storage.skirt, &skirtConfig);
        }

        SliceLayer* layer = &storage.volumes[volumeIdx].layers[layerNr];
        if (extruderChanged)
            addWipeTower(storage, gcodeLayer, layerNr, prevExtruder);

        if (storage.oozeShield.size() > 0 && storage.volumes.size() > 1)
        {
            gcodeLayer.setAlwaysRetract(true);
            gcodeLayer.addPolygonsByOptimizer(storage.oozeShield[layerNr], &skirtConfig);
            sendPolygonsToGui("oozeshield", layerNr, layer->printZ, storage.oozeShield[layerNr]);
            gcodeLayer.setAlwaysRetract(!config.enableCombing);
        }

        if (config.simpleMode)
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
            if (config.spiralizeMode)
                inset0Config.spiralize = true;
            
            gcodeLayer.addPolygonsByOptimizer(polygons, &inset0Config);
            return;
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
                    if (static_cast<int>(layerNr) >= config.downSkinCount)
                        inset0Config.spiralize = true;
                    if (static_cast<int>(layerNr) == config.downSkinCount && part->insets.size() > 0)
                        gcodeLayer.addPolygonsByOptimizer(part->insets[0], &insetXConfig);
                }
                for(int insetNr=part->insets.size()-1; insetNr>-1; insetNr--)
                {
                    if (insetNr == 0)
                        gcodeLayer.addPolygonsByOptimizer(part->insets[insetNr], &inset0Config);
                    else
                        gcodeLayer.addPolygonsByOptimizer(part->insets[insetNr], &insetXConfig);
                }
            }

            Polygons fillPolygons;
            int fillAngle = 45;
            if (layerNr & 1)
                fillAngle += 90;
            int extrusionWidth = config.extrusionWidth;
            if (layerNr == 0)
                extrusionWidth = config.layer0extrusionWidth;
            for(Polygons outline : part->skinOutline.splitIntoParts())
            {
                int bridge = -1;
                if (layerNr > 0)
                    bridge = bridgeAngle(outline, &storage.volumes[volumeIdx].layers[layerNr-1]);
                generateLineInfill(outline, fillPolygons, extrusionWidth, extrusionWidth, config.infillOverlap, (bridge > -1) ? bridge : fillAngle);
            }
            if (config.sparseInfillLineDistance > 0)
            {
                switch (config.infillPattern)
                {
                    case INFILL_AUTOMATIC:
                        generateAutomaticInfill(
                            part->sparseOutline, fillPolygons, extrusionWidth,
                            config.sparseInfillLineDistance,
                            config.infillOverlap, fillAngle);
                        break;

                    case INFILL_GRID:
                        generateGridInfill(part->sparseOutline, fillPolygons,
                                           extrusionWidth,
                                           config.sparseInfillLineDistance,
                                           config.infillOverlap, fillAngle);
                        break;

                    case INFILL_LINES:
                        generateLineInfill(part->sparseOutline, fillPolygons,
                                           extrusionWidth,
                                           config.sparseInfillLineDistance,
                                           config.infillOverlap, fillAngle);
                        break;

                    case INFILL_CONCENTRIC:
                        generateConcentricInfill(
                            part->sparseOutline, fillPolygons,
                            config.sparseInfillLineDistance);
                        break;
                }
            }

            gcodeLayer.addPolygonsByOptimizer(fillPolygons, &fillConfig);
            //sendPolygonsToGui("infill", layerNr, layer->z, fillPolygons);

            //After a layer part, make sure the nozzle is inside the comb boundary, so we do not retract on the perimeter.
            if (!config.spiralizeMode || static_cast<int>(layerNr) < config.downSkinCount)
                gcodeLayer.moveInsideCombBoundary(config.extrusionWidth * 2);
        }
        gcodeLayer.setCombBoundary(nullptr);
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
        sendPolygonsToGui("support", layerNr, z, supportGenerator.polygons);

        vector<Polygons> supportIslands = supportGenerator.polygons.splitIntoParts();

        PathOrderOptimizer islandOrderOptimizer(gcode.getPositionXY());
        for(unsigned int n=0; n<supportIslands.size(); n++)
        {
            islandOrderOptimizer.addPolygon(supportIslands[n][0]);
        }
        islandOrderOptimizer.optimize();

        for(unsigned int n=0; n<supportIslands.size(); n++)
        {
            Polygons& island = supportIslands[islandOrderOptimizer.polyOrder[n]];

            Polygons supportLines;
            if (config.supportLineDistance > 0)
            {
                switch(config.supportType)
                {
                case SUPPORT_TYPE_GRID:
                    if (config.supportLineDistance > config.extrusionWidth * 4)
                    {
                        generateLineInfill(island, supportLines, config.extrusionWidth, config.supportLineDistance*2, config.infillOverlap, 0);
                        generateLineInfill(island, supportLines, config.extrusionWidth, config.supportLineDistance*2, config.infillOverlap, 90);
                    }else{
                        generateLineInfill(island, supportLines, config.extrusionWidth, config.supportLineDistance, config.infillOverlap, (layerNr & 1) ? 0 : 90);
                    }
                    break;
                case SUPPORT_TYPE_LINES:
                    generateLineInfill(island, supportLines, config.extrusionWidth, config.supportLineDistance, config.infillOverlap, 0);
                    break;
                }
            }

            gcodeLayer.forceRetract();
            if (config.enableCombing)
                gcodeLayer.setCombBoundary(&island);
            if (config.supportType == SUPPORT_TYPE_GRID)
                gcodeLayer.addPolygonsByOptimizer(island, &supportConfig);
            gcodeLayer.addPolygonsByOptimizer(supportLines, &supportConfig);
            gcodeLayer.setCombBoundary(nullptr);
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

}//namespace cura

#endif//FFF_PROCESSOR_H
