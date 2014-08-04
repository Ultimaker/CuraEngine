#ifndef FFF_PROCESSOR_H
#define FFF_PROCESSOR_H

#include <algorithm>
#include "utils/gettime.h"
#include "utils/logoutput.h"
#include "sliceDataStorage.h"
#include "modelFile/modelFile.h"
#include "optimizedModel.h"
#include "slicer.h"
#include "support.h"
#include "multiVolumes.h"
#include "layerPart.h"
#include "inset.h"
#include "skirt.h"
#include "raft.h"
#include "skin.h"
#include "infill.h"
#include "bridge.h"
#include "pathOrderOptimizer.h"
#include "gcodeExport.h"
#include "commandSocket.h"

#define GUI_CMD_REQUEST_MESH 0x01
#define GUI_CMD_SEND_POLYGONS 0x02
#define GUI_CMD_FINISH_OBJECT 0x03

namespace cura {

//FusedFilamentFabrication processor.
class fffProcessor : public SettingsBase
{
private:
    int maxObjectHeight;
    int fileNr;
    GCodeExport gcode;
    TimeKeeper timeKeeper;
    CommandSocket* commandSocket;

    GCodePathConfig skirtConfig;
    GCodePathConfig inset0Config;
    GCodePathConfig insetXConfig;
    GCodePathConfig fillConfig[MAX_SPARSE_COMBINE];
    GCodePathConfig supportConfig;
public:
    fffProcessor()
    {
        fileNr = 1;
        maxObjectHeight = 0;
        commandSocket = NULL;
    }

    void setCommandSocket(CommandSocket* socket)
    {
        commandSocket = socket;
    }

    void sendPolygons(const char* name, int layerNr, Polygons& polygons)
    {
        if (commandSocket)
            commandSocket->sendPolygons(name, layerNr, polygons);
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
        timeKeeper.restart();
        PrintObject* model = nullptr;

        model = new PrintObject(this);
        for(unsigned int n=0; n<files.size(); n++)
        {
            log("Loading %s from disk...\n", files[n].c_str());
            //TODO: matrix from settings
            FMatrix3x3 matrix;
            if (!loadMeshFromFile(model, files[n].c_str(), matrix))
            {
                logError("Failed to load model: %s\n", files[0].c_str());
                return false;
            }
        }

        log("Loaded from disk in %5.3fs\n", timeKeeper.restart());
        return processModel(model);
    }

    bool processModel(PrintObject* model)
    {
        timeKeeper.restart();
        if (!model)
            return false;
        if (!gcode.isOpened())
            return false;

        TimeKeeper timeKeeperTotal;
        SliceDataStorage storage;
        preSetup();
        if (!prepareModel(storage, model))
            return false;

        processSliceData(storage);
        writeGCode(storage);

        logProgress("process", 1, 1);//Report the GUI that a file has been fully processed.
        log("Total time elapsed %5.2fs.\n", timeKeeperTotal.restart());

        return true;
    }
    
    void finalize()
    {
        if (!gcode.isOpened())
            return;
        gcode.finalize(maxObjectHeight, getSettingInt("moveSpeed"), getSetting("endCode").c_str());
    }
    
    double getTotalFilamentUsed(int e)
    {
        return gcode.getTotalFilamentUsed(e);
    }

    double getTotalPrintTime()
    {
        return gcode.getTotalPrintTime();
    }

private:
    void preSetup()
    {
        int extrusionWidth = getSettingInt("extrusionWidth");
        skirtConfig.setData(getSettingInt("initialLayerSpeed"), extrusionWidth, "SKIRT");
        inset0Config.setData(getSettingInt("inset0Speed"), extrusionWidth, "WALL-OUTER");
        insetXConfig.setData(getSettingInt("insetXSpeed"), extrusionWidth, "WALL-INNER");
        for(unsigned int idx=0; idx<MAX_SPARSE_COMBINE; idx++)
            fillConfig[idx].setData(getSettingInt("infillSpeed"), extrusionWidth * (idx + 1), "FILL");
        supportConfig.setData(getSettingInt("supportSpeed"), extrusionWidth, "SUPPORT");

        for(unsigned int n=1; n<MAX_EXTRUDERS;n++)
            gcode.setExtruderOffset(n, Point(getSettingInt("extruderOffset1.X"), getSettingInt("extruderOffset1.Y")));
        gcode.setSwitchExtruderCode(getSetting("preSwitchExtruderCode"), getSetting("postSwitchExtruderCode"));
        //TODO: GCode flavor
        gcode.setFlavor(GCODE_FLAVOR_REPRAP);
        gcode.setRetractionSettings(getSettingInt("retractionAmount"), getSettingInt("retractionSpeed"), getSettingInt("retractionAmountExtruderSwitch"), getSettingInt("minimalExtrusionBeforeRetraction"), getSettingInt("retractionZHop"), getSettingInt("retractionAmountPrime"));
    }

    bool prepareModel(SliceDataStorage& storage, PrintObject* object)
    {
        Point3 object_min = object->min();
        Point3 object_max = object->max();
        Point3 object_size = object_max - object_min;
        Point3 offset = Point3(-object_min.x - object_size.x / 2, -object_min.y - object_size.y / 2, -object_min.z);
        logError("%d %d %d\n", offset.x, offset.y, offset.z);
        offset.x += object->getSettingInt("position.X");
        offset.y += object->getSettingInt("position.Y");
        object->offset(offset);
        
        log("Slicing model...\n");
        std::vector<Slicer*> slicerList;
        for(Mesh& mesh : object->meshes)
        {
            Slicer* slicer = new Slicer(&mesh, mesh.getSettingInt("initialLayerThickness") - mesh.getSettingInt("layerThickness") / 2, mesh.getSettingInt("layerThickness"), mesh.getSettingInt("fixHorrible") & FIX_HORRIBLE_KEEP_NONE_CLOSED, mesh.getSettingInt("fixHorrible") & FIX_HORRIBLE_EXTENSIVE_STITCHING);
            slicerList.push_back(slicer);
            /*
            for(SlicerLayer& layer : slicer->layers)
            {
                //Reporting the outline here slows down the engine quite a bit, so only do so when debugging.
                //sendPolygons("outline", layerNr, layer.z, layer.polygonList);
                //sendPolygons("openoutline", layerNr, layer.openPolygonList);
            }
            */
        }
        log("Sliced model in %5.3fs\n", timeKeeper.restart());

        log("Generating support map...\n");
        generateSupportGrid(storage.support, object, object->getSettingInt("supportAngle"), object->getSettingInt("supportEverywhere") > 0, object->getSettingInt("supportXYDistance"), object->getSettingInt("supportZDistance"));

        storage.modelMin = object->min();
        storage.modelMax = object->max();
        storage.modelSize = storage.modelMax - storage.modelMin;
        object->clear();//Clear the mesh data, it is no longer needed after this point, and it saves a lot of memory.

        log("Generating layer parts...\n");
        for(unsigned int meshIdx=0; meshIdx < slicerList.size(); meshIdx++)
        {
            storage.meshes.emplace_back();
            SliceMeshStorage& meshStorage = storage.meshes[meshIdx];
            meshStorage.mesh = &object->meshes[meshIdx];
            createLayerParts(meshStorage, slicerList[meshIdx], meshStorage.mesh->getSettingInt("fixHorrible") & (FIX_HORRIBLE_UNION_ALL_TYPE_A | FIX_HORRIBLE_UNION_ALL_TYPE_B | FIX_HORRIBLE_UNION_ALL_TYPE_C));
            delete slicerList[meshIdx];

            //Add the raft offset to each layer.
            for(unsigned int layerNr=0; layerNr<meshStorage.layers.size(); layerNr++)
            {
                meshStorage.layers[layerNr].printZ += meshStorage.mesh->getSettingInt("raftBaseThickness") + meshStorage.mesh->getSettingInt("raftInterfaceThickness");

                if (commandSocket)
                    commandSocket->sendLayerInfo(layerNr, meshStorage.layers[layerNr].printZ, layerNr == 0 ? meshStorage.mesh->getSettingInt("initialLayerThickness") : meshStorage.mesh->getSettingInt("layerThickness"));
            }
        }
        log("Generated layer parts in %5.3fs\n", timeKeeper.restart());
        return true;
    }

    void processSliceData(SliceDataStorage& storage)
    {
        const unsigned int totalLayers = storage.meshes[0].layers.size();
        
        //carveMultipleVolumes(storage.meshes);
        generateMultipleVolumesOverlap(storage.meshes, getSettingInt("multiVolumeOverlap"));
        //dumpLayerparts(storage, "c:/models/output.html");
        if (getSettingInt("simpleMode"))
        {
            for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
            {
                for(SliceMeshStorage& mesh : storage.meshes)
                {
                    SliceLayer* layer = &mesh.layers[layerNr];
                    for(SliceLayerPart& part : layer->parts)
                    {
                        sendPolygons("inset0", layerNr, part.outline);
                    }
                }
            }
            return;
        }

        for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
        {
            for(SliceMeshStorage& mesh : storage.meshes)
            {
                int insetCount = mesh.mesh->getSettingInt("insetCount");
                if (mesh.mesh->getSettingInt("spiralizeMode") && static_cast<int>(layerNr) < mesh.mesh->getSettingInt("downSkinCount") && layerNr % 2 == 1)//Add extra insets every 2 layers when spiralizing, this makes bottoms of cups watertight.
                    insetCount += 5;
                SliceLayer* layer = &mesh.layers[layerNr];
                int extrusionWidth = mesh.mesh->getSettingInt("extrusionWidth");
                if (layerNr == 0)
                    extrusionWidth = mesh.mesh->getSettingInt("layer0extrusionWidth");
                generateInsets(layer, extrusionWidth, insetCount);

                for(unsigned int partNr=0; partNr<layer->parts.size(); partNr++)
                {
                    if (layer->parts[partNr].insets.size() > 0)
                    {
                        sendPolygons("inset0", layerNr, layer->parts[partNr].insets[0]);
                        for(unsigned int inset=1; inset<layer->parts[partNr].insets.size(); inset++)
                            sendPolygons("insetx", layerNr, layer->parts[partNr].insets[inset]);
                    }
                }
            }
            logProgress("inset",layerNr+1,totalLayers);
            if (commandSocket) commandSocket->sendProgress(1.0/3.0 * float(layerNr) / float(totalLayers));
        }
        if (getSettingInt("enableOozeShield"))
        {
            for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
            {
                Polygons oozeShield;
                for(SliceMeshStorage& mesh : storage.meshes)
                {
                    for(SliceLayerPart& part : mesh.layers[layerNr].parts)
                    {
                        oozeShield = oozeShield.unionPolygons(part.outline.offset(MM2INT(2.0)));
                    }
                }
                storage.oozeShield.push_back(oozeShield);
            }

            for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
                storage.oozeShield[layerNr] = storage.oozeShield[layerNr].offset(-MM2INT(1.0)).offset(MM2INT(1.0));
            int offsetAngle = tan(60.0*M_PI/180) * getSettingInt("layerThickness");//Allow for a 60deg angle in the oozeShield.
            for(unsigned int layerNr=1; layerNr<totalLayers; layerNr++)
                storage.oozeShield[layerNr] = storage.oozeShield[layerNr].unionPolygons(storage.oozeShield[layerNr-1].offset(-offsetAngle));
            for(unsigned int layerNr=totalLayers-1; layerNr>0; layerNr--)
                storage.oozeShield[layerNr-1] = storage.oozeShield[layerNr-1].unionPolygons(storage.oozeShield[layerNr].offset(-offsetAngle));
        }
        log("Generated inset in %5.3fs\n", timeKeeper.restart());

        for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
        {
            if (!getSettingInt("spiralizeMode") || static_cast<int>(layerNr) < getSettingInt("downSkinCount"))    //Only generate up/downskin and infill for the first X layers when spiralize is choosen.
            {
                for(SliceMeshStorage& mesh : storage.meshes)
                {
                    int extrusionWidth = mesh.mesh->getSettingInt("extrusionWidth");
                    if (layerNr == 0)
                        extrusionWidth = mesh.mesh->getSettingInt("layer0extrusionWidth");
                    generateSkins(layerNr, mesh, extrusionWidth, mesh.mesh->getSettingInt("downSkinCount"), mesh.mesh->getSettingInt("upSkinCount"), mesh.mesh->getSettingInt("infillOverlap"));
                    if (mesh.mesh->getSettingInt("sparseInfillLineDistance") > 0)
                        generateSparse(layerNr, mesh, extrusionWidth, mesh.mesh->getSettingInt("downSkinCount"), mesh.mesh->getSettingInt("upSkinCount"));

                    SliceLayer& layer = mesh.layers[layerNr];
                    for(SliceLayerPart& part : layer.parts)
                        sendPolygons("skin", layerNr, part.skinOutline);
                }
            }
            logProgress("skin",layerNr+1,totalLayers);
            if (commandSocket) commandSocket->sendProgress(1.0/3.0 + 1.0/3.0 * float(layerNr) / float(totalLayers));
        }
        for(unsigned int layerNr=totalLayers-1; layerNr>0; layerNr--)
        {
            for(SliceMeshStorage& mesh : storage.meshes)
                combineSparseLayers(layerNr, mesh, mesh.mesh->getSettingInt("sparseInfillCombineCount"));
        }
        log("Generated up/down skin in %5.3fs\n", timeKeeper.restart());

        if (getSettingInt("wipeTowerSize") > 0)
        {
            PolygonRef p = storage.wipeTower.newPoly();
            int tower_size = getSettingInt("wipeTowerSize");
            p.add(Point(storage.modelMin.x - 3000, storage.modelMax.y + 3000));
            p.add(Point(storage.modelMin.x - 3000, storage.modelMax.y + 3000 + tower_size));
            p.add(Point(storage.modelMin.x - 3000 - tower_size, storage.modelMax.y + 3000 + tower_size));
            p.add(Point(storage.modelMin.x - 3000 - tower_size, storage.modelMax.y + 3000));

            storage.wipePoint = Point(storage.modelMin.x - 3000 - tower_size / 2, storage.modelMax.y + 3000 + tower_size / 2);
        }

        generateSkirt(storage, getSettingInt("skirtDistance"), getSettingInt("layer0extrusionWidth"), getSettingInt("skirtLineCount"), getSettingInt("skirtMinLength"), getSettingInt("initialLayerThickness"));
        generateRaft(storage, getSettingInt("raftMargin"));

        sendPolygons("skirt", 0, storage.skirt);
    }

    void writeGCode(SliceDataStorage& storage)
    {
        if (fileNr == 1)
        {
            gcode.writeCode(getSetting("startCode").c_str());
            if (gcode.getFlavor() == GCODE_FLAVOR_BFB)
            {
                gcode.writeComment("enable auto-retraction");
                gcode.writeLine("M227 S%d P%d", getSettingInt("retractionAmount") * 2560 / 1000, getSettingInt("retractionAmount") * 2560 / 1000);
            }
        }else{
            gcode.writeFanCommand(0);
            gcode.resetExtrusionValue();
            gcode.writeRetraction();
            gcode.setZ(maxObjectHeight + 5000);
            gcode.writeMove(gcode.getPositionXY(), getSettingInt("moveSpeed"), 0);
            gcode.writeMove(Point(storage.modelMin.x, storage.modelMin.y), getSettingInt("moveSpeed"), 0);
        }
        fileNr++;

        unsigned int totalLayers = storage.meshes[0].layers.size();
        gcode.writeComment("Layer count: %d", totalLayers);

        if (getSettingInt("raftBaseThickness") > 0 && getSettingInt("raftInterfaceThickness") > 0)
        {
            GCodePathConfig raftBaseConfig(getSettingInt("raftBaseSpeed"), getSettingInt("raftBaseLinewidth"), "SUPPORT");
            GCodePathConfig raftMiddleConfig(getSettingInt("raftInterfaceSpeed"), getSettingInt("raftInterfaceLinewidth"), "SUPPORT");
            GCodePathConfig raftInterfaceConfig(getSettingInt("raftInterfaceSpeed"), getSettingInt("raftInterfaceLinewidth"), "SUPPORT");
            GCodePathConfig raftSurfaceConfig(getSettingInt("raftSurfaceSpeed"), getSettingInt("raftSurfaceLinewidth"), "SUPPORT");
            {
                gcode.writeComment("LAYER:-2");
                gcode.writeComment("RAFT");
                GCodePlanner gcodeLayer(gcode, getSettingInt("moveSpeed"), getSettingInt("retractionMinimalDistance"));
                if (getSettingInt("supportExtruder") > 0)
                    gcodeLayer.setExtruder(getSettingInt("supportExtruder"));
                gcode.setZ(getSettingInt("raftBaseThickness"));
                gcode.setExtrusion(getSettingInt("raftBaseThickness"), getSettingInt("filamentDiameter"), getSettingInt("filamentFlow"));
                gcodeLayer.addPolygonsByOptimizer(storage.raftOutline, &raftBaseConfig);

                Polygons raftLines;
                generateLineInfill(storage.raftOutline, raftLines, getSettingInt("raftBaseLinewidth"), getSettingInt("raftLineSpacing"), getSettingInt("infillOverlap"), 0);
                gcodeLayer.addPolygonsByOptimizer(raftLines, &raftBaseConfig);

                gcodeLayer.writeGCode(false, getSettingInt("raftBaseThickness"));
            }

            if (getSettingInt("raftFanSpeed"))
            {
                gcode.writeFanCommand(getSettingInt("raftFanSpeed"));
            }

            {
                gcode.writeComment("LAYER:-1");
                gcode.writeComment("RAFT");
                GCodePlanner gcodeLayer(gcode, getSettingInt("moveSpeed"), getSettingInt("retractionMinimalDistance"));
                gcode.setZ(getSettingInt("raftBaseThickness") + getSettingInt("raftInterfaceThickness"));
                gcode.setExtrusion(getSettingInt("raftInterfaceThickness"), getSettingInt("filamentDiameter"), getSettingInt("filamentFlow"));

                Polygons raftLines;
                generateLineInfill(storage.raftOutline, raftLines, getSettingInt("raftInterfaceLinewidth"), getSettingInt("raftInterfaceLineSpacing"), getSettingInt("infillOverlap"), getSettingInt("raftSurfaceLayers") > 0 ? 45 : 90);
                gcodeLayer.addPolygonsByOptimizer(raftLines, &raftInterfaceConfig);

                gcodeLayer.writeGCode(false, getSettingInt("raftInterfaceThickness"));
            }

            for (int raftSurfaceLayer=1; raftSurfaceLayer<=getSettingInt("raftSurfaceLayers"); raftSurfaceLayer++)
            {
                gcode.writeComment("LAYER:-1");
                gcode.writeComment("RAFT");
                GCodePlanner gcodeLayer(gcode, getSettingInt("moveSpeed"), getSettingInt("retractionMinimalDistance"));
                gcode.setZ(getSettingInt("raftBaseThickness") + getSettingInt("raftInterfaceThickness") + getSettingInt("raftSurfaceThickness")*raftSurfaceLayer);
                gcode.setExtrusion(getSettingInt("raftSurfaceThickness"), getSettingInt("filamentDiameter"), getSettingInt("filamentFlow"));

                Polygons raftLines;
                generateLineInfill(storage.raftOutline, raftLines, getSettingInt("raftSurfaceLinewidth"), getSettingInt("raftSurfaceLineSpacing"), getSettingInt("infillOverlap"), 90 * raftSurfaceLayer);
                gcodeLayer.addPolygonsByOptimizer(raftLines, &raftSurfaceConfig);

                gcodeLayer.writeGCode(false, getSettingInt("raftInterfaceThickness"));
            }
        }

        int meshIdx = 0;
        for(unsigned int layerNr=0; layerNr<totalLayers; layerNr++)
        {
            logProgress("export", layerNr+1, totalLayers);
            if (commandSocket) commandSocket->sendProgress(2.0/3.0 + 1.0/3.0 * float(layerNr) / float(totalLayers));

            int extrusionWidth = getSettingInt("extrusionWidth");
            if (layerNr == 0)
                extrusionWidth = getSettingInt("layer0extrusionWidth");
            if (static_cast<int>(layerNr) < getSettingInt("initialSpeedupLayers"))
            {
                int n = getSettingInt("initialSpeedupLayers");
                int initialLayerSpeed = getSettingInt("initialLayerSpeed");
#define SPEED_SMOOTH(speed) \
                std::min<int>((speed), (((speed)*layerNr)/n + (initialLayerSpeed*(n-layerNr)/n)))
                skirtConfig.setData(getSettingInt("skirtSpeed"), extrusionWidth, "SKIRT");
                inset0Config.setData(SPEED_SMOOTH(getSettingInt("inset0Speed")), extrusionWidth, "WALL-OUTER");
                insetXConfig.setData(SPEED_SMOOTH(getSettingInt("insetXSpeed")), extrusionWidth, "WALL-INNER");
                for(unsigned int idx=0; idx<MAX_SPARSE_COMBINE; idx++)
                    fillConfig[idx].setData(SPEED_SMOOTH(getSettingInt("infillSpeed")), extrusionWidth * (idx + 1),  "FILL");
                supportConfig.setData(SPEED_SMOOTH(getSettingInt("supportSpeed")), extrusionWidth, "SUPPORT");
#undef SPEED_SMOOTH
            }else{
                skirtConfig.setData(getSettingInt("skirtSpeed"), extrusionWidth, "SKIRT");
                inset0Config.setData(getSettingInt("inset0Speed"), extrusionWidth, "WALL-OUTER");
                insetXConfig.setData(getSettingInt("insetXSpeed"), extrusionWidth, "WALL-INNER");
                for(unsigned int idx=0; idx<MAX_SPARSE_COMBINE; idx++)
                    fillConfig[idx].setData(getSettingInt("infillSpeed"), extrusionWidth * (idx + 1), "FILL");
                supportConfig.setData(getSettingInt("supportSpeed"), extrusionWidth, "SUPPORT");
            }

            gcode.writeComment("LAYER:%d", layerNr);
            if (layerNr == 0)
                gcode.setExtrusion(getSettingInt("initialLayerThickness"), getSettingInt("filamentDiameter"), getSettingInt("filamentFlow"));
            else
                gcode.setExtrusion(getSettingInt("layerThickness"), getSettingInt("filamentDiameter"), getSettingInt("filamentFlow"));

            GCodePlanner gcodeLayer(gcode, getSettingInt("moveSpeed"), getSettingInt("retractionMinimalDistance"));
            int32_t z = getSettingInt("initialLayerThickness") + layerNr * getSettingInt("layerThickness");
            z += getSettingInt("raftBaseThickness") + getSettingInt("raftInterfaceThickness") + getSettingInt("raftSurfaceLayers")*getSettingInt("raftSurfaceThickness");
            if (getSettingInt("raftBaseThickness") > 0 && getSettingInt("raftInterfaceThickness") > 0)
            {
                if (layerNr == 0)
                {
                    z += getSettingInt("raftAirGapLayer0");
                } else {
                    z += getSettingInt("raftAirGap");
                }
            }
            gcode.setZ(z);

            bool printSupportFirst = (storage.support.generated && getSettingInt("supportExtruder") > 0 && getSettingInt("supportExtruder") == gcodeLayer.getExtruder());
            if (printSupportFirst)
                addSupportToGCode(storage, gcodeLayer, layerNr);

            for(unsigned int meshCnt = 0; meshCnt < storage.meshes.size(); meshCnt++)
            {
                if (meshCnt > 0)
                    meshIdx = (meshIdx + 1) % storage.meshes.size();
                addMeshLayerToGCode(storage, gcodeLayer, meshIdx, layerNr);
            }
            if (!printSupportFirst)
                addSupportToGCode(storage, gcodeLayer, layerNr);

            //Finish the layer by applying speed corrections for minimal layer times
            gcodeLayer.forceMinimalLayerTime(getSettingInt("minimalLayerTime"), getSettingInt("minimalFeedrate"));

            int fanSpeed = getSettingInt("fanSpeedMin");
            if (gcodeLayer.getExtrudeSpeedFactor() <= 50)
            {
                fanSpeed = getSettingInt("fanSpeedMax");
            }else{
                int n = gcodeLayer.getExtrudeSpeedFactor() - 50;
                fanSpeed = getSettingInt("fanSpeedMin") * n / 50 + getSettingInt("fanSpeedMax") * (50 - n) / 50;
            }
            if (static_cast<int>(layerNr) < getSettingInt("fanFullOnLayerNr"))
            {
                //Slow down the fan on the layers below the [fanFullOnLayerNr], where layer 0 is speed 0.
                fanSpeed = fanSpeed * layerNr / getSettingInt("fanFullOnLayerNr");
            }
            gcode.writeFanCommand(fanSpeed);

            gcodeLayer.writeGCode(getSettingInt("coolHeadLift") > 0, static_cast<int>(layerNr) > 0 ? getSettingInt("layerThickness") : getSettingInt("initialLayerThickness"));
        }

        log("Wrote layers in %5.2fs.\n", timeKeeper.restart());
        gcode.tellFileSize();
        gcode.writeFanCommand(0);

        //Store the object height for when we are printing multiple objects, as we need to clear every one of them when moving to the next position.
        maxObjectHeight = std::max(maxObjectHeight, storage.modelSize.z);
    }

    //Add a single layer from a single mesh-volume to the GCode
    void addMeshLayerToGCode(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int meshIdx, int layerNr)
    {
        int prevExtruder = gcodeLayer.getExtruder();
        bool extruderChanged = gcodeLayer.setExtruder(meshIdx);
        if (layerNr == 0 && meshIdx == 0)
        {
            //TODO: Don't think this needs to be here.
            if (storage.skirt.size() > 0)
                gcodeLayer.addTravel(storage.skirt[storage.skirt.size()-1].closestPointTo(gcode.getPositionXY()));
            gcodeLayer.addPolygonsByOptimizer(storage.skirt, &skirtConfig);
        }

        SliceLayer* layer = &storage.meshes[meshIdx].layers[layerNr];
        if (extruderChanged)
            addWipeTower(storage, gcodeLayer, layerNr, prevExtruder);

        if (storage.oozeShield.size() > 0 && storage.meshes.size() > 1)
        {
            gcodeLayer.setAlwaysRetract(true);
            gcodeLayer.addPolygonsByOptimizer(storage.oozeShield[layerNr], &skirtConfig);
            sendPolygons("oozeshield", layerNr, storage.oozeShield[layerNr]);
            gcodeLayer.setAlwaysRetract(!getSettingInt("enableCombing"));
        }

        if (getSettingInt("simpleMode"))
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
            if (getSettingInt("spiralizeMode"))
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

            if (getSettingInt("enableCombing"))
                gcodeLayer.setCombBoundary(&part->combBoundery);
            else
                gcodeLayer.setAlwaysRetract(true);

            if (getSettingInt("insetCount") > 0)
            {
                if (getSettingInt("spiralizeMode"))
                {
                    if (static_cast<int>(layerNr) >= getSettingInt("downSkinCount"))
                        inset0Config.spiralize = true;
                    if (static_cast<int>(layerNr) == getSettingInt("downSkinCount") && part->insets.size() > 0)
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

            int fillAngle = 45;
            if (layerNr & 1)
                fillAngle += 90;
            int extrusionWidth = getSettingInt("extrusionWidth");
            if (layerNr == 0)
                extrusionWidth = getSettingInt("layer0extrusionWidth");
            
            //Add thicker (multiple layers) sparse infill.
            if (getSettingInt("sparseInfillLineDistance") > 0)
            {
                for(unsigned int n=1; n<part->sparse_outline.size(); n++)
                {
                    Polygons fillPolygons;
                    if (getSetting("infillPattern") == "INFILL_GRID")
                    {
                        generateGridInfill(part->sparse_outline[n], fillPolygons, extrusionWidth, getSettingInt("sparseInfillLineDistance"), getSettingInt("infillOverlap"), fillAngle);
                    } else if (getSetting("infillPattern") == "INFILL_LINES")
                    {
                        generateLineInfill(part->sparse_outline[n], fillPolygons, extrusionWidth, getSettingInt("sparseInfillLineDistance"), getSettingInt("infillOverlap"), fillAngle);
                    } else if (getSetting("infillPattern") == "INFILL_CONCENTRIC")
                    {
                        generateConcentricInfill(part->sparse_outline[n], fillPolygons, getSettingInt("sparseInfillLineDistance"));
                    }
                    gcodeLayer.addPolygonsByOptimizer(fillPolygons, &fillConfig[n]);
                }
            }

            //Combine the 1 layer thick infill with the top/bottom skin and print that as one thing.
            Polygons fillPolygons;
            for(Polygons outline : part->skinOutline.splitIntoParts())
            {
                int bridge = -1;
                if (layerNr > 0)
                    bridge = bridgeAngle(outline, &storage.meshes[meshIdx].layers[layerNr-1]);
                if (bridge > -1)
                {
                    generateLineInfill(outline, fillPolygons, extrusionWidth, extrusionWidth, getSettingInt("infillOverlap"), bridge);
                }else{
                    if (getSetting("skinPattern") == "SKIN_LINES")
                    {
                        generateLineInfill(outline, fillPolygons, extrusionWidth, extrusionWidth, getSettingInt("infillOverlap"), fillAngle);
                    }else if (getSetting("skinPattern") == "SKIN_CONCENTRIC")
                    {
                        generateConcentricInfill(outline.offset(-extrusionWidth/2.0), fillPolygons, extrusionWidth);
                    }
                }
            }
            if (getSettingInt("sparseInfillLineDistance") > 0 && part->sparse_outline.size() > 0)
            {
                if (getSetting("infillPattern") == "INFILL_GRID")
                {
                    generateGridInfill(part->sparse_outline[0], fillPolygons, extrusionWidth, getSettingInt("sparseInfillLineDistance"), getSettingInt("infillOverlap"), fillAngle);
                } else if (getSetting("infillPattern") == "INFILL_LINES")
                {
                    generateLineInfill(part->sparse_outline[0], fillPolygons, extrusionWidth, getSettingInt("sparseInfillLineDistance"), getSettingInt("infillOverlap"), fillAngle);
                } else if (getSetting("infillPattern") == "INFILL_CONCENTRIC")
                {
                    generateConcentricInfill(part->sparse_outline[0], fillPolygons, getSettingInt("sparseInfillLineDistance"));
                }
            }
            gcodeLayer.addPolygonsByOptimizer(fillPolygons, &fillConfig[0]);

            //After a layer part, make sure the nozzle is inside the comb boundary, so we do not retract on the perimeter.
            if (!getSettingInt("spiralizeMode") || static_cast<int>(layerNr) < getSettingInt("downSkinCount"))
                gcodeLayer.moveInsideCombBoundary(extrusionWidth * 2);
        }
        gcodeLayer.setCombBoundary(nullptr);
    }

    void addSupportToGCode(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int layerNr)
    {
        if (!storage.support.generated)
            return;

        if (getSettingInt("supportExtruder") > -1)
        {
            int prevExtruder = gcodeLayer.getExtruder();
            if (gcodeLayer.setExtruder(getSettingInt("supportExtruder")))
                addWipeTower(storage, gcodeLayer, layerNr, prevExtruder);

            if (storage.oozeShield.size() > 0 && storage.meshes.size() == 1)
            {
                gcodeLayer.setAlwaysRetract(true);
                gcodeLayer.addPolygonsByOptimizer(storage.oozeShield[layerNr], &skirtConfig);
                gcodeLayer.setAlwaysRetract(!getSettingInt("enableCombing"));
            }
        }
        int32_t z = getSettingInt("initialLayerThickness") + layerNr * getSettingInt("layerThickness");
        SupportPolyGenerator supportGenerator(storage.support, z);
        for(unsigned int meshCnt = 0; meshCnt < storage.meshes.size(); meshCnt++)
        {
            SliceLayer* layer = &storage.meshes[meshCnt].layers[layerNr];
            for(unsigned int n=0; n<layer->parts.size(); n++)
                supportGenerator.polygons = supportGenerator.polygons.difference(layer->parts[n].outline.offset(getSettingInt("supportXYDistance")));
        }
        //Contract and expand the suppory polygons so small sections are removed and the final polygon is smoothed a bit.
        supportGenerator.polygons = supportGenerator.polygons.offset(-getSettingInt("extrusionWidth") * 3);
        supportGenerator.polygons = supportGenerator.polygons.offset(getSettingInt("extrusionWidth") * 3);
        sendPolygons("support", layerNr, supportGenerator.polygons);

        std::vector<Polygons> supportIslands = supportGenerator.polygons.splitIntoParts();

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
            if (getSettingInt("supportLineDistance") > 0)
            {
                int extrusionWidth = getSettingInt("extrusionWidth");
                if (getSetting("supportType") == "SUPPORT_TYPE_GRID")
                {
                    if (getSettingInt("supportLineDistance") > extrusionWidth * 4)
                    {
                        generateLineInfill(island, supportLines, extrusionWidth, getSettingInt("supportLineDistance")*2, getSettingInt("infillOverlap"), 0);
                        generateLineInfill(island, supportLines, extrusionWidth, getSettingInt("supportLineDistance")*2, getSettingInt("infillOverlap"), 90);
                    }else{
                        generateLineInfill(island, supportLines, extrusionWidth, getSettingInt("supportLineDistance"), getSettingInt("infillOverlap"), (layerNr & 1) ? 0 : 90);
                    }
                }else if (getSetting("supportType") == "SUPPORT_TYPE_LINES")
                {
                    generateLineInfill(island, supportLines, extrusionWidth, getSettingInt("supportLineDistance"), getSettingInt("infillOverlap"), 0);
                }
            }

            gcodeLayer.forceRetract();
            if (getSettingInt("enableCombing"))
                gcodeLayer.setCombBoundary(&island);
            if (getSetting("supportType") == "SUPPORT_TYPE_GRID")
                gcodeLayer.addPolygonsByOptimizer(island, &supportConfig);
            gcodeLayer.addPolygonsByOptimizer(supportLines, &supportConfig);
            gcodeLayer.setCombBoundary(nullptr);
        }
    }

    void addWipeTower(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int layerNr, int prevExtruder)
    {
        if (getSettingInt("wipeTowerSize") < 1)
            return;
        
        int extrusionWidth = getSettingInt("extrusionWidth");
        //If we changed extruder, print the wipe/prime tower for this nozzle;
        gcodeLayer.addPolygonsByOptimizer(storage.wipeTower, &supportConfig);
        Polygons fillPolygons;
        generateLineInfill(storage.wipeTower, fillPolygons, extrusionWidth, extrusionWidth, getSettingInt("infillOverlap"), 45 + 90 * (layerNr % 2));
        gcodeLayer.addPolygonsByOptimizer(fillPolygons, &supportConfig);

        //Make sure we wipe the old extruder on the wipe tower.
        gcodeLayer.addTravel(storage.wipePoint - gcode.getExtruderOffset(prevExtruder) + gcode.getExtruderOffset(gcodeLayer.getExtruder()));
    }
};

}//namespace cura

#endif//FFF_PROCESSOR_H
