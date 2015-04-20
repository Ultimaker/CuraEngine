#ifndef FFF_PROCESSOR_H
#define FFF_PROCESSOR_H

#include <algorithm>
#include <sstream>
#include <fstream>
#include "utils/gettime.h"
#include "utils/logoutput.h"
#include "sliceDataStorage.h"
#include "modelFile/modelFile.h"
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
#include "gcodePlanner.h"
#include "gcodeExport.h"
#include "commandSocket.h"
#include "Weaver.h"
#include "Wireframe2gcode.h"
#include "utils/polygonUtils.h"

namespace cura {

//FusedFilamentFabrication processor.
class fffProcessor : public SettingsBase
{
private:
    int maxObjectHeight;
    int fileNr; //!< used for sequential printing of objects
    GCodeExport gcode;
    TimeKeeper timeKeeper;
    CommandSocket* commandSocket;
    std::ofstream output_file;

public:
    fffProcessor()
    {
        fileNr = 1;
        maxObjectHeight = 0;
        commandSocket = NULL;
    }

    void resetFileNumber()
    {
        fileNr = 1;
    }

    void setCommandSocket(CommandSocket* socket)
    {
        commandSocket = socket;
    }

    void sendPolygons(PolygonType type, int layer_nr, Polygons& polygons)
    {
        if (commandSocket)
            commandSocket->sendPolygons(type, layer_nr, polygons);
    }
    
    bool setTargetFile(const char* filename)
    {
        output_file.open(filename);
        if (output_file.is_open())
        {
            gcode.setOutputStream(&output_file);
            return true;
        }
        return false;
    }
    
    void setTargetStream(std::ostream* stream)
    {
        gcode.setOutputStream(stream);
    }

    bool processFiles(const std::vector<std::string> &files)
    {
        timeKeeper.restart();
        PrintObject* model = nullptr;

        model = new PrintObject(this);
        for(std::string filename : files)
        {
            log("Loading %s from disk...\n", filename.c_str());

            FMatrix3x3 matrix;
            if (!loadMeshFromFile(model, filename.c_str(), matrix))
            {
                logError("Failed to load model: %s\n", filename.c_str());
                return false;
            }
        }
        model->finalize();

        log("Loaded from disk in %5.3fs\n", timeKeeper.restart());
        return processModel(model);
    }

    bool processModel(PrintObject* model)
    {
        timeKeeper.restart();
        if (!model)
            return false;

        TimeKeeper timeKeeperTotal;
        
        if (model->getSettingInt("neith"))
        {
            log("starting Neith Weaver...\n");
                        
            Weaver w(this);
            w.weave(model, commandSocket);
            
            log("starting Neith Gcode generation...\n");
            preSetup();
            Wireframe2gcode gcoder(w, gcode, this);
            gcoder.writeGCode(commandSocket, maxObjectHeight);
            log("finished Neith Gcode generation...\n");
            
        } else 
        {
            SliceDataStorage storage;
            preSetup();

            if (!prepareModel(storage, model))
                return false;


            processSliceData(storage);
            writeGCode(storage);
        }

        logProgress("process", 1, 1);//Report the GUI that a file has been fully processed.
        log("Total time elapsed %5.2fs.\n", timeKeeperTotal.restart());

        return true;
    }

    void finalize()
    {
        gcode.finalize(maxObjectHeight, getSettingInt("moveSpeed"), getSetting("endCode").c_str());
        for(int e=0; e<MAX_EXTRUDERS; e++)
            gcode.writeTemperatureCommand(e, 0, false);
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
        for(unsigned int n=1; n<MAX_EXTRUDERS;n++)
        {
            std::ostringstream stream;
            stream << "extruderOffset" << n;
            gcode.setExtruderOffset(n, Point(getSettingInt(stream.str() + ".X"), getSettingInt(stream.str() + ".Y")));
        }
        for(unsigned int n=0; n<MAX_EXTRUDERS;n++)
        {
            std::ostringstream stream;
            stream << n;
            gcode.setSwitchExtruderCode(n, getSetting("preSwitchExtruderCode" + stream.str()), getSetting("postSwitchExtruderCode" + stream.str()));
        }

        if (getSetting("gcodeFlavor") == "GCODE_FLAVOR_REPRAP")
            gcode.setFlavor(GCODE_FLAVOR_REPRAP);
        else if (getSetting("gcodeFlavor") == "GCODE_FLAVOR_ULTIGCODE")
            gcode.setFlavor(GCODE_FLAVOR_ULTIGCODE);
        else if (getSetting("gcodeFlavor") == "GCODE_FLAVOR_MAKERBOT")
            gcode.setFlavor(GCODE_FLAVOR_MAKERBOT);
        else if (getSetting("gcodeFlavor") == "GCODE_FLAVOR_BFB")
            gcode.setFlavor(GCODE_FLAVOR_BFB);
        else if (getSetting("gcodeFlavor") == "GCODE_FLAVOR_MACH3")
            gcode.setFlavor(GCODE_FLAVOR_MACH3);
        else if (getSetting("gcodeFlavor") == "GCODE_FLAVOR_REPRAP_VOLUMATRIC")
            gcode.setFlavor(GCODE_FLAVOR_REPRAP_VOLUMATRIC);
        gcode.setRetractionSettings(getSettingInt("retractionAmountExtruderSwitch"), getSettingInt("retractionExtruderSwitchSpeed"), getSettingInt("retractionExtruderSwitchPrimeSpeed"), getSettingInt("minimalExtrusionBeforeRetraction"));
    }

    bool prepareModel(SliceDataStorage& storage, PrintObject* object) /// slices the model
    {
        storage.model_min = object->min();
        storage.model_max = object->max();
        storage.model_size = storage.model_max - storage.model_min;

        log("Slicing model...\n");
        int initial_layer_thickness = object->getSettingInt("initialLayerThickness");
        int layer_thickness = object->getSettingInt("layerThickness");
        int layer_count = (storage.model_max.z - (initial_layer_thickness - layer_thickness / 2)) / layer_thickness + 1;
        std::vector<Slicer*> slicerList;
        for(Mesh& mesh : object->meshes)
        {
            int fix_horrible = mesh.getSettingInt("fixHorrible");
            Slicer* slicer = new Slicer(&mesh, initial_layer_thickness - layer_thickness / 2, layer_thickness, layer_count, fix_horrible & FIX_HORRIBLE_KEEP_NONE_CLOSED, fix_horrible & FIX_HORRIBLE_EXTENSIVE_STITCHING);
            slicerList.push_back(slicer);
            /*
            for(SlicerLayer& layer : slicer->layers)
            {
                //Reporting the outline here slows down the engine quite a bit, so only do so when debugging.
                //sendPolygons("outline", layer_nr, layer.z, layer.polygonList);
                //sendPolygons("openoutline", layer_nr, layer.openPolygonList);
            }
            */
        }
        
        if (false) { // remove empty first layers
            int n_empty_first_layers = 0;
            for (int layer_idx = 0; layer_idx < layer_count; layer_idx++)
            { 
                bool layer_is_empty = true;
                for (Slicer* slicer : slicerList)
                {
                    if (slicer->layers[layer_idx].polygonList.size() > 0)
                    {
                        layer_is_empty = false;
                        break;
                    }
                }
                
                if (layer_is_empty) 
                {
                    n_empty_first_layers++;
                } else
                {
                    break;
                }
            }
            
            if (n_empty_first_layers > 0)
            {
                for (Slicer* slicer : slicerList)
                {
                    std::vector<SlicerLayer>& layers = slicer->layers;
                    layers.erase(layers.begin(), layers.begin() + n_empty_first_layers);
                    for (SlicerLayer& layer : layers)
                    {
                        layer.z -= n_empty_first_layers * layer_thickness;
                    }
                }
                layer_count -= n_empty_first_layers;
            }
        }
        
        log("Layer count: %i\n", layer_count);
        log("Sliced model in %5.3fs\n", timeKeeper.restart());

        object->clear();///Clear the mesh data, it is no longer needed after this point, and it saves a lot of memory.

        log("Generating layer parts...\n");
        for(unsigned int meshIdx=0; meshIdx < slicerList.size(); meshIdx++)
        {
            storage.meshes.emplace_back(&object->meshes[meshIdx]);
            SliceMeshStorage& meshStorage = storage.meshes[meshIdx];
            createLayerParts(meshStorage, slicerList[meshIdx], meshStorage.settings->getSettingInt("fixHorrible") & (FIX_HORRIBLE_UNION_ALL_TYPE_A | FIX_HORRIBLE_UNION_ALL_TYPE_B | FIX_HORRIBLE_UNION_ALL_TYPE_C));
            delete slicerList[meshIdx];

            //Add the raft offset to each layer.
            for(unsigned int layer_nr=0; layer_nr<meshStorage.layers.size(); layer_nr++)
            {
                meshStorage.layers[layer_nr].printZ += meshStorage.settings->getSettingInt("raftBaseThickness") + meshStorage.settings->getSettingInt("raftInterfaceThickness");

                if (commandSocket)
                    commandSocket->sendLayerInfo(layer_nr, meshStorage.layers[layer_nr].printZ, layer_nr == 0 ? meshStorage.settings->getSettingInt("initialLayerThickness") : meshStorage.settings->getSettingInt("layerThickness"));
            }
        }
        log("Generated layer parts in %5.3fs\n", timeKeeper.restart());
        
        log("Finished prepareModel.\n");
        return true;
    }

    void processSliceData(SliceDataStorage& storage)
    {
        if (commandSocket)
           commandSocket->beginSendSlicedObject();

        // const 
        unsigned int totalLayers = storage.meshes[0].layers.size();

        //carveMultipleVolumes(storage.meshes);
        generateMultipleVolumesOverlap(storage.meshes, getSettingInt("multiVolumeOverlap"));
        //dumpLayerparts(storage, "c:/models/output.html");
        if (getSettingInt("simpleMode"))
        {
            for(unsigned int layer_nr=0; layer_nr<totalLayers; layer_nr++)
            {
                for(SliceMeshStorage& mesh : storage.meshes)
                {
                    SliceLayer* layer = &mesh.layers[layer_nr];
                    for(SliceLayerPart& part : layer->parts)
                    {
                        sendPolygons(Inset0Type, layer_nr, part.outline);
                    }
                }
            }
            return;
        }

        for(unsigned int layer_nr=0; layer_nr<totalLayers; layer_nr++)
        {
            for(SliceMeshStorage& mesh : storage.meshes)
            {
                int insetCount = mesh.settings->getSettingInt("insetCount");
                if (mesh.settings->getSettingInt("spiralizeMode") && static_cast<int>(layer_nr) < mesh.settings->getSettingInt("downSkinCount") && layer_nr % 2 == 1)//Add extra insets every 2 layers when spiralizing, this makes bottoms of cups watertight.
                    insetCount += 5;
                SliceLayer* layer = &mesh.layers[layer_nr];
                int extrusionWidth = mesh.settings->getSettingInt("extrusionWidth");
                if (layer_nr == 0)
                    extrusionWidth = mesh.settings->getSettingInt("layer0extrusionWidth");
                generateInsets(layer, extrusionWidth, insetCount, mesh.settings->getSettingInt("avoidOverlappingPerimeters"));

                for(unsigned int partNr=0; partNr<layer->parts.size(); partNr++)
                {
                    if (layer->parts[partNr].insets.size() > 0)
                    {
                        sendPolygons(Inset0Type, layer_nr, layer->parts[partNr].insets[0]);
                        for(unsigned int inset=1; inset<layer->parts[partNr].insets.size(); inset++)
                            sendPolygons(InsetXType, layer_nr, layer->parts[partNr].insets[inset]);
                    }
                }
            }
            logProgress("inset",layer_nr+1,totalLayers);
            if (commandSocket) commandSocket->sendProgress(1.0/3.0 * float(layer_nr) / float(totalLayers));
        }
        
   
        { // remove empty first layers
            int n_empty_first_layers = 0;
            for (int layer_idx = 0; layer_idx < totalLayers; layer_idx++)
            { 
                bool layer_is_empty = true;
                for (SliceMeshStorage& mesh : storage.meshes)
                {
                    if (mesh.layers[layer_idx].parts.size() > 0)
                    {
                        layer_is_empty = false;
                        break;
                    }
                }
                
                if (layer_is_empty) 
                {
                    n_empty_first_layers++;
                } else
                {
                    break;
                }
            }
            
            if (n_empty_first_layers > 0)
            {
                for (SliceMeshStorage& mesh : storage.meshes)
                {
                    std::vector<SliceLayer>& layers = mesh.layers;
                    layers.erase(layers.begin(), layers.begin() + n_empty_first_layers);
                    for (SliceLayer& layer : layers)
                    {
                        layer.printZ -= n_empty_first_layers * getSettingInt("layerThickness");
                    }
                }
                totalLayers -= n_empty_first_layers;
            }
        }
              
        if (getSettingInt("enableOozeShield"))
        {
            for(unsigned int layer_nr=0; layer_nr<totalLayers; layer_nr++)
            {
                Polygons oozeShield;
                for(SliceMeshStorage& mesh : storage.meshes)
                {
                    for(SliceLayerPart& part : mesh.layers[layer_nr].parts)
                    {
                        oozeShield = oozeShield.unionPolygons(part.outline.offset(MM2INT(2.0))); // TODO: put hard coded value in a variable with an explanatory name (and make var a parameter, and perhaps even a setting?)
                    }
                }
                storage.oozeShield.push_back(oozeShield);
            }

            for(unsigned int layer_nr=0; layer_nr<totalLayers; layer_nr++)
                storage.oozeShield[layer_nr] = storage.oozeShield[layer_nr].offset(-MM2INT(1.0)).offset(MM2INT(1.0)); // TODO: put hard coded value in a variable with an explanatory name (and make var a parameter, and perhaps even a setting?)
            int offsetAngle = tan(60.0*M_PI/180) * getSettingInt("layerThickness");//Allow for a 60deg angle in the oozeShield.
            for(unsigned int layer_nr=1; layer_nr<totalLayers; layer_nr++)
                storage.oozeShield[layer_nr] = storage.oozeShield[layer_nr].unionPolygons(storage.oozeShield[layer_nr-1].offset(-offsetAngle));
            for(unsigned int layer_nr=totalLayers-1; layer_nr>0; layer_nr--)
                storage.oozeShield[layer_nr-1] = storage.oozeShield[layer_nr-1].unionPolygons(storage.oozeShield[layer_nr].offset(-offsetAngle));
        }
        log("Generated inset in %5.3fs\n", timeKeeper.restart());  
             
        log("Generating support areas...\n");
        for(SliceMeshStorage& mesh : storage.meshes)
        {
            generateSupportAreas(storage, &mesh, totalLayers);
        }
        log("Generated support areas in %5.3fs\n", timeKeeper.restart());
        


        for(unsigned int layer_nr=0; layer_nr<totalLayers; layer_nr++)
        {
            if (!getSettingInt("spiralizeMode") || static_cast<int>(layer_nr) < getSettingInt("downSkinCount"))    //Only generate up/downskin and infill for the first X layers when spiralize is choosen.
            {
                for(SliceMeshStorage& mesh : storage.meshes)
                {
                    int extrusionWidth = mesh.settings->getSettingInt("extrusionWidth");
                    if (layer_nr == 0)
                        extrusionWidth = mesh.settings->getSettingInt("layer0extrusionWidth");
                    generateSkins(layer_nr, mesh, extrusionWidth, mesh.settings->getSettingInt("downSkinCount"), mesh.settings->getSettingInt("upSkinCount"), mesh.settings->getSettingInt("skinPerimeterCount"), mesh.settings->getSettingInt("avoidOverlappingPerimeters"));
                    if (mesh.settings->getSettingInt("sparseInfillLineDistance") > 0)
                        generateSparse(layer_nr, mesh, extrusionWidth, mesh.settings->getSettingInt("downSkinCount"), mesh.settings->getSettingInt("upSkinCount"), mesh.settings->getSettingInt("avoidOverlappingPerimeters"));

                    SliceLayer& layer = mesh.layers[layer_nr];
                    for(SliceLayerPart& part : layer.parts)
                        sendPolygons(SkinType, layer_nr, part.skinOutline);
                }
            }
            logProgress("skin", layer_nr+1, totalLayers);
            if (commandSocket) commandSocket->sendProgress(1.0/3.0 + 1.0/3.0 * float(layer_nr) / float(totalLayers));
        }
        for(unsigned int layer_nr=totalLayers-1; layer_nr>0; layer_nr--)
        {
            for(SliceMeshStorage& mesh : storage.meshes)
                combineSparseLayers(layer_nr, mesh, mesh.settings->getSettingInt("sparseInfillCombineCount"));
        }
        log("Generated up/down skin in %5.3fs\n", timeKeeper.restart());

        if (getSettingInt("wipeTowerSize") > 0)
        {
            PolygonRef p = storage.wipeTower.newPoly();
            int tower_size = getSettingInt("wipeTowerSize");
            int tower_distance = getSettingInt("wipeTowerDistance");
            p.add(Point(storage.model_min.x - tower_distance, storage.model_max.y + tower_distance));
            p.add(Point(storage.model_min.x - tower_distance, storage.model_max.y + tower_distance + tower_size));
            p.add(Point(storage.model_min.x - tower_distance - tower_size, storage.model_max.y + tower_distance + tower_size));
            p.add(Point(storage.model_min.x - tower_distance - tower_size, storage.model_max.y + tower_distance));

            storage.wipePoint = Point(storage.model_min.x - tower_distance - tower_size / 2, storage.model_max.y + tower_distance + tower_size / 2);
        }

        generateSkirt(storage, getSettingInt("skirtDistance"), getSettingInt("layer0extrusionWidth"), getSettingInt("skirtLineCount"), getSettingInt("skirtMinLength"), getSettingInt("initialLayerThickness"));
        generateRaft(storage, getSettingInt("raftMargin"));

        sendPolygons(SkirtType, 0, storage.skirt);
    }

    void writeGCode(SliceDataStorage& storage)
    {
        gcode.resetTotalPrintTime();
        
        if (commandSocket)
            commandSocket->beginGCode();

        //Setup the retraction parameters.
        storage.retraction_config.amount = INT2MM(getSettingInt("retractionAmount"));
        storage.retraction_config.primeAmount = INT2MM(getSettingInt("retractionPrimeAmount"));
        storage.retraction_config.speed = getSettingInt("retractionSpeed");
        storage.retraction_config.primeSpeed = getSettingInt("retractionPrimeSpeed");
        storage.retraction_config.zHop = getSettingInt("retractionZHop");
        for(SliceMeshStorage& mesh : storage.meshes)
        {
            mesh.retraction_config.amount = INT2MM(mesh.settings->getSettingInt("retractionAmount"));
            mesh.retraction_config.primeAmount = INT2MM(mesh.settings->getSettingInt("retractionPrimeAmount"));
            mesh.retraction_config.speed = mesh.settings->getSettingInt("retractionSpeed");
            mesh.retraction_config.primeSpeed = mesh.settings->getSettingInt("retractionPrimeSpeed");
            mesh.retraction_config.zHop = mesh.settings->getSettingInt("retractionZHop");
        }

        if (fileNr == 1)
        {
            if (hasSetting("bedTemperature") && getSettingInt("bedTemperature") > 0)
                gcode.writeBedTemperatureCommand(getSettingInt("bedTemperature"), true);
            
            for(SliceMeshStorage& mesh : storage.meshes)
                if (mesh.settings->hasSetting("printTemperature") && mesh.settings->getSettingInt("printTemperature") > 0)
                    gcode.writeTemperatureCommand(mesh.settings->getSettingInt("extruderNr"), mesh.settings->getSettingInt("printTemperature"));
            for(SliceMeshStorage& mesh : storage.meshes)
                if (mesh.settings->hasSetting("printTemperature") && mesh.settings->getSettingInt("printTemperature") > 0)
                    gcode.writeTemperatureCommand(mesh.settings->getSettingInt("extruderNr"), mesh.settings->getSettingInt("printTemperature"), true);
            
            gcode.writeCode(getSetting("startCode").c_str());
            gcode.writeComment("Generated with Cura_SteamEngine " VERSION);
            if (gcode.getFlavor() == GCODE_FLAVOR_BFB)
            {
                gcode.writeComment("enable auto-retraction");
                std::ostringstream tmp;
                tmp << "M227 S" << (getSettingInt("retractionAmount") * 2560 / 1000) << " P" << (getSettingInt("retractionAmount") * 2560 / 1000);
                gcode.writeLine(tmp.str().c_str());
            }
        }else{
            gcode.writeFanCommand(0);
            gcode.resetExtrusionValue();
            gcode.setZ(maxObjectHeight + 5000);
            gcode.writeMove(gcode.getPositionXY(), getSettingInt("moveSpeed"), 0);
            gcode.writeMove(Point(storage.model_min.x, storage.model_min.y), getSettingInt("moveSpeed"), 0);
        }
        fileNr++;

        unsigned int totalLayers = storage.meshes[0].layers.size();
        //gcode.writeComment("Layer count: %d", totalLayers);

        if (getSettingInt("raftBaseThickness") > 0 && getSettingInt("raftInterfaceThickness") > 0)
        {
            GCodePathConfig raft_base_config(&storage.retraction_config, "SUPPORT");
            raft_base_config.setSpeed(getSettingInt("raftBaseSpeed"));
            raft_base_config.setLineWidth(getSettingInt("raftBaseLinewidth"));
            raft_base_config.setLayerHeight(getSettingInt("raftBaseThickness"));
            raft_base_config.setFilamentDiameter(getSettingInt("filamentDiameter"));
            raft_base_config.setFlow(getSettingInt("filamentFlow"));
            GCodePathConfig raft_interface_config(&storage.retraction_config, "SUPPORT");
            raft_interface_config.setSpeed(getSettingInt("raftInterfaceSpeed"));
            raft_interface_config.setLineWidth(getSettingInt("raftInterfaceLinewidth"));
            raft_interface_config.setLayerHeight(getSettingInt("raftBaseThickness"));
            raft_interface_config.setFilamentDiameter(getSettingInt("filamentDiameter"));
            raft_interface_config.setFlow(getSettingInt("filamentFlow"));
            GCodePathConfig raft_surface_config(&storage.retraction_config, "SUPPORT");
            raft_surface_config.setSpeed(getSettingInt("raftSurfaceSpeed"));
            raft_surface_config.setLineWidth(getSettingInt("raftSurfaceLinewidth"));
            raft_surface_config.setLayerHeight(getSettingInt("raftBaseThickness"));
            raft_surface_config.setFilamentDiameter(getSettingInt("filamentDiameter"));
            raft_surface_config.setFlow(getSettingInt("filamentFlow"));
            {
                gcode.writeLayerComment(-2);
                gcode.writeComment("RAFT");
                GCodePlanner gcodeLayer(gcode, &storage.retraction_config, getSettingInt("moveSpeed"), getSettingInt("retractionMinimalDistance"));
                if (getSettingInt("supportExtruder") > 0)
                    gcodeLayer.setExtruder(getSettingInt("supportExtruder"));
                gcode.setZ(getSettingInt("raftBaseThickness"));
                gcodeLayer.addPolygonsByOptimizer(storage.raftOutline, &raft_base_config);

                Polygons raftLines;
                int offset_from_poly_outline = 0;
                generateLineInfill(storage.raftOutline, offset_from_poly_outline, raftLines, getSettingInt("raftBaseLinewidth"), getSettingInt("raftLineSpacing"), getSettingInt("infillOverlap"), 0);
                gcodeLayer.addLinesByOptimizer(raftLines, &raft_base_config);

                gcodeLayer.writeGCode(false, getSettingInt("raftBaseThickness"));
            }

            if (getSettingInt("raftFanSpeed"))
            {
                gcode.writeFanCommand(getSettingInt("raftFanSpeed"));
            }

            { /// this code block is about something which is of yet unknown
                gcode.writeLayerComment(-1);
                gcode.writeComment("RAFT");
                GCodePlanner gcodeLayer(gcode, &storage.retraction_config, getSettingInt("moveSpeed"), getSettingInt("retractionMinimalDistance"));
                gcode.setZ(getSettingInt("raftBaseThickness") + getSettingInt("raftInterfaceThickness"));

                Polygons raftLines;
                int offset_from_poly_outline = 0;
                generateLineInfill(storage.raftOutline, offset_from_poly_outline, raftLines, getSettingInt("raftInterfaceLinewidth"), getSettingInt("raftInterfaceLineSpacing"), getSettingInt("infillOverlap"), getSettingInt("raftSurfaceLayers") > 0 ? 45 : 90);
                gcodeLayer.addLinesByOptimizer(raftLines, &raft_interface_config);

                gcodeLayer.writeGCode(false, getSettingInt("raftInterfaceThickness"));
            }

            for (int raftSurfaceLayer=1; raftSurfaceLayer<=getSettingInt("raftSurfaceLayers"); raftSurfaceLayer++)
            {
                gcode.writeLayerComment(-1);
                gcode.writeComment("RAFT");
                GCodePlanner gcodeLayer(gcode, &storage.retraction_config, getSettingInt("moveSpeed"), getSettingInt("retractionMinimalDistance"));
                gcode.setZ(getSettingInt("raftBaseThickness") + getSettingInt("raftInterfaceThickness") + getSettingInt("raftSurfaceThickness")*raftSurfaceLayer);

                Polygons raftLines;
                int offset_from_poly_outline = 0;
                generateLineInfill(storage.raftOutline, offset_from_poly_outline, raftLines, getSettingInt("raftSurfaceLinewidth"), getSettingInt("raftSurfaceLineSpacing"), getSettingInt("infillOverlap"), 90 * raftSurfaceLayer);
                gcodeLayer.addLinesByOptimizer(raftLines, &raft_surface_config);

                gcodeLayer.writeGCode(false, getSettingInt("raftInterfaceThickness"));
            }
        }

        for(unsigned int layer_nr=0; layer_nr<totalLayers; layer_nr++)
        {
            logProgress("export", layer_nr+1, totalLayers);
            if (commandSocket) commandSocket->sendProgress(2.0/3.0 + 1.0/3.0 * float(layer_nr) / float(totalLayers));

            int extrusion_width = getSettingInt("extrusionWidth");
            int layer_thickness = getSettingInt("layerThickness");
            if (layer_nr == 0)
            {
                extrusion_width = getSettingInt("layer0extrusionWidth");
                layer_thickness = getSettingInt("initialLayerThickness");
            }

            storage.skirt_config.setSpeed(getSettingInt("skirtSpeed"));
            storage.skirt_config.setLineWidth(extrusion_width);
            storage.skirt_config.setFilamentDiameter(getSettingInt("filamentDiameter"));
            storage.skirt_config.setFlow(getSettingInt("filamentFlow"));
            storage.skirt_config.setLayerHeight(layer_thickness);

            storage.support_config.setLineWidth(getSettingInt("supportExtrusionWidth"));
            storage.support_config.setSpeed(getSettingInt("supportSpeed"));
            storage.support_config.setFilamentDiameter(getSettingInt("filamentDiameter"));
            storage.support_config.setFlow(getSettingInt("filamentFlow"));
            storage.support_config.setLayerHeight(layer_thickness);
            for(SliceMeshStorage& mesh : storage.meshes)
            {
                extrusion_width = mesh.settings->getSettingInt("extrusionWidth");
                if (layer_nr == 0)
                    extrusion_width = mesh.settings->getSettingInt("layer0extrusionWidth");

                mesh.inset0_config.setLineWidth(extrusion_width);
                mesh.inset0_config.setSpeed(mesh.settings->getSettingInt("inset0Speed"));
                mesh.inset0_config.setFilamentDiameter(mesh.settings->getSettingInt("filamentDiameter"));
                mesh.inset0_config.setFlow(mesh.settings->getSettingInt("filamentFlow"));
                mesh.inset0_config.setLayerHeight(layer_thickness);

                mesh.insetX_config.setLineWidth(extrusion_width);
                mesh.insetX_config.setSpeed(mesh.settings->getSettingInt("insetXSpeed"));
                mesh.insetX_config.setFilamentDiameter(mesh.settings->getSettingInt("filamentDiameter"));
                mesh.insetX_config.setFlow(mesh.settings->getSettingInt("filamentFlow"));
                mesh.insetX_config.setLayerHeight(layer_thickness);

                mesh.skin_config.setLineWidth(extrusion_width);
                mesh.skin_config.setSpeed(mesh.settings->getSettingInt("skinSpeed"));
                mesh.skin_config.setFilamentDiameter(mesh.settings->getSettingInt("filamentDiameter"));
                mesh.skin_config.setFlow(mesh.settings->getSettingInt("filamentFlow"));
                mesh.skin_config.setLayerHeight(layer_thickness);

                for(unsigned int idx=0; idx<MAX_SPARSE_COMBINE; idx++)
                {
                    mesh.infill_config[idx].setLineWidth(extrusion_width * (idx + 1));
                    mesh.infill_config[idx].setSpeed(mesh.settings->getSettingInt("infillSpeed"));
                    mesh.infill_config[idx].setFilamentDiameter(mesh.settings->getSettingInt("filamentDiameter"));
                    mesh.infill_config[idx].setFlow(mesh.settings->getSettingInt("filamentFlow"));
                    mesh.infill_config[idx].setLayerHeight(layer_thickness);
                }
            }

            int initial_speedup_layers = getSettingInt("initialSpeedupLayers");
            if (static_cast<int>(layer_nr) < initial_speedup_layers)
            {
                int initial_layer_speed = getSettingInt("initialLayerSpeed");
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

            GCodePlanner gcodeLayer(gcode, &storage.retraction_config, getSettingInt("moveSpeed"), getSettingInt("retractionMinimalDistance"));
            int32_t z = getSettingInt("initialLayerThickness") + layer_nr * getSettingInt("layerThickness");
            z += getSettingInt("raftBaseThickness") + getSettingInt("raftInterfaceThickness") + getSettingInt("raftSurfaceLayers")*getSettingInt("raftSurfaceThickness");
            if (getSettingInt("raftBaseThickness") > 0 && getSettingInt("raftInterfaceThickness") > 0)
            {
                if (layer_nr == 0)
                {
                    z += getSettingInt("raftAirGapLayer0");
                } else {
                    z += getSettingInt("raftAirGap");
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

            bool printSupportFirst = (storage.support.generated && getSettingInt("supportExtruder") > 0 && getSettingInt("supportExtruder") == gcodeLayer.getExtruder());
            if (printSupportFirst)
                addSupportToGCode(storage, gcodeLayer, layer_nr);

            if (storage.oozeShield.size() > 0)
            {
                gcodeLayer.setAlwaysRetract(true);
                gcodeLayer.addPolygonsByOptimizer(storage.oozeShield[layer_nr], &storage.skirt_config);
                gcodeLayer.setAlwaysRetract(!getSettingInt("enableCombing"));
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
                gcodeLayer.forceMinimalLayerTime(getSettingInt("minimalLayerTime"), getSettingInt("minimalFeedrate"), travelTime, extrudeTime);

                // interpolate fan speed (for fanFullOnLayerNr and for minimalLayerTimeFanSpeedMin)
                int fanSpeed = getSettingInt("fanSpeedMin");
                double totalLayerTime = travelTime + extrudeTime;
                if (totalLayerTime < getSettingInt("minimalLayerTime"))
                {
                    fanSpeed = getSettingInt("fanSpeedMax");
                }
                else if (totalLayerTime < getSettingInt("minimalLayerTimeFanSpeedMin"))
                { 
                    // when forceMinimalLayerTime didn't change the extrusionSpeedFactor, we adjust the fan speed
                    double minTime = (getSettingInt("minimalLayerTime"));
                    double maxTime = (getSettingInt("minimalLayerTimeFanSpeedMin"));
                    int fanSpeedMin = getSettingInt("fanSpeedMin");
                    int fanSpeedMax = getSettingInt("fanSpeedMax");
                    fanSpeed = fanSpeedMax - (fanSpeedMax-fanSpeedMin) * (totalLayerTime - minTime) / (maxTime - minTime);
                }
                if (static_cast<int>(layer_nr) < getSettingInt("fanFullOnLayerNr"))
                {
                    //Slow down the fan on the layers below the [fanFullOnLayerNr], where layer 0 is speed 0.
                    fanSpeed = fanSpeed * layer_nr / getSettingInt("fanFullOnLayerNr");
                }
                gcode.writeFanCommand(fanSpeed);
            }

            gcodeLayer.writeGCode(getSettingInt("coolHeadLift") > 0, static_cast<int>(layer_nr) > 0 ? getSettingInt("layerThickness") : getSettingInt("initialLayerThickness"));
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

    std::vector<SliceMeshStorage*> calculateMeshOrder(SliceDataStorage& storage, int current_extruder)
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
                if (add_list[idx]->settings->getSettingInt("extruderNr") == add_extruder_nr)
                {
                    ret.push_back(add_list[idx]);
                    add_list.erase(add_list.begin() + idx);
                    idx--;
                }
            }
            if (add_list.size() > 0)
                add_extruder_nr = add_list[0]->settings->getSettingInt("extruderNr");
        }
        return ret;
    }

    //Add a single layer from a single mesh-volume to the GCode
    void addMeshLayerToGCode(SliceDataStorage& storage, SliceMeshStorage* mesh, GCodePlanner& gcodeLayer, int layer_nr)
    {
        int prevExtruder = gcodeLayer.getExtruder();
        bool extruder_changed = gcodeLayer.setExtruder(mesh->settings->getSettingInt("extruderNr"));

        if (extruder_changed)
            addWipeTower(storage, gcodeLayer, layer_nr, prevExtruder);

        SliceLayer* layer = &mesh->layers[layer_nr];

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
            if (mesh->settings->getSettingInt("spiralizeMode"))
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

        for(unsigned int partCounter=0; partCounter<partOrderOptimizer.polyOrder.size(); partCounter++)
        {
            SliceLayerPart* part = &layer->parts[partOrderOptimizer.polyOrder[partCounter]];

            if (getSettingInt("enableCombing"))
                gcodeLayer.setCombBoundary(&part->combBoundery);
            else
                gcodeLayer.setAlwaysRetract(true);

            int fillAngle = 45;
            if (layer_nr & 1)
                fillAngle += 90;
            int extrusionWidth = getSettingInt("extrusionWidth");
            if (layer_nr == 0)
                extrusionWidth = getSettingInt("layer0extrusionWidth");

            //Add thicker (multiple layers) sparse infill.
            if (getSettingInt("sparseInfillLineDistance") > 0)
            {
                for(unsigned int n=1; n<part->sparse_outline.size(); n++)
                {
                    Polygons fillPolygons;
                    if (getSetting("infillPattern") == "INFILL_GRID")
                    {
                        generateGridInfill(part->sparse_outline[n], 0, fillPolygons, extrusionWidth, getSettingInt("sparseInfillLineDistance") * 2, getSettingInt("infillOverlap"), fillAngle);
                        gcodeLayer.addLinesByOptimizer(fillPolygons, &mesh->infill_config[n]);
                    } else if (getSetting("infillPattern") == "INFILL_LINES")
                    {
                        generateLineInfill(part->sparse_outline[n], 0, fillPolygons, extrusionWidth, getSettingInt("sparseInfillLineDistance"), getSettingInt("infillOverlap"), fillAngle);
                        gcodeLayer.addLinesByOptimizer(fillPolygons, &mesh->infill_config[n]);
                    } else if (getSetting("infillPattern") == "INFILL_TRIANGLES")
                    {
                        generateTriangleInfill(part->sparse_outline[n], 0, fillPolygons, extrusionWidth, getSettingInt("sparseInfillLineDistance") * 3, getSettingInt("infillOverlap"), 0);
                        gcodeLayer.addLinesByOptimizer(fillPolygons, &mesh->infill_config[n]);
                    } else if (getSetting("infillPattern") == "INFILL_CONCENTRIC")
                    {
                        generateConcentricInfill(part->sparse_outline[n], fillPolygons, getSettingInt("sparseInfillLineDistance"));
                        gcodeLayer.addPolygonsByOptimizer(fillPolygons, &mesh->infill_config[n]);
                    } else if (getSetting("infillPattern") == "INFILL_ZIGZAG")
                    {
                        generateZigZagInfill(part->sparse_outline[n], fillPolygons, extrusionWidth, getSettingInt("sparseInfillLineDistance"), getSettingInt("infillOverlap"), fillAngle, false, false);
                        gcodeLayer.addPolygonsByOptimizer(fillPolygons, &mesh->infill_config[n]);
                    } else {
                        logError("infillPattern has unknown value.\n");
                    }
                }
            }

            //Combine the 1 layer thick infill with the top/bottom skin and print that as one thing.
            Polygons infillPolygons;
            Polygons infillLines;
            if (getSettingInt("sparseInfillLineDistance") > 0 && part->sparse_outline.size() > 0)
            {
                if (getSetting("infillPattern") == "INFILL_GRID")
                {
                    generateGridInfill(part->sparse_outline[0], 0, infillLines, extrusionWidth, getSettingInt("sparseInfillLineDistance") * 2, getSettingInt("infillOverlap"), fillAngle);
                } else if (getSetting("infillPattern") == "INFILL_LINES")
                {
                    generateLineInfill(part->sparse_outline[0], 0, infillLines, extrusionWidth, getSettingInt("sparseInfillLineDistance"), getSettingInt("infillOverlap"), fillAngle);
                } else if (getSetting("infillPattern") == "INFILL_TRIANGLES")
                {
                    generateTriangleInfill(part->sparse_outline[0], 0, infillLines, extrusionWidth, getSettingInt("sparseInfillLineDistance") * 3, getSettingInt("infillOverlap"), 0);
                } else if (getSetting("infillPattern") == "INFILL_CONCENTRIC")
                {
                    generateConcentricInfill(part->sparse_outline[0], infillPolygons, getSettingInt("sparseInfillLineDistance"));
                } else if (getSetting("infillPattern") == "INFILL_ZIGZAG")
                {
                    generateZigZagInfill(part->sparse_outline[0], infillLines, extrusionWidth, getSettingInt("sparseInfillLineDistance"), getSettingInt("infillOverlap"), fillAngle, false, false);
                }  else {
                    logError("infillPattern has unknown value.\n");
                }
            }
            gcodeLayer.addPolygonsByOptimizer(infillPolygons, &mesh->infill_config[0]);
            gcodeLayer.addLinesByOptimizer(infillLines, &mesh->infill_config[0]);

            if (getSettingInt("insetCount") > 0)
            {
                if (getSettingInt("spiralizeMode"))
                {
                    if (static_cast<int>(layer_nr) >= getSettingInt("downSkinCount"))
                        mesh->inset0_config.spiralize = true;
                    if (static_cast<int>(layer_nr) == getSettingInt("downSkinCount") && part->insets.size() > 0)
                        gcodeLayer.addPolygonsByOptimizer(part->insets[0], &mesh->insetX_config);
                }
                for(int insetNr=part->insets.size()-1; insetNr>-1; insetNr--)
                {
                    if (insetNr == 0)
                        gcodeLayer.addPolygonsByOptimizer(part->insets[insetNr], &mesh->inset0_config);
                    else
                        gcodeLayer.addPolygonsByOptimizer(part->insets[insetNr], &mesh->insetX_config);
                }
            }

            Polygons skinPolygons;
            Polygons skinLines;
            for(Polygons outline : part->skinOutline.splitIntoParts())
            {
                int bridge = -1;
                if (layer_nr > 0)
                    bridge = bridgeAngle(outline, &mesh->layers[layer_nr-1]);
                if (bridge > -1)
                {
                    generateLineInfill(outline, 0, skinLines, extrusionWidth, extrusionWidth, getSettingInt("infillOverlap"), bridge);
                }else{
                    if (getSetting("skinPattern") == "SKIN_LINES")
                    {
                        for (Polygons& skin_perimeter : part->skinInsets)
                            gcodeLayer.addPolygonsByOptimizer(skin_perimeter, &mesh->skin_config);
                        if (part->skinInsets.size() > 0)
                        {
                            generateLineInfill(part->skinInsets.back(), -extrusionWidth/2, skinLines, extrusionWidth, extrusionWidth, getSettingInt("infillOverlap"), fillAngle);
                        } 
                        else
                        {
                            generateLineInfill(part->skinOutline, 0, skinLines, extrusionWidth, extrusionWidth, getSettingInt("infillOverlap"), fillAngle);
                        }
                    }else if (getSetting("skinPattern") == "SKIN_CONCENTRIC")
                    {
                        Polygons in_outline;
                        offsetSafe(outline, -extrusionWidth/2, extrusionWidth, in_outline, getSettingInt("avoidOverlappingPerimeters"));
                        
                        generateConcentricInfillDense(in_outline, skinPolygons, &part->perimeterGaps, extrusionWidth, getSettingInt("avoidOverlappingPerimeters"));
                        
                    }
                }
            }
            gcodeLayer.addPolygonsByOptimizer(skinPolygons, &mesh->skin_config);
            gcodeLayer.addLinesByOptimizer(skinLines, &mesh->skin_config);
            
            
            Polygons gapLines; // gaps between perimeters etc.
            generateLineInfill(part->perimeterGaps, 0, gapLines, extrusionWidth, extrusionWidth, getSettingInt("infillOverlap"), fillAngle);
            gcodeLayer.addLinesByOptimizer(gapLines, &mesh->skin_config);

            //After a layer part, make sure the nozzle is inside the comb boundary, so we do not retract on the perimeter.
            if (!getSettingInt("spiralizeMode") || static_cast<int>(layer_nr) < getSettingInt("downSkinCount"))
                gcodeLayer.moveInsideCombBoundary(extrusionWidth * 2);
        }
        gcodeLayer.setCombBoundary(nullptr);
    }

    void addSupportToGCode(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int layer_nr)
    {
        if (!storage.support.generated)
            return;
        
        
        if (getSettingInt("supportExtruder") > -1)
        {
            int prevExtruder = gcodeLayer.getExtruder();
            if (gcodeLayer.setExtruder(getSettingInt("supportExtruder")))
                addWipeTower(storage, gcodeLayer, layer_nr, prevExtruder);
        }
        Polygons support;
        if (storage.support.generated) 
            support = storage.support.supportAreasPerLayer[layer_nr];
        
        sendPolygons(SupportType, layer_nr, support);

        std::vector<Polygons> supportIslands = support.splitIntoParts();

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
                if (getSetting("supportType") == "GRID")
                {
                    int offset_from_outline = 0;
                    if (getSettingInt("supportLineDistance") > extrusionWidth * 4)
                    {
                        generateGridInfill(island, offset_from_outline, supportLines, extrusionWidth, getSettingInt("supportLineDistance")*2, getSettingInt("infillOverlap"), 0);
                    }else{
                        generateLineInfill(island, offset_from_outline, supportLines, extrusionWidth, getSettingInt("supportLineDistance"), getSettingInt("infillOverlap"), (layer_nr & 1) ? 0 : 90);
                    }
                }else if (getSetting("supportType") == "LINES")
                {
                    int offset_from_outline = 0;
                    if (layer_nr == 0)
                    {
                        generateGridInfill(island, offset_from_outline, supportLines, extrusionWidth, getSettingInt("supportLineDistance"), getSettingInt("infillOverlap") + 150, 0);
                    }else{
                        generateLineInfill(island, offset_from_outline, supportLines, extrusionWidth, getSettingInt("supportLineDistance"), getSettingInt("infillOverlap"), 0);
                    }
                }else if (getSetting("supportType") == "ZIGZAG")
                {
                    int offset_from_outline = 0;
                    if (layer_nr == 0)
                    {
                        generateGridInfill(island, offset_from_outline, supportLines, extrusionWidth, getSettingInt("supportLineDistance"), getSettingInt("infillOverlap") + 150, 0);
                    }else{
                        generateZigZagInfill(island, supportLines, extrusionWidth, getSettingInt("supportLineDistance"), getSettingInt("infillOverlap"), 0, getSettingInt("supportConnectZigZags") >0, true);
                    }
                }
            }

            gcodeLayer.forceRetract();
            if (getSettingInt("enableCombing"))
                gcodeLayer.setCombBoundary(&island);
            if (getSetting("supportType") == "GRID" || ( getSetting("supportType") == "ZIGZAG" && layer_nr == 0 ) )
                gcodeLayer.addPolygonsByOptimizer(island, &storage.support_config);
            gcodeLayer.addLinesByOptimizer(supportLines, &storage.support_config);
            gcodeLayer.setCombBoundary(nullptr);
        }
    }

    void addWipeTower(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int layer_nr, int prevExtruder)
    {
        if (getSettingInt("wipeTowerSize") < 1)
            return;

        int64_t offset = -getSettingInt("extrusionWidth");
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
};

}//namespace cura

#endif//FFF_PROCESSOR_H
