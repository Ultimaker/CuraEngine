#ifndef FFF_PROCESSOR_H
#define FFF_PROCESSOR_H

#include "settings.h"
#include "FffGcodeWriter.h"
#include "FfPolygonGenerator.h"
#include "commandSocket.h"
#include "Weaver.h"
#include "Wireframe2gcode.h"

namespace cura {

//FusedFilamentFabrication processor.
class fffProcessor
{
private:
    FfPolygonGenerator areaGenerator;
    FffGcodeWriter gcodeWriter;
    TimeKeeper timeKeeper;
    CommandSocket* commandSocket;

public:
    SettingsBase settings;
    
    fffProcessor()
    : areaGenerator(settings)
    , gcodeWriter(settings)
    {
        commandSocket = NULL;
    }
    
    void resetFileNumber()
    {
        gcodeWriter.resetFileNumber();
    }

    void setCommandSocket(CommandSocket* socket)
    {
        commandSocket = socket;
        gcodeWriter.setCommandSocket(socket);
        areaGenerator.setCommandSocket(socket);
    }

    void sendPolygons(PolygonType type, int layer_nr, Polygons& polygons)
    {
        if (commandSocket)
            commandSocket->sendPolygons(type, layer_nr, polygons);
    }
    
    bool setTargetFile(const char* filename)
    {
        return gcodeWriter.setTargetFile(filename);
    }
    
    void setTargetStream(std::ostream* stream)
    {
        return gcodeWriter.setTargetStream(stream);
    }

    double getTotalFilamentUsed(int e)
    {
        return gcodeWriter.getTotalFilamentUsed(e);
    }

    double getTotalPrintTime()
    {
        return gcodeWriter.getTotalPrintTime();
    }

    bool processFiles(const std::vector<std::string> &files)
    {
        timeKeeper.restart();
        PrintObject* model = nullptr;

        model = new PrintObject(&settings);
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
    
    void finalize()
    {
        gcodeWriter.finalize();
    }
    
    bool processModel(PrintObject* model)
    {
        timeKeeper.restart();
        if (!model)
            return false;

        TimeKeeper timeKeeperTotal;
        
        if (model->getSettingBoolean("wireframe_enabled"))
        {
            log("starting Neith Weaver...\n");
                        
            Weaver w(&settings);
            w.weave(model, commandSocket);
            
            log("starting Neith Gcode generation...\n");
            Wireframe2gcode gcoder(w, gcodeWriter.gcode, &settings);
            gcoder.writeGCode(commandSocket);
            log("finished Neith Gcode generation...\n");
            
        } else 
        {
            SliceDataStorage storage;

            if (!areaGenerator.generateAreas(storage, model, timeKeeper))
                return false;
            
            gcodeWriter.setCommandSocket(commandSocket);
            gcodeWriter.writeGCode(storage, timeKeeper);
        }

        logProgress("process", 1, 1);//Report the GUI that a file has been fully processed.
        log("Total time elapsed %5.2fs.\n", timeKeeperTotal.restart());

        return true;
    }


private:


    
};

}//namespace cura

#endif//FFF_PROCESSOR_H
