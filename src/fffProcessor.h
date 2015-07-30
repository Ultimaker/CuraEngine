#ifndef FFF_PROCESSOR_H
#define FFF_PROCESSOR_H

#include "settings.h"
#include "FffGcodeWriter.h"
#include "FffPolygonGenerator.h"
#include "commandSocket.h"
#include "Weaver.h"
#include "Wireframe2gcode.h"
#include "Progress.h"
#include "utils/gettime.h"

namespace cura {

//FusedFilamentFabrication processor.
class fffProcessor : public SettingsBase
{
private:
    FffPolygonGenerator polygon_generator;
    FffGcodeWriter gcode_writer;
    CommandSocket* command_socket;

public:
    TimeKeeper time_keeper; // TODO: use singleton time keeper
    
    fffProcessor()
    : polygon_generator(this)
    , gcode_writer(this)
    {
        command_socket = NULL;
    }
    
    void resetFileNumber()
    {
        gcode_writer.resetFileNumber();
    }

    void setCommandSocket(CommandSocket* socket)
    {
        command_socket = socket;
        gcode_writer.setCommandSocket(socket);
        polygon_generator.setCommandSocket(socket);
    }

    void sendPolygons(PolygonType type, int layer_nr, Polygons& polygons, int line_width)
    {
        if (command_socket)
            command_socket->sendPolygons(type, layer_nr, polygons, line_width);
    }
    
    bool setTargetFile(const char* filename)
    {
        return gcode_writer.setTargetFile(filename);
    }
    
    void setTargetStream(std::ostream* stream)
    {
        return gcode_writer.setTargetStream(stream);
    }

    double getTotalFilamentUsed(int e)
    {
        return gcode_writer.getTotalFilamentUsed(e);
    }

    double getTotalPrintTime()
    {
        return gcode_writer.getTotalPrintTime();
    }
    
    void finalize()
    {
        gcode_writer.finalize();
    }

    bool processFiles(const std::vector<std::string> &files)
    {
        time_keeper.restart();
        MeshGroup* meshgroup = new MeshGroup(this);
        
        for(std::string filename : files)
        {
            log("Loading %s from disk...\n", filename.c_str());

            FMatrix3x3 matrix;
            if (!loadMeshIntoMeshGroup(meshgroup, filename.c_str(), matrix))
            {
                logError("Failed to load model: %s\n", filename.c_str());
                return false;
            }
        }
        
        meshgroup->finalize();

        log("Loaded from disk in %5.3fs\n", time_keeper.restart());
        return processMeshGroup(meshgroup);
    }
    
    bool processMeshGroup(MeshGroup* meshgroup)
    {
        time_keeper.restart();
        if (!meshgroup)
            return false;

        TimeKeeper time_keeper_total;
        
        if (meshgroup->getSettingBoolean("wireframe_enabled"))
        {
            log("starting Neith Weaver...\n");
                        
            Weaver w(this);
            w.weave(meshgroup, command_socket);
            
            log("starting Neith Gcode generation...\n");
            Wireframe2gcode gcoder(w, gcode_writer.gcode, this);
            gcoder.writeGCode(command_socket);
            log("finished Neith Gcode generation...\n");
            
        } else 
        {
            SliceDataStorage storage;

            if (!polygon_generator.generateAreas(storage, meshgroup, time_keeper))
            {
                return false;
            }
            gcode_writer.setCommandSocket(command_socket);
            
            Progress::messageProgressStage(Progress::Stage::EXPORT, &time_keeper, command_socket);
            gcode_writer.writeGCode(storage, time_keeper);
        }

        Progress::messageProgress(Progress::Stage::FINISH, 1, 1, command_socket); //Report the GUI that a file has been fully processed.
        log("Total time elapsed %5.2fs.\n", time_keeper_total.restart());

        return true;
    }
};

}//namespace cura

#endif//FFF_PROCESSOR_H
