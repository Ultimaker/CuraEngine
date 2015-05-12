#ifndef GCODE_WRITER_H
#define GCODE_WRITER_H


#include <fstream>
#include "utils/gettime.h"
#include "utils/logoutput.h"
#include "sliceDataStorage.h"
#include "raft.h"
#include "infill.h"
#include "bridge.h"
#include "pathOrderOptimizer.h"
#include "gcodePlanner.h"
#include "gcodeExport.h"
#include "commandSocket.h"
#include "utils/polygonUtils.h"

namespace cura 
{

class FffGcodeWriter : public SettingsBase
{
    friend class fffProcessor; // cause WireFrame2Gcode uses the member [gcode] (TODO)
private:
    int maxObjectHeight;
    int fileNr; //!< used for sequential printing of objects
    GCodeExport gcode;
    CommandSocket* commandSocket;
    std::ofstream output_file;
public:
    FffGcodeWriter(SettingsBase* settings_)
    : SettingsBase(settings_)
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

    
    double getTotalFilamentUsed(int e)
    {
        return gcode.getTotalFilamentUsed(e);
    }

    double getTotalPrintTime()
    {
        return gcode.getTotalPrintTime();
    }
    


    void finalize()
    {
        gcode.finalize(maxObjectHeight, getSettingInMillimetersPerSecond("speed_travel"), getSettingString("machine_end_gcode").c_str());
        for(int e=0; e<MAX_EXTRUDERS; e++)
            gcode.writeTemperatureCommand(e, 0, false);
    }

    void writeGCode(SliceDataStorage& storage, TimeKeeper& timeKeeper);
    
    std::vector<SliceMeshStorage*> calculateMeshOrder(SliceDataStorage& storage, int current_extruder);
    
    //Add a single layer from a single mesh-volume to the GCode
    void addMeshLayerToGCode(SliceDataStorage& storage, SliceMeshStorage* mesh, GCodePlanner& gcodeLayer, int layer_nr);
    
    void addSupportToGCode(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int layer_nr);
    
    void addWipeTower(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int layer_nr, int prevExtruder);
};

}//namespace cura

#endif // GCODE_WRITER_H
