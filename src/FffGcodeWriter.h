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

    void writeGCode(SliceDataStorage& storage, TimeKeeper& timeKeeper);
    
private:
    //Setup the retraction parameters.
    void setConfigRetraction(SliceDataStorage& storage);
    
    void setConfigSkirt(SliceDataStorage& storage, int layer_thickness);
    
    void setConfigSupport(SliceDataStorage& storage, int layer_thickness);
    
    void setConfigInsets(SliceMeshStorage& mesh, int layer_thickness);
    
    void setConfigSkin(SliceMeshStorage& mesh, int layer_thickness);
    
    void setConfigInfill(SliceMeshStorage& mesh, int layer_thickness);
    
    
    void processStartingCode(SliceDataStorage& storage);

    void processNextPrintObjectCode(SliceDataStorage& storage);
    
    void processRaft(SliceDataStorage& storage, unsigned int totalLayers);
    
    void processLayer(SliceDataStorage& storage, unsigned int layer_nr, unsigned int totalLayers, bool has_raft);
    
    void processInitialLayersSpeedup(SliceDataStorage& storage, unsigned int layer_nr);
    
    void processLayerStartPos(unsigned int layer_nr, bool has_raft);
    
    void processSkirt(SliceDataStorage& storage, GCodePlanner& gcodeLayer, unsigned int layer_nr);
    
    void processOozeShield(SliceDataStorage& storage, GCodePlanner& gcodeLayer, unsigned int layer_nr);
    
    std::vector<SliceMeshStorage*> calculateMeshOrder(SliceDataStorage& storage, int current_extruder);
    
    //Add a single layer from a single mesh-volume to the GCode
    void addMeshLayerToGCode(SliceDataStorage& storage, SliceMeshStorage* mesh, GCodePlanner& gcodeLayer, int layer_nr);
    
    void addSupportToGCode(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int layer_nr);
    
    void addWipeTower(SliceDataStorage& storage, GCodePlanner& gcodeLayer, int layer_nr, int prevExtruder);
    
    //Finish the layer by applying speed corrections for minimal layer times and determine the fanSpeed
    void processFanSpeedAndMinimalLayerTime(SliceDataStorage& storage, GCodePlanner& gcodeLayer, unsigned int layer_nr);
    
    void finalize();
};

}//namespace cura

#endif // GCODE_WRITER_H
