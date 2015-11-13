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
#include "utils/NoCopy.h"

#define SHOW_ALL_SETTINGS true

namespace cura {

//FusedFilamentFabrication processor. Singleton class
class FffProcessor : public SettingsBase , NoCopy
{
private:
    static FffProcessor instance; 
    
    FffProcessor()
    : polygon_generator(this)
    , gcode_writer(this)
    , first_meshgroup(true)
    {
        command_socket = NULL;
    }
public:
    static FffProcessor* getInstance() 
    { 
        return &instance; 
    }
    
private:
    FffPolygonGenerator polygon_generator;
    FffGcodeWriter gcode_writer;
    CommandSocket* command_socket; // TODO: replace all refs to command_socket by CommandSocket::getInstance()
    
    bool first_meshgroup;
    
    std::string profile_string = "";
    
    std::string getAllSettingsString(MeshGroup& meshgroup, bool first_meshgroup);
    
public:
    std::string getProfileString() { return profile_string; }
    
    TimeKeeper time_keeper; // TODO: use singleton time keeper
    
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

    bool processFiles(const std::vector<std::string> &files);
    
    bool processMeshGroup(MeshGroup* meshgroup);
};

}//namespace cura

#endif//FFF_PROCESSOR_H
