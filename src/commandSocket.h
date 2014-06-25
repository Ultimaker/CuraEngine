#ifndef COMMAND_SOCKET_H
#define COMMAND_SOCKET_H

#include "utils/socket.h"
#include "utils/polygon.h"
#include "settings.h"

namespace cura {

class fffProcessor;
class CommandSocket
{
private:
    ClientSocket socket;
    
    int object_count;
    int current_object_number;
public:
    CommandSocket(int portNr);
    
    void handleIncommingData(ConfigSettings* config, fffProcessor* processor);
    
    void sendPolygons(const char* name, int layerNr, int32_t z, Polygons& polygons);
    void sendProgress(float amount);
    void sendPrintTimeForObject(int index, float print_time);
    void sendPrintMaterialForObject(int index, int extruder_nr, float material_amount);
};

}//namespace cura

#endif//COMMAND_SOCKET_H
